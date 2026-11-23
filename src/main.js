import { createMechanism, createDemo, cloneMechanism, normalizeMechanism, nextId, solveSketch,
  sketchDriven, splitElement, removeObject, distance, isPiston, balanceForce, analyzePath, Simulation } from './mechanics.js';
import { createCanvas } from './canvas.js';
import { setupStudio } from './ui.js';
import { collections, kindName, label, lookup, renderTree, inspectorContent } from './panels.js';

const $ = selector => document.querySelector(selector);
const ui = setupStudio();
const storeKey = 'rod-mechanism-studio.project.v1';
const demoTitle = 'Hatch + gas spring';
let design = createDemo(), name = demoTitle, sim = null, playing = false, frame = 0, lastTick = 0, lastPanel = 0;
let pick = null, tool = 'select', page = null, relation = null, operand = null, cachedPath, menu;
let saveState = 'Saved in this browser', startupHint = '';
const undo = [], redo = [];
const visibility = {grid:true, snap:true, dimensions:true, constraints:true, loads:true, labels:true};

function restoreProject(raw, fallback) {
  if (raw.format && raw.format !== 'rod-mechanism-studio') throw Error('This file is from a different application.');
  if (raw.format && raw.version !== 1) throw Error('This project version is not supported.');
  const model = normalizeMechanism(raw.model ?? raw);
  return { model, name: String(raw.name || fallback || 'Untitled mechanism').slice(0,100) };
}
try {
  const saved = localStorage.getItem(storeKey);
  if (saved) {const p=restoreProject(JSON.parse(saved),demoTitle); design=p.model; name=p.name;}
} catch {
  saveState = 'Local project unavailable';
  startupHint = 'The saved project could not be opened. The hatch example is displayed.';
}
const snapshot = () => ({name, model:cloneMechanism(design)});
const envelope = () => ({format:'rod-mechanism-studio',version:1,name,model:design});
function save() {
  try { localStorage.setItem(storeKey,JSON.stringify(envelope())); saveState='Saved in this browser'; }
  catch { saveState='Local save unavailable · export a copy'; }
  $('#save-state').textContent=saveState;
}
const current = () => sim ? sim.model : design;
const findJoint = (model,id) => lookup(model,{type:'joint',id});
const hint = message => {$('#status-hint').textContent=message;};
function defaultHint() {
  if(sim) return playing ? 'Simulation running. Pause to pose a pin.' : 'Simulation paused. Drag a free pin to inspect another pose.';
  if(tool==='constraint') return kindName(relation)+' · '+(operand?'Select the second object.':'Select a member or the first point.');
  if(tool==='dimension') return 'Select a member or two points, then click to place the dimension.';
  if(tool==='load') return 'Select a joint or a position on a rod or body to apply a point load.';
  if(['pin','fixed','rotation'].includes(tool)) return 'Click the drawing to place a '+kindName(tool).toLowerCase()+'.';
  if(tool==='select') return 'Select a part to inspect it. Ctrl-click a second part for constraints.';
  if(tool==='pan') return 'Drag the drawing to pan. Scroll to zoom.';
  return 'Select the first endpoint for the '+kindName(tool).toLowerCase()+'.';
}
function remember(before) {
  undo.push(before); if(undo.length>80) undo.shift(); redo.length=0;
}
function update(edit, message) {
  if(sim) return;
  const before=snapshot(), oldPick=pick;
  try {
    edit(); remember(before); cachedPath=undefined; save(); render(); hint(message || defaultHint());
  } catch(err) {design=before.model;name=before.name;pick=oldPick;render();hint(err.message);}
}
function travel() {return cachedPath ??= analyzePath(design);}
function select(s) {
  pick = s && lookup(current(),s) ? s : null;
  page=pick?'properties':null; closeContext(); render(); hint(defaultHint());
}
function selectTool(next) {
  if(sim && !['pan','select'].includes(next)) return;
  closeContext(); tool=next; relation=null; operand=null; page=null;
  render(); hint(defaultHint());
}
function itemOf(type, values) {
  const list=design[collections[type]];
  const item={id:nextId(list),name:'',...values}; list.push(item); return item;
}
function resolveEndpoint(ref, construction=false) {
  const id = typeof ref==='object'?ref.id:ref;
  const existing = id!=null && findJoint(design,id);
  if(existing && (existing.kind!=='origin'||construction)) return existing;
  const point=existing||ref;
  if(!point || !Number.isFinite(point.x)||!Number.isFinite(point.y)) throw Error('Select a valid endpoint.');
  const atPoint=design.joints.find(j=>j.kind!=='origin' && Math.hypot(j.x-point.x,j.y-point.y)<1e-7);
  if(atPoint) return atPoint;
  if(point.splitElement!=null) {
    const j=splitElement(design,point.splitElement,point);
    if(j) return j;
  }
  return itemOf('joint',{kind:'pin',x:point.x,y:point.y,mass:0.05,showForce:true});
}
function chooseCreated(type,item) {pick={type,id:item.id}; page='properties';}
function addJoint(ref) {
  update(()=>{const j=resolveEndpoint(ref);j.kind=ref.kind || 'pin';chooseCreated('joint',j);},'Joint placed. Select another location or press Esc.');
}
function addMember({kind,a,b}) {
  update(()=>{
    const start=resolveEndpoint(a),end=resolveEndpoint(b);
    const length=distance(design,start.id,end.id);
    if(length<0.001) throw Error('A component needs two endpoints at least 1 mm apart.');
    const p=itemOf('element',{kind,a:start.id,b:end.id,length,mass:kind==='body'?1:0,
      force:['actuator','gas_spring'].includes(kind)?100:0,forceMode:'push',minLength:Math.max(.001,length*.65),maxLength:length,
      stiffness:0,damping:0,autoReturn:true,torque:kind==='motor'?1:0,showForce:true});
    chooseCreated('element',p);
  },'Component added. Choose endpoints for the next component or press Esc.');
}
function addLoad(data) {
  update(()=>{const j=resolveEndpoint(data.joint);chooseCreated('load',itemOf('load',{...data,kind:'point',joint:j.id,enabled:true}));},'Point load added. Edit its force components in Properties.');
}
function addDimension(data) {
  update(()=>{const a=resolveEndpoint(data.a,true),b=resolveEndpoint(data.b,true);
    chooseCreated('dimension',itemOf('dimension',{...data,a:a.id,b:b.id,driving:true,visible:true}));solveSketch(design);},'Dimension added. Edit its value in Properties.');
}
function arm(model) {
  const p=pick?.type==='element' && lookup(model,pick);
  if(!p||!isPiston(p)) return null;
  const balance=balanceForce(model,p.id); if(!balance) return null;
  const pivot=findJoint(model,balance.pivotId),a=findJoint(model,p.a),b=findJoint(model,p.b);
  const dx=b.x-a.x,dy=b.y-a.y,t=((pivot.x-a.x)*dx+(pivot.y-a.y)*dy)/(dx*dx+dy*dy);
  return {pivot,foot:{x:a.x+dx*t,y:a.y+dy*t},linePoint:a};
}
const drawing=createCanvas($('#drawing'),{
  onSelect:select,onAddJoint:addJoint,onAddElement:addMember,onAddLoad:addLoad,onAddDimension:addDimension,
  onMoveJoint(id,p) {
    if(sketchDriven(design,id)) {hint('This point is driven. Edit its dimension or constraint.');return;}
    update(()=>{Object.assign(findJoint(design,id),p);solveSketch(design,{draggedJointId:id});},'Joint position updated.');
  },
  onPoseJoint(id,p) {if(sim&&!playing){sim.poseJoint(id,p.x,p.y);renderDrawing();renderPanel();}},
  onView(view) {ui.setZoom(Math.round(view.scale/240*100));},
  onConstrainPair:contextFor,onConstraintPick:constraintOperand,onHint:hint,onCancel:()=>selectTool('select')
});

function renderDrawing() {
  const m=current(); let forces=[];
  if(sim && pick?.type==='joint') {
    const j=lookup(m,pick);
    if(j) forces=[{joint:j.id,fx:j.fx,fy:j.fy},{joint:j.id,fx:j.reactionX,fy:j.reactionY}];
  }
  drawing.render({model:m,selection:pick,tool,grid:visibility.grid,snap:visibility.snap,gridSize:.025,
    showIds:visibility.labels,showDimensions:visibility.dimensions,showLoads:visibility.loads,showConstraints:visibility.constraints,
    readOnly:!!sim,poseable:!!sim&&!playing,traces:sim?.tracing?sim.traces:null,forces,momentArm:arm(m),
    canMoveJoint:id=>!sketchDriven(design,typeof id==='object'?id.id:id)});
}
function renderPanel() {
  if(!page){ui.setPanel(null);return;}
  const data=inspectorContent({model:current(),pick,page,simulation:!!sim,path:page==='results'?travel():null,time:sim?.time,playing,trace:sim?.tracing});
  ui.setPanel(data.title);$('#inspector').replaceChildren(...data.blocks);
}
function render() {
  const m=current();
  ui.setMode(sim?'simulation':'model');
  $('#project-name').value=name; $('#project-name').disabled=!!sim; $('#save-state').textContent=saveState;
  ui.setTool(tool,relation);
  for(const b of document.querySelectorAll('[data-action="undo"]')) b.disabled=!!sim||!undo.length;
  for(const b of document.querySelectorAll('[data-action="redo"]')) b.disabled=!!sim||!redo.length;
  for(const b of document.querySelectorAll('[data-action="trace"]')) b.setAttribute('aria-pressed',String(!!sim?.tracing));
  ui.setPlaying(playing);
  $('#model-status').textContent=sim?(playing?'Running':'Paused')+' · '+sim.time.toFixed(2)+' s':m.elements.length+' components · '+m.joints.filter(j=>j.kind!=='origin').length+' joints';
  renderTree($('#model-tree'),m,pick); renderDrawing(); renderPanel();
}
function stop() { playing=false; cancelAnimationFrame(frame); lastTick=0; }
function switchMode(mode) {
  if((mode==='simulation')===!!sim) return;
  stop(); closeContext(); sim=mode==='simulation'?new Simulation(design):null;
  pick=pick && lookup(current(),pick)?pick:null; tool='select'; relation=null; operand=null; page=null;
  render(); hint(defaultHint());
}
function tick(now) {
  if(!playing||!sim) return;
  if(lastTick) sim.step(Math.min((now-lastTick)/1000,.08));
  lastTick=now;renderDrawing();
  if(now-lastPanel>150){$('#model-status').textContent='Running · '+sim.time.toFixed(2)+' s';renderPanel();lastPanel=now;}
  frame=requestAnimationFrame(tick);
}
function replaceProject(p) {
  if(sim) switchMode('model');
  update(()=>{design=p.model;name=p.name;pick=null;page=null;tool='select';relation=null;operand=null;},'Project opened.');drawing.fit();
}
function history(back) {
  if(sim) return;
  const source=back?undo:redo, target=back?redo:undo;
  if(!source.length) return;
  target.push(snapshot()); const previous=source.pop();
  design=previous.model;name=previous.name;pick=null;page=null;cachedPath=undefined;save();render();hint(back?'Last edit undone.':'Edit restored.');
}
function exportFile() {
  const blob=new Blob([JSON.stringify(envelope(),null,2)],{type:'application/json'});
  const url=URL.createObjectURL(blob),a=document.createElement('a');a.href=url;
  a.download=(name.trim().replace(/[^\p{L}\p{N}._ -]/gu,'-') || 'mechanism')+'.json'; a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);
}
function closeContext(){menu?.remove();menu=null;}
function validRelations(first,second) {
  if(!first || !second) return [];
  if(first.type==='joint' && second.type==='joint' && first.id!==second.id) return ['coincident','horizontal','vertical'];
  if(first.type==='element' && second.type==='element' && first.id!==second.id) return ['parallel','equal_length'];
  return [];
}
function contextFor(first,second,point) {
  closeContext(); const kinds=validRelations(first,second);
  if(!kinds.length){hint('Select two joints, or two members for parallel or equal length.');return;}
  menu=document.createElement('div');menu.className='constraint-popup';menu.setAttribute('role','group');menu.setAttribute('aria-label','Available constraints');
  for(const kind of kinds){const b=document.createElement('button');b.textContent=kindName(kind);b.onclick=()=>{closeContext();applyConstraint(kind,first,second);};menu.append(b);}
  document.body.append(menu);
  menu.style.left=Math.max(0,Math.min(point.x,innerWidth-menu.offsetWidth-8))+'px';menu.style.top=Math.max(0,Math.min(point.y,innerHeight-menu.offsetHeight-8))+'px';
  menu.firstElementChild.focus();
}
function applyConstraint(kind,first,second) {
  const f=lookup(design,first),s=lookup(design,second);
  if(!f) return;
  update(()=>{
    let endpoints;
    if(first.type==='element') endpoints={a:f.a,b:f.b,c:s?.a,d:s?.b};
    else if(s) endpoints={a:f.id,b:s.id};
    else throw Error('Select two joints for this constraint.');
    if(kind==='coincident' && first.type==='element') throw Error('Select two distinct joints to make them coincident.');
    const c=itemOf('constraint',{kind,...endpoints,visible:true});
    solveSketch(design);chooseCreated('constraint',c);tool='select';relation=null;operand=null;
  },'Sketch constraint applied.');
}
function startConstraint(kind) {
  if(sim)return;
  if(pick?.type==='element'&&['horizontal','vertical'].includes(kind)){applyConstraint(kind,pick,null);return;}
  tool='constraint';relation=kind;operand=pick && (['parallel','equal_length'].includes(kind)?pick.type==='element':pick.type==='joint')?pick:null;
  page=null; render();hint(defaultHint());
}
function constraintOperand(s) {
  if(!s)return;
  if(s.type==='element' && ['horizontal','vertical'].includes(relation) && !operand) {applyConstraint(relation,s,null);return;}
  const expected=['parallel','equal_length'].includes(relation)?'element':'joint';
  if(s.type!==expected){hint('Choose a '+expected+' for this constraint.');return;}
  if(!operand){operand=s;pick=s;render();hint(defaultHint());return;}
  if(operand.id!==s.id) applyConstraint(relation,operand,s);
}
function fieldChange(input) {
  if(sim)return;
  const key=input.dataset.field;
  let value=input.type==='checkbox'?input.checked:input.type==='number'?input.valueAsNumber:input.value;
  if(input.type==='number' && (!input.checkValidity()||!Number.isFinite(value))){input.reportValidity();renderPanel();hint('Enter a value within the displayed field limits.');return;}
  if(['value','minLength','maxLength','offset','length','x','y'].includes(key)) value/=1000;
  update(()=>{
    if(page==='motion'){design.options[key]=value;return;}
    const item=lookup(design,pick);if(!item)throw Error('Select an object first.');
    if((key==='minLength'&&value>item.maxLength)||(key==='maxLength'&&value<item.minLength)) throw Error('Extended length must be at least the compressed length.');
    if(key==='length'){
      const d=design.dimensions.find(d=>d.kind==='distance' && d.driving && ((d.a===item.a && d.b===item.b)||(d.a===item.b&&d.b===item.a)));
      if(d)d.value=value;else itemOf('dimension',{kind:'distance',a:item.a,b:item.b,value,offset:.12,driving:true,visible:true});
    }else item[key]=value;
    if(['length','value','x','y','driving'].includes(key)) solveSketch(design);
  },'Properties updated.');
}
function useForce(act,button) {
  const p=button?.dataset.id!=null?lookup(design,{type:'element',id:button.dataset.id}):lookup(design,pick);
  if(!p||!isPiston(p))return;
  const bounds=act==='use-peak' && travel().forceRanges[p.id];
  const force=act==='use-peak'?(bounds && bounds.reduce((a,b)=>Math.abs(a)>Math.abs(b)?a:b)):balanceForce(design,p.id)?.force;
  if(!Number.isFinite(force)){hint('No force estimate is available for this component.');return;}
  update(()=>{p.force=Math.abs(force);p.forceMode=force<0?'pull':'push';chooseCreated('element',p);},'Rated force and direction updated.');
}
function action(act,b) {
  if(['undo','redo'].includes(act)) return history(act==='undo');
  if(act==='new'||act==='example') return replaceProject(act==='new'?{model:createMechanism(),name:'Untitled mechanism'}:{model:createDemo(),name:demoTitle});
  if(act==='open') return $('#project-file').click();
  if(act==='export') return exportFile();
  if(act==='save'){save();hint(saveState);return;}
  if(act==='fit')return drawing.fit();
  if(act==='zoom-in'||act==='zoom-out')return drawing.zoom(act==='zoom-in'?1.25:.8);
  if(act==='results'||act==='force-range'){page='results';render();return;}
  if(act==='pick-piston'){const p=lookup(current(),{type:'element',id:b.dataset.id});if(p)select({type:'element',id:p.id});return;}
  if(act==='close-panel'){page=null;return;}
  if(act==='play'&&sim){if(playing)stop();else{playing=true;frame=requestAnimationFrame(tick);}render();hint(defaultHint());return;}
  if(act==='step'&&sim){stop();sim.step(1/60);render();hint('Advanced one simulation frame.');return;}
  if(act==='reset'&&sim){stop();sim.reset();render();hint('Restored the design pose.');return;}
  if(act==='trace'&&sim){sim.setTracing(!sim.tracing);render();return;}
  if(act==='clear-traces'&&sim){sim.clearTraces();renderDrawing();return;}
  if(sim)return;
  if(act==='use-required'||act==='use-peak')return useForce(act,b);
  if(act==='delete'&&pick){if(lookup(design,pick)?.kind==='origin')return;update(()=>{removeObject(design,pick.type,pick.id);pick=null;page=null;},'Selection deleted. Undo restores the edit.');}
}
document.addEventListener('click',event=>{
  const b=event.target.closest('button'); if(!b)return;
  if(b.dataset.pick){const type=b.dataset.pick,id=lookup(current(),{type,id:b.dataset.id})?.id;if(id!=null){const s={type,id};if(tool==='constraint')constraintOperand(s);else if((event.ctrlKey||event.metaKey)&&pick)contextFor(pick,s,{x:event.clientX,y:event.clientY});else select(s);}return;}
  if(b.dataset.mode)return switchMode(b.dataset.mode);
  if(b.dataset.tool)return selectTool(b.dataset.tool);
  if(b.dataset.constraint)return startConstraint(b.dataset.constraint);
  if(b.dataset.panel && b.dataset.panel!=='tree'){page=b.dataset.panel==='structure'?'properties':b.dataset.panel;render();return;}
  if(b.dataset.action)action(b.dataset.action,b);
});
document.addEventListener('pointerdown',e=>{if(menu&&!menu.contains(e.target))closeContext();});
document.addEventListener('change',e=>{
  const el=e.target;
  if(el.dataset.field) return fieldChange(el);
  if(el.dataset.visibility){visibility[el.dataset.visibility]=el.checked;renderDrawing();}
});
$('#project-name').addEventListener('change',e=>{const next=e.target.value.trim()||'Untitled mechanism';if(next!==name)update(()=>{name=next;},'Project renamed.');});
$('#project-file').addEventListener('change',async e=>{
  const input=e.target,file=input.files[0];if(!file)return;
  try {const raw=JSON.parse((await file.text()).replace(/^\uFEFF/,''));replaceProject(restoreProject(raw,file.name.replace(/\.json$/i,'')));}
  catch(err){hint('Could not open project. '+err.message);}
  finally{input.value='';}
});
document.addEventListener('keydown',e=>{
  if(e.target.closest('input,select,textarea,dialog')||e.altKey)return;
  const key=e.key.toLowerCase(),ctrl=e.ctrlKey||e.metaKey;
  if(ctrl && ['s','o','z','y','0'].includes(key)){e.preventDefault();action(({s:'save',o:'open',z:e.shiftKey?'redo':'undo',y:'redo','0':'fit'})[key]);return;}
  if(ctrl)return;
  if(key==='f5'){e.preventDefault();switchMode('simulation');}
  else if(key==='escape'){selectTool('select');}
  else if(key==='delete')action('delete');
  else if(key==='d')selectTool('dimension');
  else if(key==='v')selectTool('select');
  else if(key==='h')selectTool('pan');
  else if(key===' '&&sim&&!e.target.closest('button,svg')){e.preventDefault();action('play');}
});
document.addEventListener('visibilitychange',()=>{if(document.hidden&&playing){stop();render();hint('Simulation paused.');}});
render();hint(startupHint || defaultHint());
