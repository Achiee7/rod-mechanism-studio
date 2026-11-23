import { createMechanism, createDemo, cloneMechanism, normalizeMechanism, nextId, solveSketch,
  sketchDriven, splitElement, removeObject, distance, isPiston, balanceForce, assistForce, currentForces, analyzePath, poseValidity, Simulation } from './mechanics.js';
import { createCanvas, pistonGeometry } from './canvas.js';
import { setupStudio } from './ui.js';
import { collections, kindName, label, lookup, renderTree, inspectorContent, updateAnalysisCursor } from './panels.js';
import { PlaybackClock, translateSelection, pathCsv } from './workspace-tools.js';

const $ = selector => document.querySelector(selector);
const ui = setupStudio();
const storeKey = 'rod-mechanism-studio.project.v1';
const demoTitle = 'Hatch + gas spring';
let design = createDemo(), name = demoTitle, sim = null, playing = false, frame = 0, lastPanel = 0;
let mode = 'model', suspendedSim = null, speed = 1, physicsMode = 'real', selections = [];
let measured = false;
const clock = new PlaybackClock();
function freshAnalysis(study) {
  const valid=Number.isFinite(study?.minAngle) && Number.isFinite(study?.maxAngle) && study.minAngle>=-360 && study.maxAngle<=360 && study.minAngle<study.maxAngle;
  return {driverId:study?.driverId ?? null,pistonId:study?.pistonId ?? null,memberId:null,sampleIndex:null,previewIndex:null,metric:'force',
    minAngle:valid?study.minAngle:-80,maxAngle:valid?study.maxAngle:80};
}
const hatchStudy={minAngle:-22,maxAngle:0};
let analysis = freshAnalysis(hatchStudy), previewModel = null, forceReport = null;
const forceOverrides = new Map();
const forcePolicy = {model:false, simulation:true, analysis:true};
let pick = null, tool = 'select', page = null, relation = null, operand = null, cachedPath, menu;
let saveState = 'Saved in this browser', startupHint = '';
const undo = [], redo = [];
const visibility = {grid:true,snap:true,dimensions:true,constraints:true,labels:true,pointLoads:true,
  excursion:true,applied:true,members:true,pistons:true,motors:true,stops:true,reactions:true,resultants:true,assist:true};

function restoreProject(raw, fallback) {
  if (raw.format && raw.format !== 'rod-mechanism-studio') throw Error('This file is from a different application.');
  if (raw.format && raw.version !== 1) throw Error('This project version is not supported.');
  const model = normalizeMechanism(raw.model ?? raw);
  return { model, name: String(raw.name || fallback || 'Untitled mechanism').slice(0,100), study:raw.study,
    physicsMode:raw.physicsMode === 'forced' ? 'forced' : 'real' };
}
try {
  const saved = localStorage.getItem(storeKey);
  if (saved) {const p=restoreProject(JSON.parse(saved),demoTitle); design=p.model; name=p.name;analysis=freshAnalysis(p.study);physicsMode=p.physicsMode;}
} catch {
  saveState = 'Local project unavailable';
  startupHint = 'The saved project could not be opened. The hatch example is displayed.';
}
const studySettings = () => ({driverId:analysis.driverId,pistonId:analysis.pistonId,minAngle:analysis.minAngle,maxAngle:analysis.maxAngle});
const snapshot = () => ({name, model:cloneMechanism(design),study:studySettings(),physicsMode});
const envelope = () => ({format:'rod-mechanism-studio',version:1,name,model:design,study:studySettings(),physicsMode});
let checkpoint = snapshot();
try {
  const savedCheckpoint = localStorage.getItem(storeKey + '.checkpoint');
  if (savedCheckpoint) checkpoint = restoreProject(JSON.parse(savedCheckpoint));
  const storedSpeed = Number(localStorage.getItem(storeKey + '.speed'));
  if ([0.1,0.25,0.5,1,2,4].includes(storedSpeed)) speed = storedSpeed;
} catch { /* The current design remains available in memory. */ }
function checkpointDesign() {
  checkpoint = snapshot();
  try { localStorage.setItem(storeKey + '.checkpoint', JSON.stringify(checkpoint)); }
  catch { /* The in-session checkpoint remains usable. */ }
}
function save() {
  try { localStorage.setItem(storeKey,JSON.stringify(envelope())); saveState='Saved in this browser'; }
  catch { saveState='Local save unavailable · export a copy'; }
  $('#save-state').textContent=saveState;
}
const current = () => mode === 'analysis' && previewModel ? previewModel : sim?.model ?? design;
const findJoint = (model,id) => lookup(model,{type:'joint',id});
const hint = message => {$('#status-hint').textContent=message;};
function defaultHint() {
  if(mode==='analysis') return 'Select a sample on the graph. Reachable samples move the drawing.';
  if(sim && !sim.validity.valid) return sim.validity.issues[0]?.message || 'Repair the starting pose in Model.';
  if(sim) return physicsMode==='forced' ? 'Forced motion drives the pistons through their available travel. Overloads are marked on the drawing.'
    : 'Real physics applies the installed forces. An opposing load can hold or backdrive a piston.';
  if(tool==='constraint') return kindName(relation)+' · '+(operand?'Select the second object.':'Select a member or the first point.');
  if(tool==='dimension') return 'Select a member or two points, then click to place the dimension.';
  if(tool==='load') return 'Select a joint or a position on a rod or body to apply a point load.';
  if(['pin','fixed','rotation'].includes(tool)) return 'Click the drawing to place a '+kindName(tool).toLowerCase()+'.';
  if(tool==='select') return 'Click a part or drag a selection box. Shift adds parts. Ctrl-click a pair for constraints.';
  if(tool==='pan') return 'Drag the drawing to pan. Scroll to zoom.';
  return 'Select the first endpoint for the '+kindName(tool).toLowerCase()+'.';
}
function remember(before) {
  undo.push(before); if(undo.length>80) undo.shift(); redo.length=0;
}
function update(edit, message, {opening=false}={}) {
  if(sim) return;
  const before=snapshot(), oldPick=pick, oldSelections=selections.slice();
  try {
    edit();
    if(!opening) checkEditedMounts(before.model,design);
    remember(before); invalidateDesign(); save(); render(); hint(message || defaultHint());
  } catch(err) {design=before.model;name=before.name;pick=oldPick;selections=oldSelections;invalidateDesign();render();hint(err.message);}
}
function checkEditedMounts(before,after) {
  for(const part of after.elements.filter(isPiston)) {
    const problem=pistonGeometry(part,distance(after,part.a,part.b)).problem;
    if(!problem) continue;
    const old=before.elements.find(p=>p.id===part.id && isPiston(p));
    // Imported projects may need several edits before a damaged installation fits.
    if(old && pistonGeometry(old,distance(before,old.a,old.b)).problem) continue;
    throw Error(label(part)+' · '+problem+'. Edit the stroke limits or mounting points.');
  }
}
function clearPreview() {
  previewModel=null;analysis.previewIndex=null;analysis.sampleIndex=null;forceReport=null;
}
function invalidateDesign() {
  cachedPath = undefined; forceReport = null; suspendedSim = null;
  clearPreview();
}
function travel() {
  return cachedPath ??= analyzePath(design,{driverId:analysis.driverId,minAngle:analysis.minAngle,maxAngle:analysis.maxAngle});
}
const samePick = (a,b) => a?.type === b?.type && a?.id === b?.id;
function setSelection(items) {
  selections = items.filter(item => lookup(current(), item));
  pick = selections.at(-1) || null;
}
function select(s) {
  setSelection(s ? [s] : []);
  forceReport=null;
  if(mode==='analysis' && pick?.type==='element') {
    const part=lookup(current(),pick);
    if(isPiston(part)) analysis.pistonId=part.id;
    else { analysis.memberId=part.id; analysis.metric='member'; }
  }
  page=mode==='analysis'?'results':pick?'properties':null;
  closeContext(); render(); hint(defaultHint());
}
function selectMany(items, {additive=false,toggle=false}={}) {
  const next = additive ? [...selections] : [];
  for(const item of items) {
    const i = next.findIndex(s => samePick(s,item));
    if(i === -1) next.push(item);
    else if(toggle) next.splice(i,1);
  }
  setSelection(next); closeContext();
  page=mode==='analysis'?'results':pick?'properties':null;
  render();hint(selections.length ? selections.length+' selected.' : defaultHint());
}
function selectTool(next) {
  if(mode!=='model' && !['pan','select'].includes(next)) return;
  closeContext(); tool=next; relation=null; operand=null; page=mode==='analysis'?'results':null;
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
function chooseCreated(type,item) {setSelection([{type,id:item.id}]); page='properties';}
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
      stiffness:0,damping:0,autoReturn:kind==='actuator',barrelEnd:'a',torque:kind==='motor'?1:0,showForce:true});
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
  const balance=balanceForce(model,p.id); if(!balance || balance.lineOfAction===false) return null;
  const pivot=findJoint(model,balance.pivotId),a=findJoint(model,p.a),b=findJoint(model,p.b);
  const dx=b.x-a.x,dy=b.y-a.y,t=((pivot.x-a.x)*dx+(pivot.y-a.y)*dy)/(dx*dx+dy*dy);
  return {pivot,foot:{x:a.x+dx*t,y:a.y+dy*t},linePoint:a};
}
const drawing=createCanvas($('#drawing'),{
  onSelect:select,onSelectMany:selectMany,onAddJoint:addJoint,onAddElement:addMember,onAddLoad:addLoad,onAddDimension:addDimension,
  onMoveSelection(items,delta) {
    if(mode==='model') update(()=>translateSelection(design,items,delta),'Selection moved.');
  },
  onMoveDimension(id,offset) {
    if(mode==='model') update(()=>{lookup(design,{type:'dimension',id}).offset=offset;},'Dimension repositioned.');
  },
  onMoveJoint(id,p) {
    if(sketchDriven(design,id)) {hint('This point is driven. Edit its dimension or constraint.');return;}
    update(()=>{Object.assign(findJoint(design,id),p);solveSketch(design,{draggedJointId:id});},'Joint position updated.');
  },
  onPoseJoint(id,p) {
    if(sim&&!playing) {
      const result=sim.poseJoint(id,p.x,p.y); measured=false; forceReport=null; renderDrawing();renderPanel();updateStatus();
      if(result?.accepted===false)hint(sim.motionMessage || defaultHint());
    }
  },
  onPointer(p) {
    $('#pointer-position').textContent = p ? Math.round(p.x*1000)+', '+Math.round(p.y*1000)+' mm' : '';
  },
  onView(view) {ui.setZoom(Math.round(view.scale/240*100));},
  onConstrainPair:contextFor,onConstraintPick:constraintOperand,onHint:hint,onCancel:cancelTool
});

function cancelTool() {
  setSelection([]);
  selectTool('select');
}
function forceDisplay(type,id) {
  if(mode!=='model' && forceOverrides.has(type+':'+id)) return forceOverrides.get(type+':'+id);
  return lookup(current(),{type,id})?.showForce !== false;
}
function inspectForces() {
  if(forceReport) return forceReport;
  const model = current();
  forceReport = currentForces(model,{dynamic:Boolean(sim && (measured || physicsMode==='forced')),validity:sim?.validity});
  const piston = model.elements.find(p=>isPiston(p) && p.id===(analysis.pistonId ?? (pick?.type==='element'?pick.id:null))) ?? model.elements.find(isPiston);
  if(piston && visibility.assist) {
    // Prefer the constrained virtual-work solution whenever a sweep sample is displayed.
    const sample = mode==='analysis' && travel().poses?.[analysis.previewIndex];
    const atPose = sample?.pistons?.[piston.id];
    const effort = atPose ? {handForce:atPose.assistForce,handleId:atPose.handleId} : assistForce(model,piston.id);
    if(effort?.handleId != null && Number.isFinite(effort.handForce))
      forceReport.arrows.push({kind:'assist',joint:effort.handleId,element:piston.id,fx:0,fy:effort.handForce});
  }
  return forceReport;
}
function renderDrawing() {
  const m=current();
  const forces=forcePolicy[mode] || mode!=='model' ? inspectForces() : null;
  drawing.render({model:m,selection:pick,selectionItems:selections,tool,grid:visibility.grid,snap:visibility.snap,gridSize:.025,
    showIds:visibility.labels,showDimensions:visibility.dimensions,showConstraints:visibility.constraints,
    showPointLoads:visibility.pointLoads,showForces:forcePolicy[mode],showAppliedForces:visibility.applied,
    showReactions:visibility.reactions,showMemberForces:visibility.members,showMotorTorques:visibility.motors,
    showPistonForces:visibility.pistons,showConstraintForces:visibility.stops,showAssistForces:visibility.assist,
    showResultantForces:visibility.resultants,forces:forcePolicy[mode]?forces:[],forceDisplay,pistonLoads:forces?.pistons,
    readOnly:mode!=='model',poseable:!!sim&&sim.validity.valid&&!playing,traces:sim?.tracing?sim.traces:null,
    showExcursion:mode==='analysis' && visibility.excursion,excursionPoses:mode==='analysis'?travel().poses:null,
    momentArm:mode==='analysis'?null:arm(m),
    canMoveJoint:id=>!sketchDriven(design,typeof id==='object'?id.id:id)});
}
function panelInputs() {
  const path = mode==='analysis' ? travel() : null;
  return {model:current(),design,pick,page,selections,simulation:!!sim,path,forces:inspectForces(),
    analysis:{...analysis,forceDisplay},physicsMode,time:sim?.time,playing,trace:sim?.tracing};
}
function renderPanel() {
  if(!page){ui.setPanel(null);return;}
  const data=inspectorContent(panelInputs());
  ui.setPanel(data.title);$('#inspector').replaceChildren(...data.blocks);
  if(sim && !sim.validity.valid) {
    for(const b of $('#inspector').querySelectorAll('[data-action="play"],[data-action="step"]')) b.disabled=true;
  }
}
function updatePistonAlert() {
  const button=$('#piston-alert');
  let message='';
  if(sim) {
    if(!sim.validity.valid) message='Invalid starting pose · Repair in Model';
    else {
      const reports=inspectForces().pistons;
      const overloaded=sim.model.elements.filter(p=>isPiston(p) && reports?.[p.id]?.overload===true);
      if(overloaded.length) message=overloaded.length===1 ? label(overloaded[0])+' overloaded · View forces' : overloaded.length+' pistons overloaded · View forces';
    }
  }
  if(button.textContent!==message)button.textContent=message;
  button.hidden=!message;
}
function updateStatus() {
  updatePistonAlert();
  if(mode==='analysis') {
    const path=travel(), pose=path.poses[analysis.previewIndex],cursor=path.poses[analysis.sampleIndex];
    $('#model-status').textContent=cursor && !cursor.feasible ? 'Unreachable sample · drawing at '+(pose ? pose.angleDeg.toFixed(1)+'°':'design pose') :
      pose ? pose.angleDeg.toFixed(1)+'° · Reachable pose' :
      path.poses.filter(p=>p.feasible).length+' reachable / '+path.poses.length+' samples';
  } else if(sim) {
    $('#model-status').textContent=(physicsMode==='forced'?'Forced motion':'Real physics')+' · '+(playing?'Running':'Paused')+' · '+sim.time.toFixed(2)+' s · '+speed+'×';
  } else {
    $('#model-status').textContent=design.elements.length+' components · '+design.joints.filter(j=>j.kind!=='origin').length+' joints';
  }
}
function render() {
  const m=current();
  ui.setMode(mode);
  $('#project-name').value=name; $('#project-name').disabled=mode!=='model'; $('#save-state').textContent=saveState;
  ui.setTool(tool,relation);
  for(const b of document.querySelectorAll('[data-action="undo"]')) b.disabled=mode!=='model'||!undo.length;
  for(const b of document.querySelectorAll('[data-action="redo"]')) b.disabled=mode!=='model'||!redo.length;
  for(const b of document.querySelectorAll('[data-action="trace"]')) b.setAttribute('aria-pressed',String(!!sim?.tracing));
  $('[data-action="excursion"]').setAttribute('aria-pressed',String(visibility.excursion));
  $('[data-visibility="loads"]').checked=forcePolicy[mode];
  $('#playback-speed').value=String(speed).replace(/^0\./,'.');
  for(const radio of document.querySelectorAll('input[name="physics-mode"]')) radio.checked=radio.value===physicsMode;
  const reverse=$('[data-action="reverse-travel"]');
  reverse.hidden=!(sim && physicsMode==='forced' && sim.pistons.length);
  reverse.disabled=!!sim && !sim.validity.valid;
  reverse.setAttribute('aria-pressed',String(!!sim?.forcedReversed));
  for(const b of document.querySelectorAll('#command-ribbon [data-action="play"],#command-ribbon [data-action="step"]')) b.disabled=!!sim&&!sim.validity.valid;
  ui.setPlaying(playing);
  updateStatus();renderTree($('#model-tree'),m,pick,selections); renderDrawing(); renderPanel();
}
function stop() { playing=false; cancelAnimationFrame(frame); clock.reset(); }
function switchMode(next) {
  if(mode===next) return;
  stop(); closeContext();
  if(sim && next==='analysis') suspendedSim={sim,measured};
  if(next==='model') suspendedSim=null;
  sim=next==='simulation'?suspendedSim?.sim ?? new Simulation(design):null;
  if(sim) sim.physicsMode=physicsMode;
  measured=sim ? suspendedSim?.measured ?? false : false;
  if(next==='simulation') suspendedSim=null;
  mode=next; clearPreview();
  setSelection(selections);
  if(mode==='analysis' && pick?.type==='element' && isPiston(lookup(design,pick))) analysis.pistonId=pick.id;
  tool='select'; relation=null; operand=null; page=mode==='analysis'?'results':null;
  render(); hint(defaultHint());
}
function tick(now) {
  if(!playing||!sim) return;
  const previous=sim.time;
  const result=clock.advance(now,speed,sim);
  if(result?.accepted===false){stop();forceReport=null;render();hint(sim.motionMessage || defaultHint());return;}
  if(sim.time!==previous){measured=true;forceReport=null;renderDrawing();}
  if(now-lastPanel>150){
    updateStatus();
    if(!$('#inspector').contains(document.activeElement))renderPanel();
    lastPanel=now;
  }
  frame=requestAnimationFrame(tick);
}
function replaceProject(p) {
  if(mode!=='model') switchMode('model');
  update(()=>{
    design=p.model;name=p.name;setSelection([]);page=null;tool='select';relation=null;operand=null;
    analysis=freshAnalysis(p.study);physicsMode=p.physicsMode==='forced'?'forced':'real';forceOverrides.clear();
  },'Project opened.',{opening:true});
  checkpointDesign();drawing.fit();
}
function history(back) {
  if(mode!=='model') return;
  const source=back?undo:redo, target=back?redo:undo;
  if(!source.length) return;
  target.push(snapshot()); const previous=source.pop();
  design=previous.model;name=previous.name;physicsMode=previous.physicsMode??'real';analysis=freshAnalysis(previous.study);forceOverrides.clear();setSelection([]);
  page=null;invalidateDesign();save();render();hint(back?'Last edit undone.':'Edit restored.');
}
function download(content,type,suffix) {
  const blob=new Blob([content],{type});
  const url=URL.createObjectURL(blob),a=document.createElement('a');a.href=url;
  a.download=(name.trim().replace(/[^\p{L}\p{N}._ -]/gu,'-') || 'mechanism')+suffix; a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);
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
  if(mode!=='model')return;
  if(pick?.type==='element'&&['horizontal','vertical'].includes(kind)){applyConstraint(kind,pick,null);return;}
  tool='constraint';relation=kind;operand=pick && (['parallel','equal_length'].includes(kind)?pick.type==='element':pick.type==='joint')?pick:null;
  page=null; render();hint(defaultHint());
}
function constraintOperand(s) {
  if(!s)return;
  if(s.type==='element' && ['horizontal','vertical'].includes(relation) && !operand) {applyConstraint(relation,s,null);return;}
  const expected=['parallel','equal_length'].includes(relation)?'element':'joint';
  if(s.type!==expected){hint('Choose a '+expected+' for this constraint.');return;}
  if(!operand){operand=s;setSelection([s]);render();hint(defaultHint());return;}
  if(operand.id!==s.id) applyConstraint(relation,operand,s);
}
function fieldChange(input) {
  const key=input.dataset.field;
  let value=input.type==='checkbox'?input.checked:input.type==='number'?input.valueAsNumber:input.value;
  if(input.type==='number' && (!input.checkValidity()||!Number.isFinite(value))){input.reportValidity();renderPanel();hint('Enter a value within the displayed field limits.');return;}
  if(['value','minLength','maxLength','offset','length','x','y'].includes(key)) value/=1000;
  if(sim && page==='motion' && ['gravity','gravityEnabled','iterations','damping'].includes(key)) {
    sim.model.options[key]=value;
    sim.refreshForces(physicsMode==='real' && sim.validity.valid);measured=false;forceReport=null;render();hint('Simulation settings updated for this run.');
    return;
  }
  if(mode!=='model')return;
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
  const recommendation=act==='use-required'?null:travel().recommendations?.[p.id];
  if(act==='use-path-lengths') {
    const minLength=recommendation?.minLength, maxLength=recommendation?.maxLength;
    const complete=travel().poses.length && travel().poses.every(pose=>Number.isFinite(pose.pistons?.[p.id]?.length));
    const problem=pistonGeometry({minLength,maxLength},distance(design,p.a,p.b)).problem;
    if(!complete || !(maxLength>minLength) || problem) {
      hint(problem || 'A single-stage cylinder cannot be sized from these stroke bounds.');return;
    }
    update(()=>{p.minLength=minLength;p.maxLength=maxLength;},'Piston stroke limits updated.');
    return;
  }
  if(act==='use-peak' && (recommendation?.status!=='ok' || recommendation.mode==='both')) {
    hint('This sweep does not give a complete single-direction force rating.');return;
  }
  const force=act==='use-peak'?recommendation?.signedForce:balanceForce(design,p.id)?.force;
  if(!Number.isFinite(force)){hint('No force estimate is available for this component.');return;}
  update(()=>{p.force=Math.abs(force);p.forceMode=force<0?'pull':'push';if(mode==='model')chooseCreated('element',p);},'Rated force and direction updated.');
}
function samplePose(index) {
  const sample=travel().poses[index];
  if(!sample) return;
  analysis.sampleIndex=index;
  if(!sample.feasible || !sample.positions) return;
  const candidate={...design,joints:design.joints.map(j=>({...j,...sample.positions[j.id]}))};
  if(!poseValidity(candidate).valid) return;
  analysis.previewIndex=index;
  previewModel=candidate;
  forceReport=null;
}
function analysisChange(input, continuous=false) {
  if(mode!=='analysis')return;
  const key=input.dataset.analysis;
  if(key==='sampleIndex'){
    samplePose(Number(input.value));
    if(continuous) {
      renderDrawing();updateStatus();updateAnalysisCursor($('#inspector'),panelInputs());
      return;
    }
  } else if(key==='minAngle'||key==='maxAngle') {
    const value=input.valueAsNumber;
    if(!input.checkValidity() || !Number.isFinite(value) || (key==='minAngle'?value>=analysis.maxAngle:value<=analysis.minAngle)){
      hint('Enter a sweep start angle smaller than the end angle.');renderPanel();return;
    }
    analysis[key]=value;cachedPath=undefined;
    clearPreview();
  } else if(key==='driverId') {
    analysis.driverId=travel().drivers.find(driver=>String(driver.id)===input.value)?.id ?? null;
    cachedPath=undefined;clearPreview();
  } else if(key==='metric') {
    analysis.metric=input.value;
  } else if(key==='pistonId'||key==='memberId') {
    const part=lookup(design,{type:'element',id:input.value});
    if(part){analysis[key]=part.id;setSelection([{type:'element',id:part.id}]);}
  }
  if(['minAngle','maxAngle','driverId','pistonId'].includes(key))save();
  forceReport=null;render();
}
function action(act,b) {
  if(['undo','redo'].includes(act)) return history(act==='undo');
  if(act==='new'||act==='example') return replaceProject(act==='new'?{model:createMechanism(),name:'Untitled mechanism'}:{model:createDemo(),name:demoTitle,study:hatchStudy});
  if(act==='open') return $('#project-file').click();
  if(act==='export') return download(JSON.stringify(envelope(),null,2),'application/json','.json');
  if(act==='export-path'){
    if(!travel().poses.length){hint('Define a motion path before exporting a sweep.');return;}
    download(pathCsv(travel(),design),'text/csv;charset=utf-8','-sweep.csv');return;
  }
  if(act==='save'){save();checkpointDesign();hint('Design checkpoint saved.');return;}
  if(act==='restore-saved'){replaceProject({...checkpoint,model:cloneMechanism(checkpoint.model)});hint('Restored the saved design. Undo returns to the previous edit.');return;}
  if(act==='fit')return drawing.fit();
  if(act==='zoom-in'||act==='zoom-out')return drawing.zoom(act==='zoom-in'?1.25:.8);
  if(act==='results'||act==='force-range')return switchMode('analysis');
  if(act==='excursion'){visibility.excursion=!visibility.excursion;render();return;}
  if(act==='preview-analysis-pose'){samplePose(Number(b.dataset.index));render();return;}
  if(act==='clear-analysis-pose'){clearPreview();render();return;}
  if(act==='pick-piston'){const p=lookup(current(),{type:'element',id:b.dataset.id});if(p)select({type:'element',id:p.id});return;}
  if(act==='close-panel'){page=null;return;}
  if(act==='inspect-piston-alert'&&sim){
    const issue=sim.validity.issues[0];
    if(!sim.validity.valid){
      switchMode('model');
      if(issue?.element!=null) select({type:'element',id:issue.element});
      hint(issue?.message || 'Repair the installation in Model.');
    }else{page='motion';render();}
    return;
  }
  if((act==='play'||act==='step')&&sim&&!sim.validity.valid){hint(defaultHint());return;}
  if(act==='reverse-travel'&&sim && physicsMode==='forced' && sim.validity.valid){
    sim.reverseForcedTravel();measured=false;forceReport=null;render();hint(sim.forcedReversed?'Forced travel reversed. The installed piston rating is unchanged.':'Following the configured piston direction.');return;
  }
  if(act==='play'&&sim){if(playing)stop();else{playing=true;frame=requestAnimationFrame(tick);}render();hint(defaultHint());return;}
  if(act==='step'&&sim){stop();const previous=sim.time;sim.step(1/60);measured=measured || sim.time>previous;forceReport=null;render();hint(sim.motionMessage || 'Advanced one simulation frame (1/60 s).');return;}
  if(act==='reset'&&sim){stop();sim.reset();measured=false;forceReport=null;render();hint('Restored the design pose.');return;}
  if(act==='trace'&&sim){sim.setTracing(!sim.tracing);render();return;}
  if(act==='clear-traces'&&sim){sim.clearTraces();renderDrawing();return;}
  if(sim)return;
  if(['use-required','use-peak','use-path-lengths'].includes(act))return useForce(act,b);
  if(mode!=='model')return;
  if(act==='invert-barrel'){
    const part=pick?.type==='element' && lookup(design,pick);
    if(part && isPiston(part)) update(()=>{part.barrelEnd=part.barrelEnd==='b'?'a':'b';},'Cylinder mounting reversed.');
    return;
  }
  if(act==='delete'&&selections.some(s=>lookup(design,s)?.kind!=='origin')){
    update(()=>{
      for(const s of selections)removeObject(design,s.type,s.id);
      setSelection([]);page=null;
    },'Selection deleted. Undo restores the edit.');
  }
}
document.addEventListener('click',event=>{
  const b=event.target.closest('button'); if(!b)return;
  if(b.dataset.pick){
    const type=b.dataset.pick, id=lookup(current(),{type,id:b.dataset.id})?.id;
    if(id!=null){
      const s={type,id};
      if(tool==='constraint')constraintOperand(s);
      else if(event.shiftKey)selectMany([s],{additive:true,toggle:true});
      else if(mode==='model'&&(event.ctrlKey||event.metaKey)&&pick)contextFor(pick,s,{x:event.clientX,y:event.clientY});
      else select(s);
    }
    return;
  }
  if(b.dataset.mode)return switchMode(b.dataset.mode);
  if(b.dataset.tool)return selectTool(b.dataset.tool);
  if(b.dataset.constraint)return startConstraint(b.dataset.constraint);
  if(b.dataset.panel==='results')return switchMode('analysis');
  if(b.dataset.panel && b.dataset.panel!=='tree'){page=b.dataset.panel==='structure'?'properties':b.dataset.panel;render();return;}
  if(b.dataset.action)action(b.dataset.action,b);
});
document.addEventListener('pointerdown',e=>{if(menu&&!menu.contains(e.target))closeContext();});
document.addEventListener('input',event=>{
  if(event.target.dataset.analysis==='sampleIndex')analysisChange(event.target,true);
});
document.addEventListener('focusin',event=>{
  const field=event.target;
  if(field.matches('input[data-field]:not([type="checkbox"])')) field.dataset.enteredValue=field.value;
});
document.addEventListener('focusout',event=>{
  const input=event.target,key=input.dataset?.analysis;
  if(input.isConnected && input.dataset.enteredValue!==undefined && input.value!==input.dataset.enteredValue){
    delete input.dataset.enteredValue;
    fieldChange(input);
    return;
  }
  if(input.isConnected && ['minAngle','maxAngle'].includes(key) && input.valueAsNumber!==analysis[key]) analysisChange(input);
});
document.addEventListener('change',e=>{
  const el=e.target;
  if(el.name==='physics-mode' && el.checked && sim){
    stop();physicsMode=el.value==='forced'?'forced':'real';sim.physicsMode=physicsMode;
    measured=false;forceReport=null;save();render();hint(defaultHint());return;
  }
  if(el.id==='playback-speed'){
    speed=Number(el.value);
    try { localStorage.setItem(storeKey+'.speed',String(speed)); } catch { /* Session speed still applies. */ }
    updateStatus();return;
  }
  if(el.dataset.analysis)return analysisChange(el);
  if(el.dataset.forceToggle) {
    const type=el.dataset.forceToggle, id=el.dataset.id;
    if(mode==='model') {
      update(()=>{const item=lookup(design,{type,id});if(item)item.showForce=el.checked;},'Force visibility updated.');
    } else {
      forceOverrides.set(type+':'+id,el.checked);renderDrawing();
    }
    return;
  }
  if(el.dataset.field) {
    delete el.dataset.enteredValue;
    return fieldChange(el);
  }
  if(el.dataset.visibility){
    const key=el.dataset.visibility;
    if(key==='loads')forcePolicy[mode]=el.checked;else visibility[key]=el.checked;
    forceReport=null;renderDrawing();
  }
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
  if(ctrl && key==='a' && mode==='model'){
    e.preventDefault();
    selectMany(Object.entries(collections).flatMap(([type,key])=>design[key].filter(item=>item.kind!=='origin').map(item=>({type,id:item.id}))));
    return;
  }
  if(ctrl && ['s','o','z','y','0'].includes(key)){e.preventDefault();action(({s:'save',o:'open',z:e.shiftKey?'redo':'undo',y:'redo','0':'fit'})[key]);return;}
  if(ctrl)return;
  if(key==='f5'){e.preventDefault();switchMode('simulation');}
  else if(key==='escape'){cancelTool();}
  else if(key==='delete')action('delete');
  else if(key==='d')selectTool('dimension');
  else if(key==='v')selectTool('select');
  else if(key==='h')selectTool('pan');
  else if(key===' '&&sim&&!e.target.closest('button,svg')){e.preventDefault();action('play');}
});
document.addEventListener('visibilitychange',()=>{if(document.hidden&&playing){stop();render();hint('Simulation paused.');}});
render();hint(startupHint || defaultHint());
