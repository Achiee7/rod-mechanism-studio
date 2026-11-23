import { balanceForce, assistForce, distance, isPiston, sketchDriven } from './mechanics.js';

export const collections = {joint:'joints', element:'elements', load:'loads', dimension:'dimensions', constraint:'constraints'};
export const kindName = kind => ({
  rotation:'Hinge', fixed:'Anchor', pin:'Pin', origin:'Sketch origin', body:'Weighted body', rod:'Rod',
  actuator:'Actuator', gas_spring:'Gas spring', motor:'Motor', point:'Point load', equal_length:'Equal length',
  horizontal:'Horizontal', vertical:'Vertical', coincident:'Coincident', parallel:'Parallel', distance:'Distance',
}[kind] || kind);
export function label(item) {
  return item.name || (kindName(item.kind) + ' ' + item.id);
}
export function lookup(model, pick) {
  return pick && model[collections[pick.type]]?.find(i => String(i.id) === String(pick.id));
}
function h(tag, text, className) {
  const e = document.createElement(tag);
  if (text != null) e.textContent = text;
  if (className) e.className = className;
  return e;
}
const fmt = (n, digits=1) => Number.isFinite(n) ? n.toLocaleString('nl-NL', {maximumFractionDigits:digits}) : '—';
const metric = (n,u,digits=1) => fmt(n,digits) + ' ' + u;
function row(title, value) {
  const line = h('div',null,'rm-readout');
  line.append(h('span',title),h('strong',value));
  return line;
}
function section(title,...items) {
  const el = h('section',null,'rm-section');
  el.append(h('h3',title),...items.filter(Boolean));
  return el;
}
function button(title, act, style='') {
  const b = h('button',title,'rm-button '+style); b.type='button'; b.dataset.action=act; return b;
}
function field(title, key, value, config={}) {
  const wrap=h('label',null,'rm-field');
  wrap.append(h('span',title));
  let input;
  if (config.choices) {
    input=h('select');
    for (const [id,text] of config.choices) {const opt=h('option',text); opt.value=id; input.add(opt);}
    input.value=value;
  } else {
    input=h('input'); input.type=config.type || 'number';
    if (input.type==='checkbox') input.checked=!!value;
    else if(input.type==='number') {
      input.step=config.step || 'any';
      input.value=Number.isFinite(value) ? Number(value.toFixed(6)) : '';
      if(config.min!=null) input.min=config.min;
      if(config.max!=null) input.max=config.max;
    } else input.value=value || '';
  }
  input.dataset.field=key;
  input.disabled=!!config.disabled;
  wrap.append(input);
  return wrap;
}
const treeGroups = [
  ['Components','element'],['Joints','joint'],['Forces','load'],['Dimensions','dimension'],['Constraints','constraint']
];
export function renderTree(host, model, pick) {
  const old = new Map([...host.querySelectorAll('details')].map(d=>[d.dataset.type,d.open]));
  const fragment=document.createDocumentFragment();
  for(const [heading,type] of treeGroups) {
    const list=model[collections[type]];
    const group=h('details',null,'rm-tree-group'); group.dataset.type=type;
    group.open=old.get(type) ?? ['joint','element','load'].includes(type);
    const summary=h('summary'); summary.append(h('span',heading),h('small',String(list.length))); group.append(summary);
    for(const item of list) {
      const b=h('button',null,'rm-tree-item'); b.type='button'; b.dataset.pick=type; b.dataset.id=item.id;
      b.setAttribute('aria-pressed', String(pick?.type===type && pick.id===item.id));
      const icon=h('span',null,'rm-symbol '+item.kind); icon.setAttribute('aria-hidden','true');
      b.title=label(item); b.append(icon,h('span',label(item))); group.append(b);
    }
    if(!list.length) group.append(h('span','None','rm-empty'));
    fragment.append(group);
  }
  host.replaceChildren(fragment);
}
function sizeSection(model, part, editable) {
  const b=balanceForce(model,part.id), a=assistForce(model,part.id);
  if(!b) return section('Piston sizing',h('p','A moving lever and an off-axis piston attachment about a grounded pivot are needed to calculate hold force.','rm-note'));
  const sign=b.force<0?'Pull':'Push';
  const direction=(part.forceMode==='pull'?-1:1)*part.force;
  const alongRequired=direction*Math.sign(b.force || 1);
  return section('Piston sizing',
    row('Required hold',sign+' · '+metric(Math.abs(b.force),'N')),
    row('External moment',metric(b.moment,'N·m',2)),
    row('Effective arm',metric(Math.abs(b.momentArm)*1000,'mm')),
    h('p',fmt(Math.abs(b.moment),2)+' N·m ÷ '+fmt(Math.abs(b.momentArm),4)+' m = '+metric(Math.abs(b.force),'N'),'rm-equation'),
    row('Force margin',metric(alongRequired-Math.abs(b.force),'N')),
    a && row('Vertical hand effort',metric(Math.abs(a.handForce),'N')+(a.handForce<0?' down':' up')),
    a && row('Applied at',label(lookup(model,{type:'joint',id:a.handleId}))),
    editable && button('Use required force','use-required'));
}
function ranges(model,path,editable) {
  const out=[h('p','Approximate pivot sweep over ±80° about the design pose. Raw travel measures the connection path. Piston travel applies the stroke stops.','rm-note')];
  const pistons=model.elements.filter(isPiston);
  if(!pistons.length) out.push(h('p','Add an actuator or gas spring to compare hold force and stroke.','rm-note'));
  for(const p of pistons) {
    const forces=path?.forceRanges?.[p.id], raw=path?.requiredLengthRanges?.[p.id], limited=path?.lengthRanges?.[p.id];
    const b=button(label(p),'pick-piston','rm-link'); b.dataset.id=p.id;
    out.push(b);
    if(!forces&&!raw) {out.push(h('p','No supported pivot sweep for this connection.','rm-note')); continue;}
    if(forces) out.push(row('Axial force range',fmt(forces[0])+' … '+metric(forces[1],'N')));
    else out.push(h('p','No usable force samples within stroke and moment-arm limits.','rm-note'));
    const assist=path?.assistForceRanges?.[p.id];
    if(assist) out.push(row('Hand effort (up +)',fmt(assist[0])+' … '+metric(assist[1],'N')));
    if(raw) out.push(row('Raw length range',fmt(raw[0]*1000)+' … '+metric(raw[1]*1000,'mm')),row('Required stroke',metric((raw[1]-raw[0])*1000,'mm')));
    if(limited) out.push(row('Stroke-limited length',fmt(limited[0]*1000)+' … '+metric(limited[1]*1000,'mm')));
    if(forces && editable){const apply=button('Use peak path force','use-peak'); apply.dataset.id=p.id;out.push(apply);}
  }
  for(const part of model.elements.filter(part=>!isPiston(part))) {
    const range=path?.forceRanges?.[part.id];
    if(range) out.push(row(label(part)+' · axial',fmt(range[0])+' … '+metric(range[1],'N')));
  }
  return section('Travel and peak force',...out);
}
export function inspectorContent({model,pick,page,simulation,path,time=0,playing=false,trace=false}) {
  const item=lookup(model,pick), edit=!simulation;
  const title=item && page==='properties' ? label(item) : page==='motion'?'Motion settings':page==='results'?'Force results':'Properties';
  const blocks=[];
  const jointLabel=id=>{const j=lookup(model,{type:'joint',id});return j?label(j):'—';};
  if(page==='results') {
    const current=pick?.type==='element' && item && isPiston(item) ? item : model.elements.find(isPiston);
    if(current) blocks.push(section('Current pose',row('Component',label(current))),sizeSection(model,current,false));
    blocks.push(ranges(model,path,edit));
  } else if(page==='motion') {
    const o=model.options;
    blocks.push(section('Simulation settings',
      field('Gravity','gravityEnabled',o.gravityEnabled,{type:'checkbox',disabled:!edit}),
      field('Gravity (m/s²)','gravity',o.gravity,{min:0,disabled:!edit}),
      field('Damping','damping',o.damping,{min:0,max:1,disabled:!edit}),
      field('Solver iterations','iterations',o.iterations,{min:1,max:200,step:1,disabled:!edit})));
    if(simulation) blocks.push(section('Playback',row('Elapsed',metric(time,'s',2)),button(playing?'Pause':'Run','play'),button('Step one frame','step'),button('Reset pose','reset'),button(trace?'Hide traces':'Trace moving pins','trace'),button('Clear traces','clear-traces')));
  } else if(item) {
    const f=(title,key,value,config={})=>field(title,key,value,{disabled:!edit,...config});
    const type=pick.type;
    if(item.kind==='origin') blocks.push(section('Construction reference',row('Position','0, 0 mm'),h('p','Use this point to dimension the sketch. Place a hinge or anchor to ground the physical mechanism.','rm-note')));
    else {
      const props=[f('Name','name',item.name,{type:'text'})];
      if(type==='joint') props.push(
        f('Joint type','kind',item.kind,{choices:['pin','fixed','rotation'].map(k=>[k,kindName(k)])}),
        f('X (mm)','x',item.x*1000,{disabled:!edit || sketchDriven(model,item.id)}),f('Y (mm)','y',item.y*1000,{disabled:!edit || sketchDriven(model,item.id)}),
        f('Mass (kg)','mass',item.mass,{min:0}));
      if(type==='element') {
        props.push(row('From',jointLabel(item.a)),row('To',jointLabel(item.b)));
        const length=distance(model,item.a,item.b);
        if(isPiston(item)) props.push(row('Connection length',metric(length*1000,'mm')),
          f('Compressed (mm)','minLength',item.minLength*1000,{min:1}),f('Extended (mm)','maxLength',item.maxLength*1000,{min:1}),
          f('Rated force (N)','force',item.force,{min:0}),f('Direction','forceMode',item.forceMode,{choices:[['push','Push / extend'],['pull','Pull / retract']]}),
          f('Stiffness (N/m)','stiffness',item.stiffness,{min:0}),f('Damping (N·s/m)','damping',item.damping,{min:0}),
          f('Auto return','autoReturn',item.autoReturn,{type:'checkbox'}));
        else props.push(f('Length (mm)','length',length*1000,{min:1}));
        props.push(f('Mass (kg)','mass',item.mass,{min:0}));
        if(item.kind==='motor') props.push(f('Torque (N·m)','torque',item.torque));
      }
      if(type==='load') props.push(row('At joint',jointLabel(item.joint)),f('X force (N)','fx',item.fx),f('Y force (N)','fy',item.fy),f('Enabled','enabled',item.enabled,{type:'checkbox'}));
      if(type==='dimension') {
        const a=lookup(model,{type:'joint',id:item.a}), b=lookup(model,{type:'joint',id:item.b});
        const measured=item.kind==='horizontal'?Math.abs(a.x-b.x):item.kind==='vertical'?Math.abs(a.y-b.y):distance(model,a.id,b.id);
        const value=simulation||!item.driving?measured:item.value;
        props.push(row('Dimension',kindName(item.kind)),row('Between',jointLabel(item.a)+' / '+jointLabel(item.b)),f('Value (mm)','value',value*1000,{min:item.kind==='distance'?1:0,disabled:!edit||!item.driving}),f('Offset (mm)','offset',item.offset*1000),f('Driving','driving',item.driving,{type:'checkbox'}));
      }
      if(type==='constraint') props.push(row('Relation',kindName(item.kind)),row('Between',jointLabel(item.a)+' / '+jointLabel(item.b)));
      blocks.push(section(kindName(item.kind),...props));
      if(type==='element' && isPiston(item)) blocks.push(sizeSection(model,item,edit),button('Inspect travel and peak force','results'));
      if(type==='joint'&&simulation) blocks.push(section('Forces at joint',row('Applied X',metric(item.fx,'N')),row('Applied Y',metric(item.fy,'N')),row('Reaction X',metric(item.reactionX,'N')),row('Reaction Y',metric(item.reactionY,'N'))));
      if(type==='element' && simulation) blocks.push(section('Simulation forces',row('Axial force',metric(item.axialForce,'N')),isPiston(item)&&row('Stroke-stop reaction',metric(item.limitForce,'N'))));
      if(edit) blocks.push(button('Delete '+({element:'component',joint:'joint',load:'load',dimension:'dimension',constraint:'constraint'}[type]),'delete','rm-delete'));
    }
  } else blocks.push(h('p','Select a component, joint, dimension or load in the drawing or model tree.','rm-note'));
  return {title,blocks};
}
