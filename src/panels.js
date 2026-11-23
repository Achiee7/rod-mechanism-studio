import { balanceForce, assistForce, distance, isPiston, sketchDriven } from './mechanics.js';
import { pathChart } from './path-chart.js';
import { pistonGeometry } from './canvas.js';

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
const millimeters = value => metric(value * 1000,'mm');
const signed = value => Number.isFinite(value) ? (value > 0 ? '+' : '') + metric(value,'N') : 'Unavailable';
const direction = value => value > 0 ? 'Push / extend' : value < 0 ? 'Pull / retract' : 'Zero';
const finitePair = bounds => Array.isArray(bounds) && bounds.length === 2 && bounds.every(Number.isFinite);
const jointName = (model,id) => model.joints.find(joint => String(joint.id) === String(id))?.name || 'Joint ' + id;
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
export function renderTree(host, model, pick, selections = []) {
  const old = new Map([...host.querySelectorAll('details')].map(d=>[d.dataset.type,d.open]));
  const fragment=document.createDocumentFragment();
  for(const [heading,type] of treeGroups) {
    const list=model[collections[type]];
    const group=h('details',null,'rm-tree-group'); group.dataset.type=type;
    group.open=old.get(type) ?? ['joint','element','load'].includes(type);
    const summary=h('summary'); summary.append(h('span',heading),h('small',String(list.length))); group.append(summary);
    for(const item of list) {
      const b=h('button',null,'rm-tree-item'); b.type='button'; b.dataset.pick=type; b.dataset.id=item.id;
      b.setAttribute('aria-pressed', String([pick,...selections].some(selected => selected?.type === type && String(selected.id) === String(item.id))));
      const icon=h('span',null,'rm-symbol '+item.kind); icon.setAttribute('aria-hidden','true');
      b.title=label(item); b.append(icon,h('span',label(item))); group.append(b);
    }
    if(!list.length) group.append(h('span','None','rm-empty'));
    fragment.append(group);
  }
  host.replaceChildren(fragment);
}
function handEffort(value) {
  return Number.isFinite(value) ? metric(Math.abs(value),'N') + (value < 0 ? ' down' : value > 0 ? ' up' : '') : 'Unavailable';
}

function pistonState(status) {
  return ({
    singular: 'The piston has too little leverage at this pose to determine a reliable hold force.',
    out_of_stroke: 'The connection length is outside the configured stroke.',
    unavailable: 'This pose does not give a unique piston force with the current supports and drive.',
  })[status] || 'Hold force is unavailable at this pose. Check the supports and available drive.';
}

function strokeInfo(part, length) {
  const epsilon = 1e-6;
  if (length < part.minLength - epsilon) return millimeters(part.minLength - length) + ' below compressed limit';
  if (length > part.maxLength + epsilon) return millimeters(length - part.maxLength) + ' above extended limit';
  if (length <= part.minLength + epsilon) return 'At compressed stop';
  if (length >= part.maxLength - epsilon) return 'At extended stop';
  return 'Within stroke';
}

function sizeSection(model, part, {editable = false, sample, pathEstimate = false, reachable = true} = {}) {
  const length = sample?.length ?? distance(model,part.a,part.b);
  const fit = strokeInfo(part,length);
  const inStroke = length >= part.minLength-1e-6 && length <= part.maxLength+1e-6;
  const barrelCheck = pistonGeometry(part,Math.min(part.maxLength,Math.max(part.minLength,length)));
  const balance = !pathEstimate && inStroke ? balanceForce(model,part.id) : null;
  const assist = balance ? assistForce(model,part.id) : null;
  const required = pathEstimate ? sample?.force : balance?.force;
  const rated = (part.forceMode === 'pull' ? -1 : 1) * part.force;
  const configured = sample?.configuredForce ?? assist?.configuredForce ?? rated;
  const effort = reachable ? (sample?.assistForce ?? assist?.handForce) : null;
  const oppose = required && configured && Math.sign(required) !== Math.sign(configured);
  const margin = reachable && inStroke && Number.isFinite(required) && Number.isFinite(configured)
    ? oppose ? 'Opposing force direction' : signed(Math.abs(configured) - Math.abs(required))
    : 'Unavailable';
  return section(pathEstimate ? 'Sample piston sizing' : 'Piston sizing at the current pose',
    row('Connection length',millimeters(length)),
    row('Stroke-limited length',millimeters(Math.min(part.maxLength,Math.max(part.minLength,length)))),
    row('Piston range',millimeters(part.minLength) + ' – ' + millimeters(part.maxLength)),
    row('Stroke capacity',millimeters(part.maxLength-part.minLength)),
    row('Stroke check',fit),
    barrelCheck.problem && h('p','Single-stage barrel: ' + barrelCheck.problem + '.','rm-path-message'),
    row('Configured rating (+ push)',signed(rated)),
    reachable && configured !== rated && row('Effective configured force',signed(configured)),
    Number.isFinite(required) ? row(reachable ? 'Required hold (+ push)' : 'Geometry-only hold demand (+ push)',signed(required)) : h('p',
      !inStroke ? pistonState('out_of_stroke') : pathEstimate ? pistonState(sample?.status)
        : 'A grounded pivot, a moving lever and an off-axis attachment are needed to determine hold force at this pose.',
      'rm-note'),
    Number.isFinite(required) && row('Required direction',direction(required)),
    row('Force headroom',margin),
    row('Vertical hand effort',handEffort(effort)),
    (sample?.handleId ?? assist?.handleId) != null && Number.isFinite(effort)
      && row('Hand effort at',jointName(model,sample?.handleId ?? assist.handleId)),
    balance && row('External moment',metric(balance.moment,'N·m',2)),
    balance && row('Effective arm',millimeters(Math.abs(balance.momentArm))),
    balance && h('p',fmt(Math.abs(balance.moment),2)+' N·m ÷ '+fmt(Math.abs(balance.momentArm),4)+' m = '+metric(Math.abs(required),'N'),'rm-equation'),
    Number.isFinite(effort) && h('p','Hand effort is the vertical force required to hold this pose with the configured piston.','rm-note'),
    editable && Number.isFinite(required) && button('Use required force','use-required'));
}

function forceToggle(type, item, analysis) {
  const wrap = h('label',null,'rm-force-toggle');
  const input = h('input');
  input.type = 'checkbox';
  input.checked = analysis.forceDisplay?.(type,item.id) ?? item.showForce !== false;
  input.dataset.forceToggle = type;
  input.dataset.id = item.id;
  wrap.append(input,h('span','Show force on drawing'));
  return wrap;
}

function vectorText(vector) {
  const {x,y} = vector ?? {};
  return Number.isFinite(x) && Number.isFinite(y)
    ? 'X ' + signed(x) + ' · Y ' + signed(y) : 'Unavailable';
}

function jointForceRows(joint, forces, status) {
  const resolved = status === 'balanced' || status === 'dynamic';
  return [
    row('Applied',vectorText(forces?.applied)),
    row('From connected elements',vectorText(resolved ? forces?.member : null)),
    ['fixed','rotation'].includes(joint.kind) && row('Support reaction',vectorText(forces?.support)),
    row('Resultant',vectorText(resolved ? forces?.net : null)),
  ];
}

function elementForceRows(item, forces, status) {
  if (item.kind === 'motor') return [
    row('Motor torque (counterclockwise +)',metric(item.torque,'N·m')),
    row('Drive tangential force',signed(forces?.axialForce)),
    row('Constraint axial (+ compression)',signed(forces?.memberAxialForce)),
  ];
  return [
    row(isPiston(item) ? 'Piston axial (+ push)' : 'Axial (+ compression)',signed(forces?.axialForce)),
    isPiston(item) && row('Stroke-stop force',signed(forces?.limitForce ?? (status === 'balanced' ? 0 : null))),
  ];
}

function forceItem(title, rows, toggle, open = false) {
  const item = h('details',null,'rm-force-item');
  item.open = open;
  item.append(h('summary',title),...rows.filter(Boolean),toggle);
  return item;
}

function forceBreakdown(model, forces, analysis, pick) {
  const current = h('div',null,'rm-force-groups');
  const elements = h('details',null,'rm-force-category');
  const joints = h('details',null,'rm-force-category');
  const physicalJoints = model.joints.filter(joint => joint.kind !== 'origin');
  elements.open = pick?.type === 'element';
  joints.open = pick?.type === 'joint';
  elements.append(h('summary','Element forces · ' + model.elements.length));
  joints.append(h('summary','Joint forces · ' + physicalJoints.length));
  for (const item of model.elements) {
    elements.append(forceItem(label(item),elementForceRows(item,forces?.elements?.[item.id],forces?.status),forceToggle('element',item,analysis),elements.open && item.id === pick.id));
  }
  for (const joint of physicalJoints) {
    joints.append(forceItem(label(joint),jointForceRows(joint,forces?.joints?.[joint.id],forces?.status),forceToggle('joint',joint,analysis),joints.open && joint.id === pick.id));
  }
  current.append(elements,joints);
  if (model.loads.length) {
    const loads = h('details',null,'rm-force-category');
    loads.open = pick?.type === 'load';
    loads.append(h('summary','Point loads · ' + model.loads.length));
    for (const item of model.loads) {
      loads.append(forceItem(label(item),[row('Point load',item.enabled ? vectorText({x:item.fx,y:item.fy}) : 'Disabled')],forceToggle('load',item,analysis),loads.open && item.id === pick.id));
    }
    current.append(loads);
  }
  const source = forces?.strokeViolation ? 'This preview is outside a piston stroke. Applied inputs are shown without support and member estimates.'
    : forces?.indeterminate ? 'Static equilibrium is possible. Redundant members share load through a minimum-norm estimate, so individual forces are not unique.'
    : ({
    dynamic: 'Simulation forces from the most recent time step.',
    balanced: 'Quasi-static estimate at this pose using gravity, applied loads, and configured drive forces.',
    unbalanced: 'Static equilibrium could not be resolved at this pose. Available input force vectors are shown.',
  })[forces?.status] || 'No force estimate is available at this pose.';
  return section('Forces at the displayed pose',h('p',source,'rm-note'),current);
}

const quantities = [['force','Piston hold force'],['length','Piston connection length'],['hand','Vertical hand effort'],['member','Member axial force']];

function analysisChoice(title, key, value, choices) {
  const wrap = field(title,key,value,{choices});
  const input = wrap.lastChild;
  delete input.dataset.field;
  input.dataset.analysis = key;
  return wrap;
}

function analysisAngle(title, key, value) {
  const wrap = field(title,key,value,{min:-360,max:360,step:1});
  const input = wrap.lastChild;
  delete input.dataset.field;
  input.dataset.analysis = key;
  return wrap;
}

function chartState(model, path, analysis) {
  const pistons = model.elements.filter(isPiston);
  const members = model.elements.filter(part => ['rod','body'].includes(part.kind));
  const target = (items, id) => items.find(item => String(item.id) === String(id)) ?? items[0];
  const piston = target(pistons,analysis.pistonId);
  const member = target(members,analysis.memberId);
  const choices = quantities.filter(([key]) => key === 'member' ? members.length : pistons.length);
  const metric = choices.find(([key]) => key === analysis.metric)?.[0] ?? choices[0]?.[0] ?? 'force';
  const poses = path?.poses ?? [];
  const defaultIndex = poses.reduce((best,pose,index) => Math.abs(pose.angleDeg) < Math.abs(poses[best].angleDeg) ? index : best,0);
  const requested = analysis.sampleIndex == null ? defaultIndex : Number(analysis.sampleIndex);
  const sampleIndex = Number.isInteger(requested) && poses[requested] ? requested : defaultIndex;
  return {poses,piston,member,pistons,members,choices,metric,sampleIndex,pistonId:piston?.id,memberId:member?.id};
}

function sampleText(pose) {
  if (!pose) return 'No sample';
  return fmt(pose.angleDeg,2) + '° · ' + (pose.feasible ? 'Reachable' : pose.positions ? 'Geometry only' : 'Unsolved');
}

function renderChart(host, state, sampleInput) {
  host.replaceChildren(pathChart(state.poses,state,index => {
    sampleInput.value = index;
    sampleInput.dispatchEvent(new Event('input',{bubbles:true}));
    sampleInput.dispatchEvent(new Event('change',{bubbles:true}));
  }));
}

export function updateAnalysisCursor(host, {model, design = model, path, analysis = {}}) {
  const chartHost = host.querySelector('[data-analysis-chart]');
  const sampleInput = host.querySelector('[data-analysis="sampleIndex"]');
  if (!chartHost || !sampleInput) return;
  const state = chartState(design,path,analysis);
  const sample = state.poses[state.sampleIndex];
  const text = sampleText(sample);
  host.querySelector('[data-analysis-sample-caption]').textContent = text;
  sampleInput.setAttribute('aria-valuetext',text);
  const preview = host.querySelector('[data-action="preview-analysis-pose"]');
  preview.dataset.index = state.sampleIndex;
  preview.textContent = sample?.feasible ? 'Preview sample' : 'Preview geometry';
  preview.disabled = !sample?.positions;
  renderChart(chartHost,state,sampleInput);
}

function pathControls(model,path,analysis) {
  const drivers = path?.drivers || [];
  const state = chartState(model,path,analysis);
  const items = [h('p','Sweep a grounded member about its pivot. Angle is measured from the saved design pose, positive counterclockwise.','rm-note')];
  if (drivers.length) {
    const choices = drivers.map(driver => [driver.id,driver.label || 'About ' + jointName(model,driver.pivotId)]);
    if (path.driverId == null) choices.unshift(['','Choose a drive member']);
    items.push(analysisChoice('Drive member','driverId',path.driverId ?? '',choices));
    const bounds = h('div',null,'rm-analysis-bounds');
    bounds.append(analysisAngle('From (°)','minAngle',analysis.minAngle ?? -80),analysisAngle('To (°)','maxAngle',analysis.maxAngle ?? 80));
    items.push(bounds);
  }
  if (path?.message) items.push(h('p',path.message,'rm-path-message'));
  if (path?.message?.includes('free motions')) items.push(h('p','Use weighted-body segments to model a rigid bracket, or add a physical link. Sketch dimensions and constraints define the drawing pose.','rm-note'));
  if (!state.poses.length) return section('Motion path',...items,
    !path?.message && h('p','A grounded joint connected to a movable rod or body is needed to drive a path.','rm-note'));

  items.push(analysisChoice('Plot','metric',state.metric,state.choices));
  if (state.metric === 'member' && state.members.length) {
    items.push(analysisChoice('Member','memberId',state.memberId,state.members.map(item => [item.id,label(item)])));
  }
  if (state.metric !== 'member' && state.pistons.length) {
    items.push(analysisChoice('Piston','pistonId',state.pistonId,state.pistons.map(item => [item.id,label(item)])));
  }
  const chart = h('div',null,'rm-chart-host');
  chart.dataset.analysisChart = '';
  const slider = h('label',null,'rm-sample-slider');
  const labelText = h('span','Pose sample');
  const caption = h('strong',sampleText(state.poses[state.sampleIndex]));
  caption.dataset.analysisSampleCaption = '';
  const input = h('input');
  input.type = 'range';
  input.min = 0;
  input.max = state.poses.length - 1;
  input.step = 1;
  input.value = state.sampleIndex;
  input.dataset.analysis = 'sampleIndex';
  input.setAttribute('aria-label','Pose sample');
  input.setAttribute('aria-valuetext',caption.textContent);
  slider.append(labelText,caption,input);
  renderChart(chart,state,input);

  const buttons = h('div',null,'rm-sample-actions');
  const sample = state.poses[state.sampleIndex];
  const preview = button(sample.feasible ? 'Preview sample' : 'Preview geometry','preview-analysis-pose');
  preview.dataset.index = state.sampleIndex;
  preview.disabled = !sample.positions;
  const clear = button('Return to design pose','clear-analysis-pose','rm-link');
  clear.disabled = !analysis.preview;
  buttons.append(preview,clear);
  const reachable = state.poses.filter(pose => pose.feasible).length;
  items.push(chart,slider,buttons);
  items.push(row('Reachable pose samples',reachable + ' / ' + state.poses.length));
  if (!sample.feasible) items.push(h('p',sample.positions
    ? 'Geometry-only sample. The requested angle cannot be reached from the design pose with the installed linkage and piston strokes.'
    : 'The linkage geometry could not be solved at this angle.',
    'rm-note'));
  if (analysis.preview && sample.feasible) items.push(h('p','The canvas is previewing a sampled pose. The saved mechanism stays in its design pose.','rm-note'));
  return section('Motion path',...items);
}

function pathSizing(path, part, editable) {
  if (!part) return section('Piston sizing',h('p','Add an actuator or gas spring to size piston force and stroke.','rm-note'));
  const id = part.id;
  const recommendation = path?.recommendations?.[id];
  const forces = path?.requiredForceRanges?.[id];
  const installed = path?.forceRanges?.[id];
  const raw = path?.requiredLengthRanges?.[id];
  const permitted = path?.lengthRanges?.[id];
  const hand = path?.assistForceRanges?.[id];
  const mixed = recommendation?.mode === 'both' || finitePair(forces) && forces[0] < -1e-6 && forces[1] > 1e-6;
  const usableForce = Number.isFinite(recommendation?.signedForce) && !mixed && recommendation.status === 'ok';
  const lower = recommendation?.minLength;
  const upper = recommendation?.maxLength;
  const lengthRecommendation = Number.isFinite(lower) && Number.isFinite(upper) && upper >= lower && lower > 0;
  const candidate = lengthRecommendation
    ? pistonGeometry({...part,minLength:lower,maxLength:upper},(lower+upper)/2)
    : null;
  const completeGeometry = path?.poses?.length && path.poses.every(pose => Number.isFinite(pose.pistons?.[id]?.length));
  const peak = recommendation?.signedForce;
  const peakAt = recommendation?.peakAngleDeg;
  const formatRange = (bounds, formatValue) => formatValue(bounds[0]) + ' … ' + formatValue(bounds[1]);
  const out = [row('Piston',label(part))];
  if (finitePair(forces)) {
    out.push(row('Hold demand · full solved sweep',formatRange(forces,signed)));
    if (Number.isFinite(peak)) {
      out.push(row('Peak sampled hold',signed(peak) + (Number.isFinite(peakAt) ? ' at ' + fmt(peakAt,2) + '°' : '')),
        row('Required direction',mixed ? 'Push and pull' : direction(peak)));
    }
  } else {
    out.push(h('p','No unique hold force was available from the solved geometries. Check the drive and supports.','rm-note'));
  }
  if (finitePair(installed)) out.push(row('Hold demand · reachable poses',formatRange(installed,signed)));
  if (finitePair(hand)) out.push(row('Vertical hand effort',formatRange(hand,handEffort)));
  if (finitePair(raw)) out.push(row('Raw connection lengths',formatRange(raw,millimeters)));
  if (finitePair(permitted)) out.push(row('Stroke-limited lengths',formatRange(permitted,millimeters)));
  if (lengthRecommendation && completeGeometry && !candidate.problem) {
    out.push(row('Suggested compressed length',millimeters(lower)),row('Suggested extended length',millimeters(upper)),
      row('Travel required',millimeters(recommendation.stroke ?? upper-lower)));
  } else if (finitePair(raw)) {
    out.push(row('Raw travel required',millimeters(raw[1]-raw[0])));
  }
  if (completeGeometry && candidate?.problem) {
    out.push(row('Modeled barrel capacity',millimeters(candidate.barrelLength)),
      h('p','These lengths are incompatible with the modeled single-stage barrel. Reduce the requested angle range or relocate the piston mounts.','rm-path-message'));
  }
  if (finitePair(raw) && (raw[0] < part.minLength-1e-6 || raw[1] > part.maxLength+1e-6)) {
    out.push(h('p','The requested path extends beyond the current stroke envelope.','rm-path-message'));
  }
  if (mixed) out.push(h('p','The required direction changes along the path. A single push-only or pull-only setting cannot cover these poses.','rm-path-message'));
  if (recommendation?.status === 'partial' || (finitePair(raw) && !completeGeometry)) {
    out.push(h('p','Some target poses could not be resolved. Sample extrema describe the available results.','rm-path-message'));
  }
  if (finitePair(forces)) out.push(h('p','Hold demand is the quasi-static force if this piston alone supplies the holding effort. The installed travel uses the connected mechanism and current stroke limits.','rm-note'));
  if (editable && usableForce && completeGeometry) {
    const apply = button('Apply peak force and direction','use-peak');
    apply.dataset.id = id;
    out.push(apply);
  }
  if (editable && lengthRecommendation && completeGeometry && !candidate.problem && upper-lower > 1e-6) {
    const apply = button('Apply suggested stroke limits','use-path-lengths');
    apply.dataset.id = id;
    out.push(apply);
  }
  return section('Piston travel and force extrema',...out);
}

function multiSelection(model, selections, editable) {
  const objects = selections.map(pick => lookup(model,pick)).filter(Boolean);
  const list = h('ul',null,'rm-selection-list');
  for (const selected of objects) list.append(h('li',label(selected)));
  return section(objects.length + ' items selected',list,editable && button('Delete selected items','delete','rm-delete'));
}
export function inspectorContent({model,design=model,pick,selections=[],page,simulation,path,forces,analysis={},time=0,playing=false,trace=false}) {
  const item=lookup(model,pick), edit=!simulation;
  const title=page==='properties' && selections.length>1 ? 'Selection' :
    item && page==='properties' ? label(item) : page==='motion'?'Motion settings':page==='results'?'Force results':'Properties';
  const blocks=[];
  const jointLabel=id=>{const j=lookup(model,{type:'joint',id});return j?label(j):'—';};
  if(page==='results') {
    const state = chartState(design,path,analysis);
    const sample = state.poses[state.sampleIndex];
    blocks.push(pathControls(design,path,analysis));
    if (state.piston) {
      const part = lookup(model,{type:'element',id:state.piston.id}) ?? state.piston;
      blocks.push(sizeSection(model,part,{
        sample:analysis.preview ? sample?.pistons?.[part.id] : undefined,
        pathEstimate:!!analysis.preview,
        reachable:!analysis.preview || !!sample?.feasible,
      }));
    }
    blocks.push(pathSizing(path,state.piston,edit),forceBreakdown(model,forces,analysis,pick));
  } else if(page==='motion') {
    const o=model.options;
    blocks.push(section('Simulation settings',
      field('Gravity','gravityEnabled',o.gravityEnabled,{type:'checkbox'}),
      field('Gravity (m/s²)','gravity',o.gravity,{min:0}),
      field('Damping','damping',o.damping,{min:0,max:1}),
      field('Solver iterations','iterations',o.iterations,{min:1,max:200,step:1})));
    if(simulation) blocks.push(section('Playback',row('Elapsed',metric(time,'s',2)),button(playing?'Pause':'Run','play'),button('Step one frame','step'),button('Reset pose','reset'),button(trace?'Hide traces':'Trace moving pins','trace'),button('Clear traces','clear-traces')));
  } else if(selections.length>1) {
    blocks.push(multiSelection(model,selections,edit));
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
        if(['rod','body'].includes(item.kind)) {
          props.push(f('Member type','kind',item.kind,{choices:[['rod','Rod'],['body','Weighted body']]}));
        }
        if(isPiston(item)) props.push(row('Connection length',metric(length*1000,'mm')),
          f('Compressed (mm)','minLength',item.minLength*1000,{min:1}),f('Extended (mm)','maxLength',item.maxLength*1000,{min:1}),
          f('Rated force (N)','force',item.force,{min:0}),f('Direction','forceMode',item.forceMode,{choices:[['push','Push / extend'],['pull','Pull / retract']]}),
          f('Barrel end','barrelEnd',item.barrelEnd ?? 'a',{choices:[['a','Start · '+jointLabel(item.a)],['b','End · '+jointLabel(item.b)]]}),
          edit && button('Invert barrel orientation','invert-barrel'),
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
      if(['joint','element','load'].includes(type)) props.push(
        edit ? f('Show force on drawing','showForce',item.showForce !== false,{type:'checkbox'}) : forceToggle(type,item,analysis));
      blocks.push(section(kindName(item.kind),...props));
      if(type==='element' && isPiston(item)) blocks.push(sizeSection(model,item,{editable:edit}),button('Inspect travel and peak force','results'));
      if(type==='joint' && (simulation || forces)) blocks.push(section('Joint forces',...jointForceRows(item,forces?.joints?.[item.id],forces?.status)));
      if(type==='element' && (simulation || forces)) blocks.push(section('Element forces',...elementForceRows(item,forces?.elements?.[item.id],forces?.status)));
      if(edit) blocks.push(button('Delete '+({element:'component',joint:'joint',load:'load',dimension:'dimension',constraint:'constraint'}[type]),'delete','rm-delete'));
    }
  } else blocks.push(h('p','Select a component, joint, dimension or load in the drawing or model tree.','rm-note'));
  return {title,blocks};
}
