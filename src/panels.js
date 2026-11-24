import { balanceForce, assistForce, distance, isPiston, sketchDriven } from './mechanics.js';
import { pathChart } from './path-chart.js';
import { pistonGeometry, pistonLengthRanges, roundPistonLength } from './piston-physics.js';

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
const millimeters = value => metric(value * 1000,'mm',2);
const signed = value => Number.isFinite(value) ? (value > 0 ? '+' : '') + metric(value,'N') : 'Unavailable';
const direction = value => value > 0 ? 'Push / extend' : value < 0 ? 'Pull / retract' : 'Zero';
export const strokeStopLabel = stop => ({min:'Compressed stop',max:'Extended stop',both:'Locked stroke'})[stop];
const finitePair = bounds => Array.isArray(bounds) && bounds.length === 2 && bounds.every(Number.isFinite);
const jointName = (model,id) => model.joints.find(joint => String(joint.id) === String(id))?.name || 'Joint ' + id;
function row(title, value) {
  const line = h('div',null,'rm-readout');
  line.append(h('span',title),h('strong',value));
  return line;
}
function liveReadout(title, value, name) {
  const line = row(title,value);
  line.lastElementChild.dataset.simulationReadout = name;
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
      input.value=Number.isFinite(value) ? Number(value.toFixed(config.decimals ?? 6)) : '';
      if(config.min!=null) input.min=config.min;
      if(config.max!=null) input.max=config.max;
    } else input.value=value || '';
  }
  input.dataset.field=key;
  input.disabled=!!config.disabled;
  wrap.append(input);
  return wrap;
}
function pistonRangeText([minimum, maximum]) {
  const low = roundPistonLength(minimum,'ceil');
  const high = roundPistonLength(maximum,'floor');
  if (low > high) return null;
  return low === high ? millimeters(low) : fmt(low * 1000,2) + '–' + millimeters(high);
}
function pistonLengthFields(part, mountingLength, editable) {
  const makeField = (title,key) => {
    const label = field(title,key,roundPistonLength(part[key])*1000,{min:1,step:.01,decimals:2,disabled:!editable});
    const input = label.querySelector('input');
    input.inputMode = 'decimal';
    if (!editable) return {group:label};
    const group = h('div',null,'rm-piston-length');
    const help = h('div',null,'rm-field-hint');
    help.id = 'piston-length-range-' + key;
    help.setAttribute('aria-live','polite');
    input.setAttribute('aria-describedby',help.id);
    group.append(label,help);
    return {group,input,help};
  };
  const min = makeField('Compressed (mm)','minLength');
  const max = makeField('Extended (mm)','maxLength');
  if (!editable) return [min.group,max.group];
  const refresh = () => {
    const compressed = roundPistonLength(min.input.valueAsNumber / 1000);
    const extended = roundPistonLength(max.input.valueAsNumber / 1000);
    const ranges = pistonLengthRanges(compressed,extended,mountingLength);
    const update = (help,range,otherLength,otherName) => {
      const validOpposite = otherLength >= 0.001;
      const rangeText = validOpposite && range && pistonRangeText(range);
      const message = !validOpposite ? 'Enter at least 1 mm for the ' + otherName + ' length.'
        : rangeText ? 'Fits this mount · ' + rangeText
          : 'No 0.01 mm increment fits. Change the ' + otherName + ' length or the mounting points.';
      if (help.textContent !== message) help.textContent = message;
      help.dataset.fit = String(Boolean(rangeText));
    };
    update(min.help,ranges.compressed,extended,'extended');
    update(max.help,ranges.extended,compressed,'compressed');
  };
  min.input.addEventListener('input',refresh);
  max.input.addEventListener('input',refresh);
  refresh();
  return [min.group,max.group];
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

function magnitude(value) {
  return Number.isFinite(value) ? metric(Math.abs(value),'N') : 'Unavailable';
}

function effortRow(up, down, neutral, effort) {
  const state = Math.abs(effort) < 0.05 ? 'balanced' : effort < 0 ? 'down' : 'up';
  const line = row(state === 'balanced' ? neutral : state === 'down' ? down : up,magnitude(state === 'balanced' ? 0 : effort));
  line.classList.add('rm-balance-effort');
  line.dataset.effort = state;
  return line;
}

function axialBalance(part, load) {
  if (!part || !load) return null;
  const axial = h('div',null,'rm-axial-balance');
  const command = load.commandMode === 'pull' ? 'pull' : 'push';
  const opposite = command === 'push' ? 'pull' : 'push';
  axial.append(h('h4','Piston axis · ' + label(part)));
  const {commandDemandWithoutPiston:demand,commandContribution:provided,commandShortfall:shortfall} = load;
  if (![demand,provided,shortfall].every(Number.isFinite)) {
    axial.append(h('p',load.reason || 'Axial demand is unavailable at this pose.','rm-note'));
    return axial;
  }
  axial.append(
    row(demand < -0.05 ? 'Load assists commanded ' + command : 'Demand without this piston',magnitude(demand) + (demand >= 0.05 ? ' ' + command : '')),
    row(provided < -0.05 ? 'Piston opposes with' : 'Piston supplies',magnitude(provided) + (Math.abs(provided) >= 0.05 ? ' ' + (provided < 0 ? opposite : command) : '')),
    effortRow('Extra ' + command + ' needed','','No extra ' + command + ' needed',shortfall),
  );
  return axial;
}

function forceBalanceSection({model,part,report,load}) {
  if (!part && !report) return null;
  const point = report?.handleId != null && lookup(model,{type:'joint',id:report.handleId});
  const handle = point ? label(point) : report?.handleName;
  const result = section(handle ? 'Force balance at ' + handle : 'Force balance');
  result.classList.add('rm-force-balance');
  result.dataset.forceBalance = '';
  const source = ({
    'measured-rated':'Uses the piston force from the latest simulation step.',
    'configured-in-forced-pose':'Uses the installed piston force at the forced pose.',
    'configured-at-pose':'Uses the installed piston force at the displayed pose.',
    mixed:'Uses measured piston forces where available and installed force estimates elsewhere.',
  })[report?.forceBasis];
  if (Number.isFinite(report?.downwardExternalLoad)) {
    const total=row('Total downward load',magnitude(report.downwardExternalLoad));
    total.title='Weight of the moving assembly plus downward point loads, before piston forces.';
    result.append(total);
  }
  if (report?.status === 'available') {
    result.append(
      row('Load at handle before pistons',handEffort(-report.equivalentExternal)),
      row('Piston contribution at handle',handEffort(report.pistonContribution)),
    );
    if (report.stopSupported) {
      result.append(row('Extra force to hold at the stop',magnitude(0)));
      if (report.stopLocked) result.append(h('p','The installed stroke limits block motion in both directions.','rm-note'));
      else result.append(effortRow('Upward lift to leave the stop','Downward force to leave the stop','No force needed to leave the stop',report.extra));
    } else {
      result.append(effortRow('Extra upward lift needed','Downward restraint needed','Balanced at the handle',report.extra));
    }
  } else {
    result.append(h('p',report?.reason || 'Vertical effort at a handle could not be estimated for this pose.','rm-note'));
  }
  if (source && part) result.append(h('p',source,'rm-note rm-balance-source'));
  const axial = axialBalance(part,load);
  if (axial) result.append(axial);
  return result;
}

function balanceSectionFor({model,design=model,pick,page,simulation,path,forces,pistonLoads=forces?.pistons,balancePistonId,analysis={}}) {
  let part;
  if (page === 'results') {
    const piston = chartState(design,path,analysis).piston;
    part = piston && lookup(model,{type:'element',id:piston.id});
  } else if ((page === 'motion' || page === 'balance') && simulation) {
    const parts = model.elements.filter(isPiston);
    part = parts.find(item=>item.id===balancePistonId) ?? parts.find(item=>pistonLoads?.[item.id]?.overload) ?? parts[0];
  } else if (page === 'properties' && simulation && pick?.type === 'element') {
    const item = lookup(model,pick);
    if (item && isPiston(item)) part = item;
  } else return null;
  if (!part && page==='properties') return null;
  return forceBalanceSection({model,part,report:forces?.handleBalance,load:part && pistonLoads?.[part.id]});
}

export function updateForceBalance(host, inputs) {
  const current = host.querySelector('[data-force-balance]');
  const updated = current && balanceSectionFor(inputs);
  if (updated) current.replaceWith(updated);
}

export function updateSimulationReadouts(host, inputs) {
  updateForceBalance(host,inputs);
  const {model,pick,motionStatus,reversalStatus,time,playing} = inputs;
  const part = pick?.type === 'element' && lookup(model,pick);
  const readouts = {motion:motionStatus, reversal:reversalStatus ?? '—', elapsed:metric(time,'s',2),
    length:part && isPiston(part) ? millimeters(distance(model,part.a,part.b)) : ''};
  for (const output of host.querySelectorAll('[data-simulation-readout]')) {
    output.textContent = readouts[output.dataset.simulationReadout];
  }
  const play = host.querySelector('[data-action="play"]');
  if (play) play.textContent = playing ? 'Pause' : 'Run';
}

function technicalDetails(title, ...contents) {
  const details = h('details',null,'rm-technical-details');
  details.append(h('summary',title),...contents.filter(Boolean));
  return details;
}

function forcedStopsSection(part, editable) {
  const limits = [
    ['Retract stop','forcedMinLength','Use current length for retract'],
    ['Extend stop','forcedMaxLength','Use current length for extend'],
  ];
  const configured = limits.some(([,key]) => Number.isFinite(part[key]));
  if (!editable && !configured) return null;
  const controls = limits.flatMap(([title,key,buttonTitle]) => {
    const value = part[key];
    if (!editable) return [row(title,Number.isFinite(value) ? millimeters(value) : 'Installed limit')];
    const control = field(title + ' (mm)',key,value == null ? NaN : value*1000,{min:1,step:.01,decimals:2});
    const input = control.querySelector('input');
    input.inputMode = 'decimal';
    input.placeholder = 'Installed limit';
    const useCurrent = button(buttonTitle,'use-current-stop','rm-link');
    useCurrent.dataset.stop = key;
    return [control,useCurrent];
  });
  const outside = limits.some(([,key]) => part[key] != null && (part[key] < part.minLength || part[key] > part.maxLength));
  const crossed = part.forcedMinLength != null && part.forcedMaxLength != null && part.forcedMinLength > part.forcedMaxLength;
  const details = technicalDetails('Forced motion stops',
    editable && h('p','Optional pin-to-pin limits for a physical contact such as a closed hatch. Leave a field blank to use the installed stroke.','rm-note'),
    ...controls,
    outside && h('p','A modeled stop is outside the installed stroke. Forced motion still respects the installed limits.','rm-note'),
    crossed && h('p','Set the retract stop at or below the extend stop.','rm-note'));
  details.open = configured;
  return details;
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

function sizeSection(model, part, {editable = false, sample, pathEstimate = false, showHandEffort = true} = {}) {
  const length = sample?.length ?? distance(model,part.a,part.b);
  const fit = strokeInfo(part,length);
  const inStroke = length >= part.minLength-1e-6 && length <= part.maxLength+1e-6;
  const barrelCheck = pistonGeometry(part,Math.min(part.maxLength,Math.max(part.minLength,length)));
  const balance = !pathEstimate && inStroke ? balanceForce(model,part.id) : null;
  const assist = balance ? assistForce(model,part.id) : null;
  const required = pathEstimate ? sample?.force : balance?.force;
  const rated = (part.forceMode === 'pull' ? -1 : 1) * part.force;
  const configured = sample?.configuredForce ?? assist?.configuredForce ?? rated;
  const effort = sample?.assistForce ?? assist?.handForce;
  return section(pathEstimate ? 'Sample piston sizing' : 'Piston sizing at the current pose',
    row('Connection length',millimeters(length)),
    row('Stroke-limited length',millimeters(Math.min(part.maxLength,Math.max(part.minLength,length)))),
    row('Piston range',millimeters(part.minLength) + ' – ' + millimeters(part.maxLength)),
    row('Stroke capacity',millimeters(part.maxLength-part.minLength)),
    row('Stroke check',fit),
    barrelCheck.problem && h('p','Single-stage barrel: ' + barrelCheck.problem + '.','rm-path-message'),
    row('Configured rating (+ push)',signed(rated)),
    configured !== rated && row('Effective configured force',signed(configured)),
    Number.isFinite(required) ? row('Single-piston hold (+ push)',signed(required)) : h('p',
      !inStroke ? pistonState('out_of_stroke') : pathEstimate ? pistonState(sample?.status)
        : 'A grounded pivot, a moving lever and an off-axis attachment are needed to determine hold force at this pose.',
      'rm-note'),
    Number.isFinite(required) && row('Required direction',direction(required)),
    showHandEffort && row('Vertical hand effort',handEffort(effort)),
    showHandEffort && (sample?.handleId ?? assist?.handleId) != null && Number.isFinite(effort)
      && row('Hand effort at',jointName(model,sample?.handleId ?? assist.handleId)),
    balance && row('External moment',metric(balance.moment,'N·m',2)),
    balance && row('Effective arm',millimeters(Math.abs(balance.momentArm))),
    balance && h('p',fmt(Math.abs(balance.moment),2)+' N·m ÷ '+fmt(Math.abs(balance.momentArm),4)+' m = '+metric(Math.abs(required),'N'),'rm-equation'),
    showHandEffort && Number.isFinite(effort) && h('p','Hand effort is the vertical force required to hold this pose with the configured piston.','rm-note'),
    editable && Number.isFinite(required) && button('Use required force','use-required'));
}

const capacityStatuses = {
  ready:'Within rating',
  overload:'Demand exceeds rating',
  assisted:'External load assists motion',
  blocked:'Piston travel is blocked',
  unknown:'Demand unavailable',
  invalid:'Invalid piston geometry',
};

function capacityRows(load) {
  const assessment = load.commandAssessment;
  const commanded = {push:'Push / extend',pull:'Pull / retract'}[load.commandMode];
  const reverse = commanded && load.commandMode !== load.mode;
  const external = assessment === 'externally-driven';
  const items = [
    row('Rated force (+ push)',signed(load.ratedForce)),
  ];
  if (commanded) items.push(row('Commanded direction',commanded));
  if (reverse) {
    items.push(row('Available drive for command (+ push)',signed(load.commandAvailableForce)));
    if (external) {
      items.push(row('Additional external drive required (+ push)',signed(load.commandRequiredForce)));
    } else if (assessment === 'load-assisted' || assessment === 'neutral') {
      items.push(row('Additional external drive required (+ push)',signed(0)));
    }
  } else {
    items.push(row('Available drive (+ push)',signed(load.availableForce)),
      row('Required hold with other drives',signed(load.requiredForce)),
      row('Directional headroom',signed(load.margin)));
  }
  if (Number.isFinite(load.appliedForce)) items.push(row('Applied drive at last step',signed(load.appliedForce)));
  if (!reverse && Number.isFinite(load.utilization)) items.push(row('Available drive utilization',fmt(100 * load.utilization,1) + '%'));
  if (load.atStop) items.push(row('Stroke position',strokeStopLabel(load.atStop)));
  if (Number.isFinite(load.limitForce) && Math.abs(load.limitForce) >= 0.05) {
    items.push(row('Stroke-stop reaction (+ push)',signed(load.limitForce)));
  }
  const status = h('p',capacityStatus(load),'rm-capacity-status');
  status.dataset.capacity = external ? 'overload' : load.status ?? 'unknown';
  items.unshift(status);
  if (load.reason) items.push(h('p',load.reason,'rm-note'));
  return items;
}

function capacityStatus(load) {
  if (load?.commandAssessment === 'externally-driven') return 'Reverse drive overload · external force required';
  if (load?.commandAssessment === 'load-assisted') return 'External load assists commanded motion';
  if (load?.commandAssessment === 'neutral') return 'No external drive required';
  return capacityStatuses[load?.status] ?? 'Demand unavailable';
}

function capacitySection(parts, loads, physicsMode) {
  if (!parts.length) return null;
  const lead = physicsMode === 'forced'
    ? h('p','Forced motion prescribes the piston travel. The load estimate shows whether the installed drive can produce that motion.','rm-note')
    : physicsMode === 'real' ? h('p','Real simulation uses the available piston force. The mechanism can stall or move under load if the rating is insufficient.','rm-note') : null;
  if (parts.length === 1 && loads?.[parts[0].id]) return section('Piston rating and load',lead,...capacityRows(loads[parts[0].id]));
  const list = parts.map(part => {
    const e = h('details',null,'rm-capacity-item');
    const load = loads?.[part.id];
    e.open = !!load?.overload || load?.commandAssessment === 'externally-driven';
    e.append(h('summary',label(part) + ' · ' + capacityStatus(load)),
      ...(load ? capacityRows(load) : [h('p','No capacity estimate is available for this piston.','rm-note')]));
    return e;
  });
  return section('Piston rating and load',lead,...list);
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
  const source = forces?.strokeViolation ? 'The displayed geometry exceeds a piston stroke. Applied inputs are shown without support and member estimates.'
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
  const wrap = field(title,key,value,{min:-360,max:360,step:'any'});
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
  const previewIndex = poses[analysis.previewIndex]?.feasible ? analysis.previewIndex : null;
  return {poses,piston,member,pistons,members,choices,metric,sampleIndex,previewIndex,pistonId:piston?.id,memberId:member?.id};
}

function sampleText(pose) {
  if (!pose) return 'No sample';
  return fmt(pose.angleDeg,2) + '° · ' + (pose.feasible ? 'Reachable' : pose.positions ? 'Unreachable' : 'Unsolved');
}

function previewText(state) {
  const selected = state.poses[state.sampleIndex];
  const displayed = state.poses[state.previewIndex];
  const angle = displayed ? fmt(displayed.angleDeg,2) + '°' : null;
  if (selected.feasible) return angle ? 'Drawing shows the reachable pose at ' + angle + '.' : 'Drawing shows the design pose.';
  const reason = selected.status === 'stroke' ? 'Installed stroke prevents this pose.'
    : selected.positions ? 'This pose is unreachable from the design pose with the installed linkage.'
    : 'Linkage geometry could not be solved at this angle.';
  return reason + (angle ? ' Drawing stays at the last reachable pose (' + angle + ').' : ' Drawing stays at the design pose.');
}

function chartReadout(state) {
  const sample = state.poses[state.sampleIndex];
  const piston = sample?.pistons?.[state.pistonId];
  if (state.metric === 'length' && Number.isFinite(piston?.length)) return 'Connection length · ' + millimeters(piston.length);
  if (state.metric === 'force' && Number.isFinite(piston?.force)) return 'Hold demand · ' + signed(piston.force);
  if (state.metric === 'hand' && sample?.feasible && Number.isFinite(piston?.assistForce)) return 'Vertical hand effort · ' + handEffort(piston.assistForce);
  if (state.metric === 'member' && sample?.feasible && Number.isFinite(sample.memberForces?.[state.memberId])) {
    return 'Member axial force · ' + signed(sample.memberForces[state.memberId]);
  }
  return 'Selected quantity unavailable at this angle.';
}

function renderChart(host, state, sampleInput) {
  host.replaceChildren(pathChart(state.poses,state,index => {
    sampleInput.value = index;
    sampleInput.dispatchEvent(new Event('input',{bubbles:true}));
    sampleInput.dispatchEvent(new Event('change',{bubbles:true}));
  }));
}

export function updateAnalysisCursor(host, inputs) {
  const {model,design=model,path,analysis={}} = inputs;
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
  preview.disabled = !sample?.feasible;
  host.querySelector('[data-action="clear-analysis-pose"]').disabled = state.previewIndex == null;
  host.querySelector('[data-analysis-preview-status]').textContent = previewText(state);
  host.querySelector('[data-analysis-chart-readout]').textContent = chartReadout(state);
  renderChart(chartHost,state,sampleInput);
  updateForceBalance(host,inputs);
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
  const preview = button('Preview sample','preview-analysis-pose');
  preview.dataset.index = state.sampleIndex;
  preview.disabled = !sample.feasible;
  const clear = button('Return to design pose','clear-analysis-pose','rm-link');
  clear.disabled = state.previewIndex == null;
  buttons.append(preview,clear);
  const reachable = state.poses.filter(pose => pose.feasible).length;
  const readout = h('p',chartReadout(state),'rm-analysis-chart-readout');
  readout.dataset.analysisChartReadout = '';
  const drawState = h('p',previewText(state),'rm-note rm-analysis-preview-status');
  drawState.dataset.analysisPreviewStatus = '';
  items.push(chart,slider,readout,buttons,drawState);
  items.push(row('Reachable pose samples',reachable + ' / ' + state.poses.length));
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
  const lower = roundPistonLength(recommendation?.minLength,'floor');
  const upper = roundPistonLength(recommendation?.maxLength,'ceil');
  const lengthRecommendation = Number.isFinite(lower) && Number.isFinite(upper) && upper >= lower && lower >= .001;
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
  if (editable && lengthRecommendation && completeGeometry && !candidate.problem && recommendation.maxLength-recommendation.minLength > 1e-6) {
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
export function inspectorContent({model,design=model,pick,selections=[],page,simulation,path,forces,pistonLoads=forces?.pistons,
  balancePistonId,physicsMode,analysis={},time=0,playing=false,trace=false,motionStatus,reversalStatus}) {
  const item=lookup(model,pick), edit=!simulation;
  const title=page==='properties' && selections.length>1 ? 'Selection' :
    item && page==='properties' ? label(item) : page==='motion'?'Motion settings':page==='results'?'Force results':page==='balance'?'Force balance':'Properties';
  const blocks=[];
  const balance = balanceSectionFor({model,design,pick,page,simulation,path,forces,pistonLoads,balancePistonId,analysis});
  const jointLabel=id=>{const j=lookup(model,{type:'joint',id});return j?label(j):'—';};
  if(page==='results') {
    const state = chartState(design,path,analysis);
    const sample = state.poses[state.previewIndex];
    blocks.push(pathControls(design,path,analysis));
    if (balance) blocks.push(balance);
    if (state.piston) {
      const part = lookup(model,{type:'element',id:state.piston.id}) ?? state.piston;
      blocks.push(technicalDetails('Piston geometry, sizing and capacity',sizeSection(model,part,{
        sample:sample?.pistons?.[part.id],
        pathEstimate:!!sample,
        showHandEffort:false,
      }),capacitySection([part],pistonLoads)));
    }
    blocks.push(pathSizing(path,state.piston,edit),forceBreakdown(model,forces,analysis,pick));
  } else if (page === 'balance') {
    if (balance) {
      blocks.push(balance);
      const parts = model.elements.filter(isPiston);
      if (parts.length > 1) {
        const links = parts.map(part => {
          const control = button(label(part),'pick-piston','rm-link');
          control.dataset.id = part.id;
          return control;
        });
        blocks.push(technicalDetails('Inspect a piston',...links));
      }
      if (parts.length) blocks.push(technicalDetails('Piston ratings and loads',capacitySection(parts,pistonLoads,physicsMode)));
    } else {
      blocks.push(section('Force balance',h('p','Add a piston to compare installed drive with the load at the handle.','rm-note')));
    }
    blocks.push(technicalDetails('Individual forces',forceBreakdown(model,forces,analysis,pick)));
  } else if(page==='motion') {
    const o=model.options;
    if (simulation) {
      const capacity = capacitySection(model.elements.filter(isPiston),pistonLoads,physicsMode);
      if (balance) blocks.push(balance);
      if (capacity) blocks.push(technicalDetails('Piston ratings and loads',capacity));
    }
    blocks.push(section('Simulation settings',
      field('Gravity','gravityEnabled',o.gravityEnabled,{type:'checkbox'}),
      field('Gravity (m/s²)','gravity',o.gravity,{min:0}),
      field('Damping','damping',o.damping,{min:0,max:1}),
      field('Solver iterations','iterations',o.iterations,{min:1,max:200,step:1})));
    if(simulation) blocks.push(section('Playback',liveReadout('Motion',motionStatus,'motion'),liveReadout('Elapsed',metric(time,'s',2),'elapsed'),
      physicsMode==='forced' && liveReadout('Last reversal',reversalStatus ?? '—','reversal'),
      button(playing?'Pause':'Run','play'),button('Step one frame','step'),button('Reset pose','reset'),button(trace?'Hide traces':'Trace moving pins','trace'),button('Clear traces','clear-traces')));
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
        if(isPiston(item)) props.push(simulation ? liveReadout('Connection length',millimeters(length),'length') : row('Connection length',millimeters(length)),
          ...pistonLengthFields(item,length,edit),
          (edit || physicsMode==='forced') && forcedStopsSection(item,edit),
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
      if (balance) blocks.push(balance);
      blocks.push(section(kindName(item.kind),...props));
      if(type==='element' && isPiston(item)) {
        const sizing = sizeSection(model,item,{editable:edit,showHandEffort:!simulation});
        if (simulation) blocks.push(technicalDetails('Piston geometry, sizing and capacity',sizing,capacitySection([item],pistonLoads,physicsMode)));
        else blocks.push(sizing);
        blocks.push(button('Inspect travel and peak force','results'));
      }
      if(type==='joint' && (simulation || forces)) blocks.push(section('Joint forces',...jointForceRows(item,forces?.joints?.[item.id],forces?.status)));
      if(type==='element' && (simulation || forces)) blocks.push(section('Element forces',...elementForceRows(item,forces?.elements?.[item.id],forces?.status)));
      if(edit) blocks.push(button('Delete '+({element:'component',joint:'joint',load:'load',dimension:'dimension',constraint:'constraint'}[type]),'delete','rm-delete'));
    }
  } else blocks.push(h('p','Select a component, joint, dimension or load in the drawing or model tree.','rm-note'));
  return {title,blocks};
}
