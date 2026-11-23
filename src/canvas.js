import { pistonGeometry } from './piston-physics.js';

export { pistonGeometry } from './piston-physics.js';

const SVG_NS = 'http://www.w3.org/2000/svg';
const PART_TOOLS = new Set(['rod', 'body', 'actuator', 'gas_spring', 'motor']);
const JOINT_TOOLS = new Set(['joint', 'pin', 'fixed', 'rotation', 'origin']);
const CONSTRAINT_GLYPHS = {
  horizontal: '—', vertical: '│', coincident: '⊙', parallel: '∥',
  perpendicular: '⊥', equal: '=', equal_length: '=', collinear: '⋯',
};
const PART_NAMES = {
  gas_spring: 'Gas spring', rotation: 'Fixed hinge', fixed: 'Fixed anchor',
  pin: 'Pin joint', point: 'Point load',
};
const MIN_SCALE = 4;
const MAX_SCALE = 80000;
const FORCE_TYPES = {
  gravity: ['showAppliedForces', 'Gravity'],
  applied: ['showAppliedForces', 'Applied'],
  piston: ['showPistonForces', 'Piston'],
  motor: ['showMotorTorques', 'Motor'],
  member: ['showMemberForces', 'Axial'],
  stroke: ['showConstraintForces', 'Stroke stop'],
  constraint: ['showConstraintForces', 'Constraint'],
  reaction: ['showReactions', 'Reaction'],
  resultant: ['showResultantForces', 'Resultant'],
  assist: ['showAssistForces', 'Manual lift'],
};
let drawingId = 0;

function svg(tag, attributes = {}, parent) {
  const node = document.createElementNS(SVG_NS, tag);
  for (const [name, value] of Object.entries(attributes)) node.setAttribute(name, value);
  parent?.append(node);
  return node;
}

function line(parent, a, b, attributes) {
  return svg('line', { x1: a.x, y1: a.y, x2: b.x, y2: b.y, ...attributes }, parent);
}

function circle(parent, p, r, attributes) {
  return svg('circle', { cx: p.x, cy: p.y, r, ...attributes }, parent);
}

function label(parent, p, text, attributes) {
  const node = svg('text', { x: p.x, y: p.y, ...attributes }, parent);
  node.textContent = text;
  return node;
}

function offset(point, dx, dy) {
  return { x: point.x + dx, y: point.y + dy };
}

function midpoint(a, b) {
  return { x: (a.x + b.x) / 2, y: (a.y + b.y) / 2 };
}

function distance(a, b) {
  return Math.hypot(b.x - a.x, b.y - a.y);
}

function angle(a, b) {
  return Math.atan2(b.y - a.y, b.x - a.x);
}

function projectedPoint(p, a, b) {
  const dx = b.x - a.x;
  const dy = b.y - a.y;
  const lengthSquared = dx * dx + dy * dy;
  if (lengthSquared === 0) return a;
  const t = Math.max(0, Math.min(1, ((p.x - a.x) * dx + (p.y - a.y) * dy) / lengthSquared));
  return offset(a, dx * t, dy * t);
}

function selectionBounds(a, b) {
  return { x: Math.min(a.x, b.x), y: Math.min(a.y, b.y), width: Math.abs(a.x - b.x), height: Math.abs(a.y - b.y) };
}

export function marqueeSelection(model, a, b, position, { loads = true } = {}) {
  const { x, y, width, height } = selectionBounds(a, b);
  const crossing = b.x < a.x;
  const inside = (p) => p && p.x >= x && p.x <= x + width && p.y >= y && p.y <= y + height;
  function intersects(start, end) {
    if (!start || !end) return false;
    let low = 0;
    let high = 1;
    for (const [value, delta, min, max] of [[start.x, end.x - start.x, x, x + width], [start.y, end.y - start.y, y, y + height]]) {
      if (Math.abs(delta) < 1e-12) {
        if (value < min || value > max) return false;
      } else {
        const t1 = (min - value) / delta;
        const t2 = (max - value) / delta;
        low = Math.max(low, Math.min(t1, t2));
        high = Math.min(high, Math.max(t1, t2));
        if (low > high) return false;
      }
    }
    return true;
  }

  const result = [];
  for (const joint of model.joints ?? []) {
    if (joint.kind !== 'origin' && inside(position(joint.id))) result.push({ type: 'joint', id: joint.id });
  }
  for (const element of model.elements ?? []) {
    const start = position(element.a);
    const end = position(element.b);
    if (crossing ? intersects(start, end) : inside(start) && inside(end)) result.push({ type: 'element', id: element.id });
  }
  if (loads) for (const load of model.loads ?? []) {
    if (inside(position(load.joint))) result.push({ type: 'load', id: load.id });
  }
  return result;
}

function isPiston(element) {
  return element?.kind === 'actuator' || element?.kind === 'gas_spring';
}

function partName(kind) {
  return PART_NAMES[kind] ?? kind?.replaceAll('_', ' ') ?? 'Part';
}

const decimal = new Intl.NumberFormat('nl-NL', { maximumFractionDigits: 1 });
const meterDecimal = new Intl.NumberFormat('nl-NL', { maximumFractionDigits: 3 });

function shownValue(value, unit) {
  return (unit === 'm' ? meterDecimal : decimal).format(value) + ' ' + unit;
}

export function pistonShortfallLabel(report) {
  if (!(report?.overload ?? (report?.status === 'overload' || report?.utilization > 1))) return null;
  const commandMode = report.commandMode ?? report.mode;
  const commandSign = commandMode === 'pull' ? -1 : commandMode === 'push' ? 1 : Math.sign(report.commandRequiredForce ?? report.requiredForce ?? 0);
  const legacyDemand = report.commandRequiredForce ?? report.requiredForce;
  const legacyCapacity = report.commandAvailableForce ?? report.availableForce;
  const legacyShortfall = Number.isFinite(legacyDemand) && Number.isFinite(legacyCapacity)
    ? legacyDemand * commandSign - Math.abs(legacyCapacity) : null;
  const shortfall = report.commandShortfall ?? (Number.isFinite(report.margin) ? -report.margin : legacyShortfall);
  const sign = Math.sign(report.signedCommandShortfall ?? commandSign);
  if (!Number.isFinite(shortfall) || !sign) return { text: 'Piston overloaded', detail: report.reason ?? 'The amount of extra force is unavailable.' };
  if (shortfall <= 1e-6) return null;

  const direction = sign > 0 ? 'push' : 'pull';
  const text = shownValue(shortfall, 'N') + ' more ' + direction;
  const details = ['Needs ' + text + ' along the piston axis.'];
  const demand = report.demandWithoutPiston ?? report.requiredForce;
  if (Number.isFinite(demand)) {
    details.push(Math.abs(demand) > 1e-6
      ? 'Axial demand without this piston is ' + shownValue(Math.abs(demand), 'N') + ' of ' + (demand > 0 ? 'push.' : 'pull.')
      : 'Axial demand without this piston is zero.');
  }
  const contribution = report.commandContribution ?? (Number.isFinite(report.availableForce) ? report.availableForce * sign : null);
  if (Number.isFinite(contribution)) {
    if (contribution > 1e-6) details.push('The installed piston supplies ' + shownValue(contribution, 'N') + ' toward this motion.');
    else if (contribution < -1e-6) details.push('The installed piston opposes this motion with ' + shownValue(-contribution, 'N') + '.');
    else details.push('The installed piston supplies no force along this axis.');
  }
  return { text, detail: details.join(' ') };
}

export function assistArrowLabel({ fx, fy }, handle) {
  const magnitude = Math.hypot(fx, fy);
  if (!Number.isFinite(magnitude) || magnitude < 1e-8) return null;
  const vertical = Math.abs(fy) > magnitude * 1e-6 ? (fy > 0 ? 'up' : 'down') : '';
  const horizontal = Math.abs(fx) > magnitude * 1e-6 ? (fx > 0 ? 'right' : 'left') : '';
  const direction = [vertical, horizontal].filter(Boolean).join(' and ');
  const action = vertical === 'up' && !horizontal ? 'Manual lift' : 'Manual force';
  return action + ' · ' + shownValue(magnitude, 'N') + ' ' + direction + (handle ? ' at ' + handle : '');
}

function dimensionPoints(a, b, kind, shift) {
  const m = midpoint(a, b);
  if (kind === 'horizontal') return [{ x: a.x, y: m.y + shift }, { x: b.x, y: m.y + shift }];
  if (kind === 'vertical') return [{ x: m.x + shift, y: a.y }, { x: m.x + shift, y: b.y }];
  const direction = angle(a, b);
  const dx = -Math.sin(direction) * shift;
  const dy = Math.cos(direction) * shift;
  return [offset(a, dx, dy), offset(b, dx, dy)];
}

function dimensionFromPlacement(a, b, cursor, aligned) {
  const m = midpoint(a, b);
  const dx = cursor.x - m.x;
  const dy = cursor.y - m.y;
  const width = Math.abs(b.x - a.x);
  const height = Math.abs(b.y - a.y);
  if (aligned) {
    const direction = angle(a, b);
    return { kind: 'distance', value: distance(a, b), offset: -Math.sin(direction) * dx + Math.cos(direction) * dy };
  }
  if (width < 1e-9 || (height >= 1e-9 && Math.abs(dx) > Math.abs(dy))) {
    return { kind: 'vertical', value: height, offset: dx };
  }
  return { kind: 'horizontal', value: width, offset: dy };
}

export function createCanvas(host, handlers = {}) {
  const id = 'mechanism-drawing-' + (++drawingId);
  const root = svg('svg', {
    class: 'mechanism-drawing',
    role: 'application',
    tabindex: '0',
    'aria-label': 'Interactive rod mechanism drawing',
    'aria-describedby': id + '-description',
    width: '100%',
    height: '100%',
  }, host);
  const description = svg('desc', { id: id + '-description' }, root);
  description.textContent = 'Drag over an empty area left to right to select contained items, or right to left to select crossing items. Hold Shift to add to the selection. Use Tab to focus an item and Enter to select it. Use Control-click or Control-Enter to choose a constraint for two items.';

  const defs = svg('defs', {}, root);
  const patterns = ['minor', 'major'].map((name) => {
    const pattern = svg('pattern', { id: id + name, patternUnits: 'userSpaceOnUse' }, defs);
    const path = svg('path', { class: 'drawing-grid-' + name }, pattern);
    const rect = svg('rect', { width: '100%', height: '100%', fill: 'url(#' + id + name + ')' }, root);
    return { pattern, path, rect };
  });
  const geometry = svg('g', { class: 'drawing-geometry' }, root);
  const annotations = svg('g', { class: 'drawing-annotations', 'pointer-events': 'none', 'aria-hidden': 'true' }, root);
  const overlay = svg('g', { class: 'drawing-overlay', 'pointer-events': 'none', 'aria-hidden': 'true' }, root);

  let state = { model: { joints: [], elements: [], loads: [], dimensions: [], constraints: [] }, tool: 'select' };
  let joints = new Map();
  let elements = new Map();
  let size = { width: 0, height: 0 };
  let view = { x: 0, y: 0, scale: 240 };
  let fitted = false;
  let draft = {};
  let hover = null;
  let gesture = null;
  let keyboardPoint = null;
  let constraintPair = null;
  let renderedSelection = new Set();
  const annotationLanes = new Map();
  const torqueRings = new Map();
  const labelBoxes = [];
  const pistonStatuses = [];
  const invalidPistons = new Set();

  const world = (p) => ({ x: (p.x - view.x) / view.scale, y: (view.y - p.y) / view.scale });
  const screen = (p) => ({ x: view.x + p.x * view.scale, y: view.y - p.y * view.scale });
  const activeTool = () => state.readOnly && state.tool !== 'pan' ? 'select' : state.tool;
  const gridSize = () => Number.isFinite(state.gridSize) && state.gridSize > 0 ? state.gridSize : 0.1;
  const selectionKey = ({ type, id }) => type + ':' + id;
  const selected = (type, id) => renderedSelection.has(selectionKey({ type, id }));
  const showPointLoads = () => state.showPointLoads ?? state.showLoads !== false;

  function point(ref) {
    if (typeof ref === 'object') return ref;
    if (gesture?.joint === ref && gesture.point && !gesture.simulated) return gesture.point;
    const positions = state.positions;
    const p = (positions instanceof Map ? positions.get(ref) : positions?.[ref]) ?? joints.get(ref);
    if (p && gesture?.movingJoints?.has(ref) && gesture.delta) {
      return offset(p, gesture.delta.x, gesture.delta.y);
    }
    return p;
  }

  function group(type, item, extraClass = '') {
    const isSelected = selected(type, item.id);
    const paired = constraintPair?.some((selection) => selection.type === type && selection.id === item.id);
    const title = item.name || item.label || (partName(item.kind ?? type) + ' ' + item.id);
    const node = svg('g', {
      class: ('drawing-' + type + ' ' + extraClass + (isSelected ? ' selected' : '') + (paired ? ' paired' : '')).trim(),
      'data-type': type,
      'data-id': item.id,
      role: 'button',
      tabindex: '0',
      'aria-label': title,
      'aria-pressed': isSelected,
    }, geometry);
    svg('title', {}, node).textContent = title;
    return node;
  }

  function drawJoint(joint) {
    const p = point(joint.id);
    if (!p) return;
    const s = screen(p);
    const g = group('joint', joint, 'joint-' + joint.kind);
    circle(g, s, 14, { class: 'drawing-hit' });
    if (selected('joint', joint.id)) {
      circle(g, s, 13, { class: 'joint-halo' });
    }
    const local = svg('g', { transform: 'translate(' + s.x + ' ' + s.y + ')' }, g);
    if (joint.kind === 'fixed' || joint.kind === 'rotation') {
      const support = joint.kind === 'fixed'
        ? 'M-10 5H10 M-9 5l-4 5m9-5-4 5m9-5-4 5m9-5-4 5m9-5-4 5'
        : 'M-3 5l-6 9h18L3 5 M-12 14h24m-21 0-4 4m10-4-4 4m10-4-4 4m10-4-4 4';
      svg('path', { d: support, class: 'joint-ground' }, local);
    }
    if (joint.kind === 'fixed') {
      svg('rect', { x: -4, y: -4, width: 8, height: 8, rx: 1, class: 'joint-centre' }, local);
    } else if (joint.kind === 'origin') {
      svg('path', { d: 'M-10 0h20M0-10v20', class: 'joint-ground' }, local);
      circle(local, { x: 0, y: 0 }, 3, { class: 'joint-centre' });
    } else {
      circle(local, { x: 0, y: 0 }, joint.kind === 'rotation' ? 6.5 : 5, { class: 'joint-centre' });
      circle(local, { x: 0, y: 0 }, 1.65, { class: 'joint-core' });
    }
    if (state.showIds) label(g, offset(s, 11, -10), joint.name || joint.id, { class: 'drawing-label' });
  }

  function addPistonStatus(parent, anchor, text, description) {
    parent.setAttribute('aria-label', parent.getAttribute('aria-label') + '. ' + description);
    const title = parent.querySelector('title');
    title.textContent += '. ' + description;
    pistonStatuses.push({ parent, anchor, text });
  }

  function drawInvalidPiston(g, parent, length) {
    parent.classList.add('piston-problem');
    parent.setAttribute('data-piston-geometry', 'invalid');
    line(g, { x: 0, y: 0 }, { x: length, y: 0 }, { class: 'piston-mount-reference' });
    for (const x of [0, length]) {
      circle(g, { x, y: 0 }, 11, { class: 'drawing-hit' });
      svg('path', { d: 'M0 -10L10 0 0 10 -10 0Z', transform: 'translate(' + x + ' 0)', class: 'piston-mount-indicator' }, g);
    }
  }

  function drawPiston(element, g, parent, start, end, length, centerLength) {
    const piston = pistonGeometry(element, centerLength);
    const anchor = offset(midpoint(start, end), 4, 13);
    if (!piston.valid) {
      invalidPistons.add(element.id);
      drawInvalidPiston(g, parent, length);
      addPistonStatus(parent, anchor, piston.problem, 'Invalid piston geometry. ' + piston.problem);
      return;
    }
    parent.setAttribute('data-piston-geometry', 'physical');
    const fitting = piston.fittingLength * view.scale;
    const front = fitting + piston.barrelLength * view.scale;
    const radius = Math.min(24, Math.max(4, piston.barrelRadius * view.scale));
    const endFitting = Math.max(front, length - fitting);
    const along = (from, to, className) => line(g, { x: from, y: 0 }, { x: to, y: 0 }, { class: className });
    along(0, fitting, 'piston-fitting');
    along(front, endFitting, 'piston-rod');
    along(Math.min(length, endFitting), length, 'piston-fitting');
    const body = svg('rect', { x: fitting, y: -radius, width: front - fitting, height: radius * 2, rx: Math.min(radius / 3, 3), class: 'piston-cylinder' }, g);
    body.setAttribute('data-barrel-length', piston.barrelLength);
    line(g, { x: front, y: -radius }, { x: front, y: radius }, { class: 'piston-seal' });
    line(g, { x: fitting, y: -radius }, { x: fitting, y: radius }, { class: 'piston-seal' });
    for (const x of [0, length]) circle(g, { x, y: 0 }, Math.min(11, Math.max(7, radius * 0.7)), { class: 'piston-eye' });
    if (state.showStrokeLimits !== false) {
      for (const [kind, value] of [['compressed', piston.minLength], ['extended', piston.maxLength]]) {
        if (!Number.isFinite(value) || value <= 0) continue;
        const x = value * view.scale;
        line(g, { x, y: -radius - 5 }, { x, y: radius + 5 }, { class: 'piston-limit piston-limit-' + kind });
      }
    }
    const report = state.pistonLoads instanceof Map ? state.pistonLoads.get(element.id) : state.pistonLoads?.[element.id];
    const shortfall = pistonShortfallLabel(report);
    if (shortfall) {
      parent.classList.add('piston-overload');
      parent.setAttribute('data-piston-overload', 'true');
      addPistonStatus(parent, anchor, shortfall.text, shortfall.detail);
    }
  }

  function drawPart(element, parent, a, b, centerLength) {
    const piston = isPiston(element);
    if (piston && element.barrelEnd === 'b') [a, b] = [b, a];
    const length = distance(a, b);
    if (!length && !piston) return;
    const degrees = angle(a, b) * 180 / Math.PI;
    const g = svg('g', { transform: 'translate(' + a.x + ' ' + a.y + ') rotate(' + degrees + ')' }, parent);
    const stroke = (from, to, className) => line(g, { x: from, y: 0 }, { x: to, y: 0 }, { class: className });
    stroke(0, length, 'drawing-hit');
    if (element.kind === 'body') {
      stroke(0, length, 'body-edge');
      stroke(0, length, 'body-fill');
      const middle = { x: length / 2, y: 0 };
      if (length > 34) circle(g, middle, 3, { class: 'body-mass' });
    } else if (piston) {
      drawPiston(element, g, parent, a, b, length, centerLength);
    } else if (element.kind === 'motor') {
      stroke(0, length, 'motor-link');
      const center = Math.min(21, length / 2);
      circle(g, { x: center, y: 0 }, 9, { class: 'motor-case' });
      const direction = Math.sign(element.torque ?? 1) || 1;
      const end = direction < 0 ? center - 4 : center + 4;
      const start = 2 * center - end;
      svg('path', { d: 'M' + start + ' 2 A5 5 0 1 ' + Number(direction >= 0) + ' ' + end + ' 2 M' + end + ' 2 l' + (-direction * 4) + ' 0 m' + (direction * 4) + ' 0 v-4', class: 'motor-arrow' }, g);
    } else {
      stroke(0, length, 'rod-stroke');
    }
  }

  function drawElement(element) {
    const a = point(element.a);
    const b = point(element.b);
    if (!a || !b) return;
    const p = screen(a);
    const q = screen(b);
    const g = group('element', element, 'element-' + element.kind);
    drawPart(element, g, p, q, distance(a, b));
    if (state.showIds) {
      const name = element.name || element.id;
      const textPoint = offset(midpoint(p, q), 0, -12);
      reserveLabel(textPoint, name, true);
      label(g, textPoint, name, { class: 'drawing-label', 'text-anchor': 'middle' });
    }
  }

  function arrow(parent, a, b, className) {
    const heading = angle(a, b);
    arrowHead(parent, b, heading, className + '-head');
    line(parent, a, b, { class: className });
  }

  function arrowHead(parent, b, heading, className) {
    const side = (delta) => offset(b, -8 * Math.cos(heading + delta), -8 * Math.sin(heading + delta));
    const left = side(0.5);
    const right = side(-0.5);
    svg('polygon', { points: b.x + ',' + b.y + ' ' + left.x + ',' + left.y + ' ' + right.x + ',' + right.y, class: className }, parent);
  }

  function annotationLane(origin, dx, dy) {
    // Parallel and opposite arrows share the same lateral channel counter.
    const bucket = Math.round(Math.atan2(dy, dx) * 12 / Math.PI);
    const directionClass = ((bucket % 12) + 12) % 12;
    const key = [Math.round(origin.x / 10), Math.round(origin.y / 10), directionClass].join(',');
    const index = annotationLanes.get(key) ?? 0;
    annotationLanes.set(key, index + 1);
    const lane = index === 0 ? 0 : Math.ceil(index / 2) * (index % 2 ? 1 : -1);
    const sign = dx < -1e-7 || (Math.abs(dx) < 1e-7 && dy < 0) ? -1 : 1;
    return lane * 14 * sign;
  }

  function reserveLabel(p, text, centered = false) {
    const width = String(text).length * 6.2 + 6;
    labelBoxes.push({ x: p.x - (centered ? width / 2 : 0), y: p.y, width });
  }

  function placeAnnotation(anchor, width) {
    const candidates = [
      [7, -8], [-width - 7, -8], [7, 9], [-width - 7, 9],
      [7, -24], [-width - 7, -24], [7, 25], [-width - 7, 25],
      [7, -40], [-width - 7, -40], [7, 41], [-width - 7, 41],
      [7, -56], [-width - 7, -56], [7, 57], [-width - 7, 57],
    ];
    const inside = ({ x, y }) => x >= 4 && x + width <= size.width - 4 && y >= 16 && y <= size.height - 4;
    const overlaps = ({ x, y }) => labelBoxes.some((box) => x < box.x + box.width && x + width > box.x && y > box.y - 14 && y - 14 < box.y);
    const points = candidates.map(([dx, dy]) => offset(anchor, dx, dy));
    const p = points.find((candidate) => inside(candidate) && !overlaps(candidate)) ?? points.find(inside) ?? points[0];
    labelBoxes.push({ ...p, width });
    return p;
  }

  function annotationLabel(parent, anchor, text, className) {
    const p = placeAnnotation(anchor, Math.max(20, text.length * 6.2 + 6));
    return label(parent, p, text, { class: className });
  }

  function drawPistonStatuses() {
    for (const { parent, anchor, text } of pistonStatuses) {
      const width = Math.max(52, text.length * 6.5 + 16);
      const p = placeAnnotation(anchor, width);
      const status = svg('g', { class: 'piston-status', 'aria-hidden': 'true' }, parent);
      svg('rect', { x: p.x, y: p.y - 13, width, height: 18, rx: 4, class: 'piston-status-box' }, status);
      label(status, offset(p, 8, 0), text, { class: 'piston-status-label' });
    }
  }

  function drawLoad(load) {
    if ((state.forceDisplay?.('load',load.id) ?? load.showForce) === false) return;
    const p = point(load.joint);
    if (!p) return;
    const g = group('load', load);
    if (load.enabled === false) g.classList.add('load-disabled');
    const s = screen(p);
    const magnitude = Math.hypot(load.fx, load.fy);
    const nx = magnitude ? load.fx / magnitude : 0;
    const ny = magnitude ? -load.fy / magnitude : 1;
    const lane = annotationLane(s, nx, ny);
    const base = offset(s, -ny * lane, nx * lane);
    if (magnitude < 1e-9 && !selected('load', load.id)) {
      const marker = offset(base, 13, -13);
      circle(g, marker, 8, { class: 'drawing-hit' });
      circle(g, marker, 3, { class: 'force-placeholder' });
      return;
    }
    const a = offset(base, -nx * 55, -ny * 55);
    const b = offset(base, -nx * 7, -ny * 7);
    line(g, a, b, { class: 'drawing-hit' });
    if (lane) line(g, s, b, { class: 'force-leader' });
    arrow(g, a, b, 'force-arrow');
    annotationLabel(g, a, shownValue(magnitude, 'N'), 'force-label');
  }

  function drawDimension(parent, dim) {
    const a = point(dim.a);
    const b = point(dim.b);
    if (!a || !b) return;
    const shift = gesture?.dimension === dim.id && gesture.dimensionOffset != null ? gesture.dimensionOffset : dim.offset;
    const [start, end] = dimensionPoints(a, b, dim.kind, shift ?? 0.12).map(screen);
    const middle = midpoint(start, end);
    const g = dim.id == null ? svg('g', { class: 'drawing-dimension dimension-preview' }, parent) : group('dimension', dim, dim.driving === false ? 'dimension-reference' : '');
    line(g, screen(a), start, { class: 'dimension-extension' });
    line(g, screen(b), end, { class: 'dimension-extension' });
    line(g, start, end, { class: 'drawing-hit' });
    line(g, start, end, { class: 'dimension-line' });
    for (const p of [start, end]) line(g, offset(p, -3, 3), offset(p, 3, -3), { class: 'dimension-line' });
    const measured = dim.kind === 'horizontal' ? Math.abs(b.x - a.x) : dim.kind === 'vertical' ? Math.abs(b.y - a.y) : distance(a, b);
    const value = state.readOnly || dim.driving === false ? measured : (dim.value ?? measured);
    const unit = state.dimensionUnits === 'm' ? 'm' : 'mm';
    const text = shownValue(unit === 'mm' ? value * 1000 : value, unit);
    const width = Math.max(43, text.length * 6.4 + 12);
    if (dim.id != null) labelBoxes.push({ x: middle.x - width / 2, y: middle.y + 8, width });
    svg('rect', { x: middle.x - width / 2, y: middle.y - 9, width, height: 18, rx: 4, class: 'dimension-box' }, g);
    label(g, offset(middle, 0, 0.5), text, { class: 'dimension-label', 'text-anchor': 'middle', 'dominant-baseline': 'middle' });
  }

  function drawConstraint(constraint) {
    const element = elements.get(constraint.element);
    const a = point(element?.a ?? constraint.a);
    const b = point(element?.b ?? constraint.b);
    if (!a) return;
    const p = offset(screen(b ? midpoint(a, b) : a), 0, 17);
    const g = group('constraint', constraint);
    svg('rect', { x: p.x - 10, y: p.y - 9, width: 20, height: 18, rx: 4, class: 'constraint-box' }, g);
    label(g, p, CONSTRAINT_GLYPHS[constraint.kind] ?? 'C', { class: 'constraint-label', 'text-anchor': 'middle', 'dominant-baseline': 'middle' });
  }

  function drawTrace(trace, className) {
    if (!trace || trace.length < 2) return;
    let section = [];
    function drawSection() {
      if (section.length < 2 || section.every((p) => distance(p, section[0]) < 1e-8)) return;
      const points = section.map(screen).map((p) => p.x + ',' + p.y).join(' ');
      svg('polyline', { points, class: className, 'pointer-events': 'none' }, geometry);
    }
    for (const value of trace) {
      const p = Array.isArray(value) ? { x: value[0], y: value[1] } : value;
      if (Number.isFinite(p?.x) && Number.isFinite(p?.y)) section.push(p);
      else {
        drawSection();
        section = [];
      }
    }
    drawSection();
  }

  function drawTraces() {
    if (state.showTraces !== false) {
      const paths = state.traces instanceof Map ? state.traces.values() : Object.values(state.traces ?? {});
      for (const trace of paths) drawTrace(trace, 'joint-trace');
    }
    if (state.showExcursion === false) return;
    const featured = elements.get(state.selection?.type === 'element' ? state.selection.id : null);
    const piston = isPiston(featured) ? featured : null;
    const highlighted = state.excursionJoint ?? (piston?.barrelEnd === 'b' ? piston.a : piston?.b);
    const className = (id) => 'excursion-path' + (String(id) === String(highlighted) ? ' excursion-featured' : '');
    if (state.excursionPoses) {
      const pistons = [...elements.values()].filter(isPiston);
      const at = (positions, id) => positions instanceof Map ? positions.get(id) : positions?.[id];
      const connected = state.excursionPoses.map((pose) => {
        if (!pose.feasible || !pose.positions) return null;
        const usable = pistons.every((element) => {
          const a = at(pose.positions, element.a);
          const b = at(pose.positions, element.b);
          return a && b && pistonGeometry(element, distance(a, b)).valid;
        });
        return usable ? pose.positions : null;
      });
      for (const joint of joints.values()) {
        if (joint.kind === 'origin' || joint.kind === 'fixed' || joint.kind === 'rotation') continue;
        drawTrace(connected.map((positions) => at(positions, joint.id)), className(joint.id));
      }
    } else {
      const paths = state.excursionPaths instanceof Map ? state.excursionPaths : Object.entries(state.excursionPaths ?? {});
      for (const [id, trace] of paths) drawTrace(trace, className(id));
    }
  }

  function drawMomentArm() {
    const arm = state.momentArm;
    if (!arm?.pivot || !arm?.foot) return;
    if (invalidPistons.has(arm.element ?? state.selection?.id)) return;
    const a = screen(arm.pivot);
    const b = screen(arm.foot);
    const g = svg('g', { class: 'drawing-moment-arm' }, annotations);
    if (arm.linePoint) line(g, screen(arm.linePoint), b, { class: 'moment-extension' });
    line(g, a, b, { class: 'moment-arm' });
    const theta = angle(b, a);
    svg('path', { d: 'M0 7H7V0', transform: 'translate(' + b.x + ' ' + b.y + ') rotate(' + (theta * 180 / Math.PI - 90) + ')', class: 'moment-corner' }, g);
    label(g, offset(midpoint(a, b), 7, -5), shownValue(distance(arm.pivot, arm.foot) * 1000, 'mm'), { class: 'moment-label' });
  }

  function drawTorque(force, pivot) {
    const direction = force.torque > 0 ? -1 : 1;
    const key = Math.round(pivot.x / 10) + ',' + Math.round(pivot.y / 10);
    const ring = torqueRings.get(key) ?? 0;
    torqueRings.set(key, ring + 1);
    const radius = 22 + 11 * ring;
    const startAngle = -Math.PI / 4;
    const endAngle = startAngle + direction * 4.6;
    const arcPoint = (theta) => offset(pivot, radius * Math.cos(theta), radius * Math.sin(theta));
    const a = arcPoint(startAngle);
    const b = arcPoint(endAngle);
    const group = svg('g', { class: 'force-kind-motor force-couple', 'data-force-kind': 'motor-torque' }, annotations);
    svg('path', { d: 'M' + a.x + ' ' + a.y + 'A' + radius + ' ' + radius + ' 0 1 ' + Number(direction > 0) + ' ' + b.x + ' ' + b.y, class: 'result-force' }, group);
    arrowHead(group, b, endAngle + direction * Math.PI / 2, 'result-force-head');
    annotationLabel(group, offset(pivot, radius, -radius), shownValue(force.torque, 'N·m'), 'result-force-label');
  }

  function drawForces() {
    if (state.showForces === false) return;
    const arrows = (state.forces?.arrows ?? state.forces ?? []).slice();
    if (state.showResultantForces && ['balanced', 'dynamic'].includes(state.forces?.status) && !arrows.some((force) => force.kind === 'resultant')) {
      for (const joint of joints.values()) {
        const net = state.forces?.joints?.[joint.id]?.net;
        if (net) arrows.push({ joint: joint.id, fx: net.x, fy: net.y, kind: 'resultant' });
      }
    }
    const hasTorque = new Set();
    for (const force of arrows) {
      const kind = force.kind ?? force.category ?? 'reaction';
      if (kind === 'load') continue;
      if (invalidPistons.has(force.element)) continue;
      const partType = force.element != null ? 'element' : 'joint';
      const item = force.element != null ? elements.get(force.element) : joints.get(force.joint);
      if (item && (state.forceDisplay?.(partType, item.id) ?? item.showForce) === false) continue;
      const [visibility, title] = FORCE_TYPES[kind] ?? FORCE_TYPES.applied;
      if (!(state[visibility] ?? state.showLoads !== false)) continue;
      const origin = force.point ?? point(force.joint) ?? (Number.isFinite(force.x) && Number.isFinite(force.y) ? force : null);
      if (!origin) continue;
      const pointOnScreen = screen(origin);
      if (force.torque) {
        hasTorque.add(force.element);
        const torqueOrigin = Number.isFinite(force.x) && Number.isFinite(force.y) ? screen(force) : pointOnScreen;
        drawTorque(force, torqueOrigin);
      }
      const magnitude = Math.hypot(force.fx, force.fy);
      if (!Number.isFinite(magnitude) || magnitude < 1e-8) continue;
      const dx = force.fx / magnitude;
      const dy = -force.fy / magnitude;
      const lane = annotationLane(pointOnScreen, dx, dy);
      const length = Math.min(82, 22 + 10 * Math.log10(magnitude + 1));
      const base = offset(pointOnScreen, -dy * lane, dx * lane);
      const a = offset(base, dx * 8, dy * 8);
      const b = offset(base, dx * length, dy * length);
      const group = svg('g', { class: 'force-kind-' + kind, 'data-force-kind': kind }, annotations);
      if (lane) line(group, pointOnScreen, a, { class: 'force-leader' });
      arrow(group, a, b, 'result-force');
      const handle = force.handleName ?? (force.joint != null ? joints.get(force.joint)?.name || 'joint ' + force.joint : null);
      const text = kind === 'assist' ? assistArrowLabel(force, handle) : (force.label ?? title) + ' ' + shownValue(magnitude, 'N');
      annotationLabel(group, b, text, 'result-force-label');
    }
    if (state.showMotorTorques) {
      for (const element of elements.values()) {
        if (element.kind !== 'motor' || !element.torque || (state.forceDisplay?.('element', element.id) ?? element.showForce) === false || hasTorque.has(element.id)) continue;
        const pivot = point(element.a);
        if (pivot) drawTorque(element, screen(pivot));
      }
    }
  }

  function updateGrid() {
    let minor = view.scale * gridSize();
    while (minor > 70) minor /= 2;
    while (minor < 15) minor *= 2;
    const major = minor * 5;
    patterns.forEach(({ pattern, path, rect }, index) => {
      const spacing = index === 0 ? minor : major;
      pattern.setAttribute('x', view.x % spacing);
      pattern.setAttribute('y', view.y % spacing);
      pattern.setAttribute('width', spacing);
      pattern.setAttribute('height', spacing);
      path.setAttribute('d', 'M' + spacing + ' 0H0V' + spacing);
      rect.setAttribute('visibility', state.grid === false ? 'hidden' : 'visible');
    });
  }

  function drawPreview() {
    overlay.replaceChildren();
    if (gesture?.marquee && gesture.end) {
      svg('rect', {
        ...selectionBounds(gesture.start, gesture.end),
        class: 'selection-box' + (gesture.end.x < gesture.start.x ? ' selection-crossing' : ''),
      }, overlay);
      return;
    }
    if (draft.start != null) {
      const a = point(draft.start);
      if (a) {
        const p = screen(a);
        circle(overlay, p, 11, { class: 'draft-anchor' });
        if (hover && PART_TOOLS.has(activeTool())) line(overlay, p, screen(hover.point), { class: 'draft-line' });
      }
    }
    if (draft.pair && hover) {
      const [a, b] = draft.pair;
      const p = point(a);
      const q = point(b);
      if (p && q) drawDimension(overlay, { a, b, ...dimensionFromPlacement(p, q, hover.raw, hover.shiftKey) });
      return;
    }
    if (!hover || gesture?.pan) return;
    const tool = activeTool();
    if ((tool === 'select' || tool === 'pan') && !keyboardPoint && !gesture?.joint) return;
    const p = screen(hover.point);
    svg('path', { d: 'M' + (p.x - 5) + ' ' + p.y + 'h10 M' + p.x + ' ' + (p.y - 5) + 'v10', class: 'snap-cross' }, overlay);
    circle(overlay, p, hover.id != null ? 11 : 8, { class: 'snap-ring ' + (hover.mode === 'joint' ? 'snap-joint' : ''), 'stroke-dasharray': hover.mode === 'grid' ? '2 2' : 'none' });
    if (hover.mode === 'segment') {
      svg('rect', { x: p.x - 4, y: p.y - 4, width: 8, height: 8, class: 'snap-segment', transform: 'rotate(45 ' + p.x + ' ' + p.y + ')' }, overlay);
    }
  }

  function paint() {
    const previous = document.activeElement?.closest?.('[data-type]');
    const shouldRestore = previous && root.contains(previous);
    const oldType = shouldRestore ? previous.dataset.type : '';
    const oldId = shouldRestore ? previous.dataset.id : '';
    renderedSelection = new Set((state.selectionItems ?? (state.selection ? [state.selection] : [])).map(selectionKey));
    root.setAttribute('class', 'mechanism-drawing tool-' + activeTool() + (gesture?.pan ? ' is-panning' : '') + (gesture?.joint != null ? ' is-dragging' : ''));
    updateGrid();
    geometry.replaceChildren();
    annotations.replaceChildren();
    annotationLanes.clear();
    torqueRings.clear();
    labelBoxes.length = 0;
    pistonStatuses.length = 0;
    invalidPistons.clear();
    drawTraces();
    for (const element of elements.values()) drawElement(element);
    if (state.showDimensions !== false) {
      for (const dimension of state.model.dimensions ?? []) {
        if (dimension.visible !== false) drawDimension(geometry, dimension);
      }
    }
    if (state.showConstraints !== false) for (const constraint of state.model.constraints ?? []) {
      if (constraint.visible !== false) drawConstraint(constraint);
    }
    if (state.showIds) for (const joint of joints.values()) {
      const p = point(joint.id);
      if (p) reserveLabel(offset(screen(p), 11, -10), joint.name || joint.id);
    }
    drawPistonStatuses();
    if (showPointLoads()) for (const load of state.model.loads ?? []) drawLoad(load);
    for (const joint of joints.values()) drawJoint(joint);
    drawForces();
    drawMomentArm();
    drawPreview();
    if (shouldRestore) {
      const replacement = Array.from(geometry.children).find((node) => node.dataset.type === oldType && node.dataset.id === oldId);
      (replacement ?? root).focus({ preventScroll: true });
    }
  }

  function viewport() {
    const insets = state.insets ?? {};
    const left = insets.left ?? 0;
    const top = insets.top ?? 0;
    const width = Math.max(1, size.width - left - (insets.right ?? 0));
    const height = Math.max(1, size.height - top - (insets.bottom ?? 0));
    return { x: left + width / 2, y: top + height / 2, width, height };
  }

  function setView(next) {
    view = {
      x: Number.isFinite(next.x) ? next.x : view.x,
      y: Number.isFinite(next.y) ? next.y : view.y,
      scale: Number.isFinite(next.scale) ? Math.min(MAX_SCALE, Math.max(MIN_SCALE, next.scale)) : view.scale,
    };
    fitted = true;
    paint();
    handlers.onView?.({ ...view });
  }

  function fit() {
    if (!size.width || !size.height) {
      fitted = false;
      return;
    }
    const bounds = viewport();
    const points = Array.from(joints.keys(), point).filter(Boolean);
    if (!points.length) {
      setView({ x: bounds.x, y: bounds.y, scale: 240 });
      return;
    }
    let minX = Infinity;
    let minY = Infinity;
    let maxX = -Infinity;
    let maxY = -Infinity;
    for (const p of points) {
      minX = Math.min(minX, p.x);
      minY = Math.min(minY, p.y);
      maxX = Math.max(maxX, p.x);
      maxY = Math.max(maxY, p.y);
    }
    if (state.showDimensions !== false) {
      for (const dim of state.model.dimensions ?? []) {
        if (dim.visible === false) continue;
        const a = point(dim.a);
        const b = point(dim.b);
        if (!a || !b) continue;
        for (const p of dimensionPoints(a, b, dim.kind, dim.offset ?? 0.12)) {
          minX = Math.min(minX, p.x);
          minY = Math.min(minY, p.y);
          maxX = Math.max(maxX, p.x);
          maxY = Math.max(maxY, p.y);
        }
      }
    }
    const pad = Math.min(90, 0.15 * Math.min(bounds.width, bounds.height));
    const scale = Math.max(MIN_SCALE, Math.min(MAX_SCALE, (bounds.width - 2 * pad) / Math.max(0.4, maxX - minX), (bounds.height - 2 * pad) / Math.max(0.4, maxY - minY)));
    setView({ x: bounds.x - (minX + maxX) / 2 * scale, y: bounds.y + (minY + maxY) / 2 * scale, scale });
  }

  function zoom(factor, center = viewport()) {
    if (!Number.isFinite(factor) || factor <= 0) return;
    const p = world(center);
    const scale = Math.min(MAX_SCALE, Math.max(MIN_SCALE, view.scale * factor));
    setView({ x: center.x - p.x * scale, y: center.y + p.y * scale, scale });
  }

  function resize() {
    const next = { width: root.clientWidth, height: root.clientHeight };
    if (size.width === next.width && size.height === next.height) return;
    const prior = viewport();
    size = next;
    if (!fitted) fit();
    else {
      const current = viewport();
      setView(offsetView(current.x - prior.x, current.y - prior.y));
    }
  }

  function offsetView(dx, dy) {
    return { ...view, x: view.x + dx, y: view.y + dy };
  }

  function hit(eventTarget) {
    const target = eventTarget?.closest?.('[data-type]');
    if (!target || !root.contains(target)) return null;
    const type = target.dataset.type;
    const rawId = target.dataset.id;
    const collection = type === 'joint' ? joints.values() : type === 'element' ? elements.values() : state.model[type + 's'] ?? [];
    const item = Array.from(collection).find((candidate) => String(candidate.id) === rawId);
    return item ? { type, id: item.id } : null;
  }

  function snapPoint(raw, { jointsOnly = false, segments = false, excludeOrigin = false, tolerance = 12 } = {}) {
    let best;
    let closest = tolerance / view.scale;
    for (const joint of joints.values()) {
      if (excludeOrigin && joint.kind === 'origin') continue;
      const p = point(joint.id);
      const delta = distance(raw, p);
      if (delta > closest) continue;
      best = { point: p, id: joint.id, mode: 'joint', raw };
      closest = delta;
    }
    if (best || jointsOnly) return best ?? { point: raw, raw };
    if (segments && state.snap !== false) {
      for (const element of elements.values()) {
        if (element.kind !== 'rod' && element.kind !== 'body') continue;
        const a = point(element.a);
        const b = point(element.b);
        if (!a || !b) continue;
        const p = projectedPoint(raw, a, b);
        const delta = distance(p, raw);
        if (delta > closest || p === a || p === b || distance(p, a) < 1e-8 || distance(p, b) < 1e-8) continue;
        best = { point: p, splitElement: element.id, mode: 'segment', raw };
        closest = delta;
      }
    }
    if (best) return best;
    if (state.snap === false) return { point: raw, raw };
    const step = gridSize();
    const p = { x: Math.round(raw.x / step) * step, y: Math.round(raw.y / step) * step };
    return { point: p, mode: 'grid', raw };
  }

  function eventPosition(event) {
    const bounds = root.getBoundingClientRect();
    return { x: event.clientX - bounds.left, y: event.clientY - bounds.top };
  }

  function currentHover(p, shiftKey, tolerance) {
    const tool = activeTool();
    const raw = world(p);
    const snapped = snapPoint(raw, {
      jointsOnly: tool === 'dimension' || tool === 'select',
      segments: PART_TOOLS.has(tool) || JOINT_TOOLS.has(tool) || tool === 'load',
      excludeOrigin: PART_TOOLS.has(tool) || tool === 'load',
      tolerance,
    });
    return { ...snapped, shiftKey };
  }

  function endpoint(snapped, kind = 'pin') {
    if (snapped.id != null) return snapped.id;
    const ref = { ...snapped.point, kind };
    if (snapped.splitElement != null) ref.splitElement = snapped.splitElement;
    return ref;
  }

  function hint(text) {
    handlers.onHint?.(text);
  }

  function useDimension(selected, cursor) {
    if (draft.pair) {
      const [a, b] = draft.pair;
      const p = point(a);
      const q = point(b);
      draft = {};
      if (!p || !q || distance(p, q) < 1e-9) return;
      handlers.onAddDimension?.({ a, b, ...dimensionFromPlacement(p, q, cursor.raw, cursor.shiftKey) });
      return;
    }
    if (selected?.type === 'element' && draft.start == null) {
      const element = elements.get(selected.id);
      draft.pair = [element.a, element.b];
    } else if (cursor.id != null && cursor.id !== draft.start) {
      if (draft.start == null) {
        draft.start = cursor.id;
        hint('Select a second point.');
        return;
      }
      draft.pair = [draft.start, cursor.id];
      delete draft.start;
    }
    if (draft.pair) hint('Click to place the dimension. Hold Shift for aligned length.');
    else hint('Select an element or two existing joints.');
  }

  function useTool(selected, cursor) {
    const tool = activeTool();
    if (tool === 'select') {
      handlers.onSelect?.(selected ?? (cursor.id != null ? { type: 'joint', id: cursor.id } : null));
    } else if (JOINT_TOOLS.has(tool)) {
      const kind = tool === 'joint' ? 'pin' : tool;
      if (cursor.id != null) {
        handlers.onAddJoint?.({ id: cursor.id, x: cursor.point.x, y: cursor.point.y, kind });
      } else {
        handlers.onAddJoint?.(endpoint(cursor, kind));
      }
    } else if (PART_TOOLS.has(tool)) {
      const ref = endpoint(cursor);
      if (draft.start == null) {
        draft.start = ref;
        hint('Select the second endpoint.');
      } else {
        const a = draft.start;
        const p = point(a);
        if (!p || distance(p, cursor.point) < 1e-8) {
          hint('Select a different endpoint.');
          return;
        }
        draft = {};
        handlers.onAddElement?.({ kind: tool, a, b: ref });
      }
    } else if (tool === 'load') {
      if (cursor.id != null || cursor.splitElement != null) handlers.onAddLoad?.({ joint: endpoint(cursor), fx: 0, fy: -100 });
      else hint('Select a joint or rod segment to place the point load.');
    } else if (tool === 'dimension') {
      useDimension(cursor.id != null ? { type: 'joint', id: cursor.id } : selected, cursor);
    } else if (tool === 'constraint') {
      handlers.onConstraintPick?.(selected);
    }
  }

  function selectPair(selected, anchor) {
    const first = state.selection;
    if (!first || (selected.type === first.type && selected.id === first.id)) {
      constraintPair = null;
      handlers.onSelect?.(selected);
      return;
    }
    constraintPair = [first, selected];
    handlers.onConstrainPair?.(first, selected, anchor);
  }

  function findMoveGroup(item) {
    const items = state.selectionItems;
    if (!handlers.onMoveSelection || !items || items.length < 2 || !item || !selected(item.type, item.id)) return null;
    if (item.type !== 'joint' && item.type !== 'element') return null;
    const ids = new Set();
    for (const entry of items) {
      if (entry.type === 'joint') ids.add(entry.id);
      if (entry.type === 'element') {
        const element = elements.get(entry.id);
        if (element) {
          ids.add(element.a);
          ids.add(element.b);
        }
      }
    }
    return ids.size && ![...ids].some((id) => joints.get(id)?.kind === 'origin') ? { items, ids } : null;
  }

  function dimensionDragOffset(dim, target) {
    const a = point(dim.a);
    const b = point(dim.b);
    if (!a || !b) return null;
    const shift = midpoint(a, b);
    if (dim.kind === 'horizontal') return target.y - shift.y;
    if (dim.kind === 'vertical') return target.x - shift.x;
    const heading = angle(a, b);
    return -(target.x - shift.x) * Math.sin(heading) + (target.y - shift.y) * Math.cos(heading);
  }

  function pointerDown(event) {
    if (gesture || (event.button !== 0 && event.button !== 1)) return;
    event.preventDefault();
    root.focus({ preventScroll: true });
    keyboardPoint = null;
    const p = eventPosition(event);
    const tool = activeTool();
    const pan = event.button === 1 || tool === 'pan';
    const pair = tool === 'select' && !state.readOnly && (event.ctrlKey || event.metaKey);
    const additive = tool === 'select' && event.shiftKey && !pair;
    const cursor = currentHover(p, event.shiftKey, event.pointerType === 'touch' ? 19 : 12);
    const item = hit(event.target) ?? (cursor.id != null ? { type: 'joint', id: cursor.id } : null);
    const joint = item?.type === 'joint' ? joints.get(item.id) : null;
    const mayDrag = !pan && !pair && !additive && tool === 'select';
    const canMove = joint && joint.kind !== 'origin' && (state.readOnly
      ? state.poseable && joint.kind === 'pin'
      : state.canMoveJoint?.(joint.id) !== false);
    const group = mayDrag && !state.readOnly ? findMoveGroup(item) : null;
    const dragJoint = mayDrag && !group && canMove ? joint.id : null;
    const press = dragJoint == null ? null : point(dragJoint);
    const dimension = mayDrag && !state.readOnly && handlers.onMoveDimension && item?.type === 'dimension' ? item.id : null;
    const dim = dimension == null ? null : state.model.dimensions?.find((d) => d.id === dimension);
    const offsetBias = dim ? (dim.offset ?? 0.12) - (dimensionDragOffset(dim, world(p)) ?? 0) : 0;
    gesture = {
      id: event.pointerId, start: p, view, pan, pair, selected: item, joint: dragJoint, press, simulated: !!state.readOnly,
      marquee: !pan && !pair && !item && tool === 'select', additive, moveItems: group?.items, movingJoints: group?.ids,
      dimension, offsetBias,
    };
    root.setPointerCapture?.(event.pointerId);
    hover = pan ? null : cursor;
    root.classList.toggle('is-panning', pan);
  }

  function pointerMove(event) {
    if (gesture && gesture.id !== event.pointerId) return;
    keyboardPoint = null;
    const p = eventPosition(event);
    handlers.onPointer?.(world(p));
    if (gesture?.pan) {
      const { view: initial, start } = gesture;
      setView({ ...initial, x: initial.x + p.x - start.x, y: initial.y + p.y - start.y });
      return;
    }
    const movedPointer = gesture && distance(p, gesture.start) > 3;
    if (gesture?.marquee && movedPointer) {
      gesture.end = p;
      drawPreview();
      return;
    }
    if (gesture?.movingJoints && (movedPointer || gesture.delta)) {
      const dx = (p.x - gesture.start.x) / view.scale;
      const dy = (gesture.start.y - p.y) / view.scale;
      const step = gridSize();
      gesture.delta = state.snap === false ? { x: dx, y: dy } : { x: Math.round(dx / step) * step, y: Math.round(dy / step) * step };
      paint();
      return;
    }
    if (gesture?.dimension != null && (movedPointer || gesture.dimensionOffset != null)) {
      const dim = state.model.dimensions.find((entry) => entry.id === gesture.dimension);
      const next = dim && dimensionDragOffset(dim, world(p));
      if (next != null) {
        gesture.dimensionOffset = next + gesture.offsetBias;
        paint();
      }
      return;
    }
    if (gesture?.joint != null && movedPointer) {
      const target = offset(gesture.press, (p.x - gesture.start.x) / view.scale, (gesture.start.y - p.y) / view.scale);
      const precision = gridSize();
      const moved = state.snap === false || gesture.simulated ? target : { x: Math.round(target.x / precision) * precision, y: Math.round(target.y / precision) * precision };
      gesture.point = moved;
      if (gesture.simulated) handlers.onPoseJoint?.(gesture.joint, moved, { commit: false });
      const displayed = gesture.simulated ? point(gesture.joint) : moved;
      hover = displayed ? { point: displayed } : null;
      paint();
      return;
    }
    hover = currentHover(p, event.shiftKey, event.pointerType === 'touch' ? 19 : 12);
    if (gesture?.point) {
      gesture.point = null;
      paint();
    } else {
      drawPreview();
    }
  }

  function pointerUp(event) {
    if (gesture?.id !== event.pointerId) return;
    const finished = gesture;
    gesture = null;
    if (root.hasPointerCapture?.(event.pointerId)) root.releasePointerCapture(event.pointerId);
    if (event.type !== 'pointercancel' && !finished.pan) {
      const end = eventPosition(event);
      if (finished.marquee && distance(finished.start, end) > 3) {
        const position = (id) => {
          const p = point(id);
          return p && screen(p);
        };
        const items = marqueeSelection(state.model, finished.start, end, position, { loads: showPointLoads() });
        constraintPair = null;
        handlers.onSelectMany?.(items, { additive: finished.additive });
      } else if (finished.movingJoints && finished.delta && (finished.delta.x || finished.delta.y)) {
        handlers.onMoveSelection?.(finished.moveItems, finished.delta);
      } else if (finished.dimension != null && finished.dimensionOffset != null) {
        handlers.onMoveDimension?.(finished.dimension, finished.dimensionOffset);
      } else if (finished.joint != null && finished.point) {
        handlers.onSelect?.({ type: 'joint', id: finished.joint });
        const update = finished.simulated ? handlers.onPoseJoint : handlers.onMoveJoint;
        update?.(finished.joint, finished.point, { commit: true });
      } else if (distance(end, finished.start) < 6) {
        const cursor = currentHover(end, event.shiftKey, event.pointerType === 'touch' ? 19 : 12);
        if (finished.pair && finished.selected) {
          selectPair(finished.selected, { x: event.clientX, y: event.clientY });
        } else if (finished.additive && finished.selected) {
          constraintPair = null;
          handlers.onSelectMany?.([finished.selected], { additive: true, toggle: true });
        } else if (finished.additive) {
          constraintPair = null;
        } else {
          constraintPair = null;
          useTool(finished.selected, cursor);
        }
      }
    }
    paint();
  }

  function wheel(event) {
    event.preventDefault();
    const delta = event.deltaY * (event.deltaMode === 1 ? 16 : event.deltaMode === 2 ? 200 : 1);
    zoom(Math.exp(-0.0015 * delta), eventPosition(event));
  }

  function keyDown(event) {
    if (event.key === 'Escape') {
      if (gesture && root.hasPointerCapture?.(gesture.id)) root.releasePointerCapture(gesture.id);
      gesture = null;
      draft = {};
      hover = null;
      keyboardPoint = null;
      constraintPair = null;
      paint();
      handlers.onCancel?.();
      return;
    }
    const selected = hit(event.target);
    if (event.altKey) return;
    if (event.ctrlKey || event.metaKey) {
      if (event.key === 'Enter' && selected && activeTool() === 'select' && !state.readOnly) {
        event.preventDefault();
        const bounds = event.target.getBoundingClientRect();
        selectPair(selected, { x: bounds.right, y: bounds.top + bounds.height / 2 });
        paint();
      }
      return;
    }
    if (event.key === 'Enter' || event.key === ' ') {
      event.preventDefault();
      if (event.shiftKey && selected && activeTool() === 'select') {
        handlers.onSelectMany?.([selected], { additive: true, toggle: true });
        paint();
        return;
      }
      const element = selected?.type === 'element' ? elements.get(selected.id) : null;
      const a = element && point(element.a);
      const b = element && point(element.b);
      const source = selected?.type === 'joint' ? point(selected.id) : a && b ? midpoint(a, b) : null;
      constraintPair = null;
      const cursor = currentHover(source ? screen(source) : keyboardPoint ?? viewport(), event.shiftKey);
      if (selected?.type === 'joint' && activeTool() === 'dimension') {
        cursor.id = selected.id;
        cursor.point = source;
      }
      useTool(selected, cursor);
      paint();
      return;
    }
    const directions = { ArrowLeft: [-1, 0], ArrowRight: [1, 0], ArrowUp: [0, -1], ArrowDown: [0, 1] };
    const direction = directions[event.key];
    if (!direction || selected) return;
    event.preventDefault();
    if (activeTool() === 'pan') {
      setView(offsetView(-direction[0] * 32, -direction[1] * 32));
      return;
    }
    const step = state.snap === false ? 12 : Math.max(1, gridSize() * view.scale);
    keyboardPoint = offset(keyboardPoint ?? viewport(), direction[0] * step, direction[1] * step);
    hover = currentHover(keyboardPoint, event.shiftKey);
    handlers.onPointer?.(hover.point);
    drawPreview();
  }

  function pointerLeave() {
    if (!gesture) {
      hover = null;
      drawPreview();
    }
  }

  const events = { pointerdown: pointerDown, pointermove: pointerMove, pointerup: pointerUp, pointercancel: pointerUp, pointerleave: pointerLeave, keydown: keyDown, wheel };
  for (const [event, listener] of Object.entries(events)) root.addEventListener(event, listener);
  const observer = new ResizeObserver(resize);
  observer.observe(host);

  return {
    render(next) {
      const previousTool = activeTool();
      const previousPose = state.readOnly;
      const previousSelection = state.selection;
      state = { ...state, ...next };
      joints = new Map((state.model.joints ?? []).map((item) => [item.id, item]));
      elements = new Map((state.model.elements ?? []).map((item) => [item.id, item]));
      if (activeTool() !== previousTool || state.readOnly !== previousPose) {
        draft = {};
        hover = null;
        constraintPair = null;
      } else if (previousSelection?.type !== state.selection?.type || previousSelection?.id !== state.selection?.id) {
        constraintPair = null;
      }
      resize();
      if (!fitted) fit();
      else paint();
    },
    fit,
    zoom,
    setView,
    getView: () => ({ ...view }),
    destroy() {
      observer.disconnect();
      root.remove();
    },
  };
}
