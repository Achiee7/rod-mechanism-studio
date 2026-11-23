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

function partName(kind) {
  return PART_NAMES[kind] ?? kind?.replaceAll('_', ' ') ?? 'Part';
}

const decimal = new Intl.NumberFormat('nl-NL', { maximumFractionDigits: 1 });
const meterDecimal = new Intl.NumberFormat('nl-NL', { maximumFractionDigits: 3 });

function shownValue(value, unit) {
  return (unit === 'm' ? meterDecimal : decimal).format(value) + ' ' + unit;
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
  description.textContent = 'Use Tab to focus existing geometry and Enter to select it. Use the arrow keys to move the keyboard cursor and Enter to use the active tool. Hold Shift to place an aligned dimension. Use Control-click or Control-Enter to add a second item to a constraint selection.';

  const defs = svg('defs', {}, root);
  const patterns = ['minor', 'major'].map((name) => {
    const pattern = svg('pattern', { id: id + name, patternUnits: 'userSpaceOnUse' }, defs);
    const path = svg('path', { class: 'drawing-grid-' + name }, pattern);
    const rect = svg('rect', { width: '100%', height: '100%', fill: 'url(#' + id + name + ')' }, root);
    return { pattern, path, rect };
  });
  const geometry = svg('g', { class: 'drawing-geometry' }, root);
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

  const world = (p) => ({ x: (p.x - view.x) / view.scale, y: (view.y - p.y) / view.scale });
  const screen = (p) => ({ x: view.x + p.x * view.scale, y: view.y - p.y * view.scale });
  const activeTool = () => state.readOnly && state.tool !== 'pan' ? 'select' : state.tool;
  const gridSize = () => Number.isFinite(state.gridSize) && state.gridSize > 0 ? state.gridSize : 0.1;

  function point(ref) {
    if (typeof ref === 'object') return ref;
    if (gesture?.joint === ref && gesture.point && !gesture.simulated) return gesture.point;
    const positions = state.positions;
    return (positions instanceof Map ? positions.get(ref) : positions?.[ref]) ?? joints.get(ref);
  }

  function group(type, item, extraClass = '') {
    const selected = state.selection?.type === type && state.selection.id === item.id;
    const paired = constraintPair?.some((selection) => selection.type === type && selection.id === item.id);
    const title = item.name || item.label || (partName(item.kind ?? type) + ' ' + item.id);
    const node = svg('g', {
      class: ('drawing-' + type + ' ' + extraClass + (selected ? ' selected' : '') + (paired ? ' paired' : '')).trim(),
      'data-type': type,
      'data-id': item.id,
      role: 'button',
      tabindex: '0',
      'aria-label': title,
      'aria-pressed': selected,
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
    if (state.selection?.type === 'joint' && state.selection.id === joint.id) {
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

  function drawPart(element, parent, a, b) {
    const length = distance(a, b);
    if (!length) return;
    const degrees = angle(a, b) * 180 / Math.PI;
    const g = svg('g', { transform: 'translate(' + a.x + ' ' + a.y + ') rotate(' + degrees + ')' }, parent);
    const stroke = (from, to, className) => line(g, { x: from, y: 0 }, { x: to, y: 0 }, { class: className });
    stroke(0, length, 'drawing-hit');
    if (element.kind === 'body') {
      stroke(0, length, 'body-edge');
      stroke(0, length, 'body-fill');
      const middle = { x: length / 2, y: 0 };
      if (length > 34) circle(g, middle, 3, { class: 'body-mass' });
    } else if (element.kind === 'actuator' || element.kind === 'gas_spring') {
      stroke(0, length, 'piston-rod');
      const start = Math.min(10, length * 0.15);
      const cap = length * 0.63;
      svg('rect', { x: start, y: -5.5, width: cap - start, height: 11, rx: 2, class: 'piston-cylinder' }, g);
      line(g, { x: cap - 5, y: -4 }, { x: cap - 5, y: 4 }, { class: 'piston-seal' });
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
    drawPart(element, g, p, q);
    if (state.showIds) label(g, offset(midpoint(p, q), 0, -12), element.name || element.id, { class: 'drawing-label', 'text-anchor': 'middle' });
  }

  function arrow(parent, a, b, className) {
    const heading = angle(a, b);
    const side = (delta) => offset(b, -9 * Math.cos(heading + delta), -9 * Math.sin(heading + delta));
    const left = side(0.5);
    const right = side(-0.5);
    line(parent, a, b, { class: className });
    svg('polygon', { points: b.x + ',' + b.y + ' ' + left.x + ',' + left.y + ' ' + right.x + ',' + right.y, class: className + '-head' }, parent);
  }

  function drawLoad(load) {
    const p = point(load.joint);
    if (!p) return;
    const g = group('load', load);
    if (load.enabled === false) g.classList.add('load-disabled');
    const s = screen(p);
    const magnitude = Math.hypot(load.fx, load.fy);
    if (magnitude < 1e-9 && (state.selection?.type !== 'load' || state.selection.id !== load.id)) {
      svg('circle', { cx: s.x + 13, cy: s.y - 13, r: 3, class: 'force-placeholder' }, g);
      return;
    }
    const nx = magnitude ? load.fx / magnitude : 0;
    const ny = magnitude ? -load.fy / magnitude : 1;
    const a = offset(s, -nx * 55, -ny * 55);
    const b = offset(s, -nx * 7, -ny * 7);
    line(g, a, b, { class: 'drawing-hit' });
    arrow(g, a, b, 'force-arrow');
    label(g, offset(a, nx > 0.45 ? -6 : 8, ny > 0.4 ? -5 : 12), shownValue(magnitude, 'N'), { class: 'force-label', 'text-anchor': nx > 0.45 ? 'end' : 'start' });
  }

  function drawDimension(parent, dim) {
    const a = point(dim.a);
    const b = point(dim.b);
    if (!a || !b) return;
    const [start, end] = dimensionPoints(a, b, dim.kind, dim.offset ?? 0.12).map(screen);
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

  function drawTraces() {
    const paths = state.traces instanceof Map ? state.traces.values() : Object.values(state.traces ?? {});
    for (const trace of paths) {
      if (!trace || trace.length < 2) continue;
      const points = trace.map((p) => screen(Array.isArray(p) ? { x: p[0], y: p[1] } : p));
      svg('polyline', { points: points.map((p) => p.x + ',' + p.y).join(' '), class: 'joint-trace', 'pointer-events': 'none' }, geometry);
    }
  }

  function drawMomentArm() {
    const arm = state.momentArm;
    if (!arm?.pivot || !arm?.foot) return;
    const a = screen(arm.pivot);
    const b = screen(arm.foot);
    const g = svg('g', { class: 'drawing-moment-arm' }, overlay);
    if (arm.linePoint) line(g, screen(arm.linePoint), b, { class: 'moment-extension' });
    line(g, a, b, { class: 'moment-arm' });
    const theta = angle(b, a);
    svg('path', { d: 'M0 7H7V0', transform: 'translate(' + b.x + ' ' + b.y + ') rotate(' + (theta * 180 / Math.PI - 90) + ')', class: 'moment-corner' }, g);
    label(g, offset(midpoint(a, b), 7, -5), shownValue(distance(arm.pivot, arm.foot) * 1000, 'mm'), { class: 'moment-label' });
  }

  function drawForces() {
    for (const force of state.forces ?? []) {
      const origin = force.point ?? point(force.joint);
      const magnitude = Math.hypot(force.fx, force.fy);
      if (!origin || !Number.isFinite(magnitude) || magnitude < 1e-8) continue;
      const a = screen(origin);
      const length = Math.min(82, 26 + 11 * Math.log10(magnitude + 1));
      const b = offset(a, force.fx / magnitude * length, -force.fy / magnitude * length);
      arrow(overlay, a, b, 'result-force');
      label(overlay, offset(b, 7, -4), shownValue(magnitude, 'N'), { class: 'result-force-label' });
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
    if (state.showLoads !== false) drawForces();
    drawMomentArm();
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
    root.setAttribute('class', 'mechanism-drawing tool-' + activeTool() + (gesture?.pan ? ' is-panning' : '') + (gesture?.joint != null ? ' is-dragging' : ''));
    updateGrid();
    geometry.replaceChildren();
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
    if (state.showLoads !== false) for (const load of state.model.loads ?? []) drawLoad(load);
    for (const joint of joints.values()) drawJoint(joint);
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

  function pointerDown(event) {
    if (gesture || (event.button !== 0 && event.button !== 1)) return;
    event.preventDefault();
    root.focus({ preventScroll: true });
    keyboardPoint = null;
    const p = eventPosition(event);
    const tool = activeTool();
    const pan = event.button === 1 || tool === 'pan';
    const pair = tool === 'select' && !state.readOnly && (event.ctrlKey || event.metaKey);
    const selected = hit(event.target);
    const cursor = currentHover(p, event.shiftKey, event.pointerType === 'touch' ? 19 : 12);
    const joint = selected?.type === 'joint' ? joints.get(selected.id) : selected ? null : joints.get(cursor.id);
    const canMove = joint && joint.kind !== 'origin' && (state.readOnly
      ? state.poseable && joint.kind === 'pin'
      : state.canMoveJoint?.(joint.id) !== false);
    const dragJoint = !pan && !pair && tool === 'select' && canMove ? joint.id : null;
    const press = dragJoint == null ? null : point(dragJoint);
    gesture = { id: event.pointerId, start: p, view, pan, pair, selected, joint: dragJoint, press, simulated: !!state.readOnly };
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
    if (gesture?.joint != null && distance(p, gesture.start) > 3) {
      const target = offset(gesture.press, (p.x - gesture.start.x) / view.scale, (gesture.start.y - p.y) / view.scale);
      const precision = gridSize();
      const moved = state.snap === false || gesture.simulated ? target : { x: Math.round(target.x / precision) * precision, y: Math.round(target.y / precision) * precision };
      gesture.point = moved;
      if (gesture.simulated) handlers.onPoseJoint?.(gesture.joint, moved, { commit: false });
      hover = { point: moved };
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
      if (finished.joint != null && finished.point) {
        handlers.onSelect?.({ type: 'joint', id: finished.joint });
        const update = finished.simulated ? handlers.onPoseJoint : handlers.onMoveJoint;
        update?.(finished.joint, finished.point, { commit: true });
      } else if (distance(eventPosition(event), finished.start) < 6) {
        const cursor = currentHover(eventPosition(event), event.shiftKey, event.pointerType === 'touch' ? 19 : 12);
        if (finished.pair && finished.selected) {
          selectPair(finished.selected, { x: event.clientX, y: event.clientY });
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
