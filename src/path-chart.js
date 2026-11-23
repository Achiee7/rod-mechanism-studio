const NS = 'http://www.w3.org/2000/svg';
const LEFT = 49;
const RIGHT = 383;
const TOP = 10;
const BOTTOM = 113;
const STATUS = 128;

const format = (n, digits = 1) => n.toLocaleString('nl-NL', {maximumFractionDigits: digits});

function svgNode(name, attributes, content) {
  const node = document.createElementNS(NS, name);
  for (const [key, value] of Object.entries(attributes)) node.setAttribute(key, value);
  if (content != null) node.textContent = content;
  return node;
}

function plottedSeries(poses, metric, pistonId, memberId) {
  const piston = pose => pose.pistons?.[pistonId];
  const length = pose => Number.isFinite(piston(pose)?.length) ? 1_000 * piston(pose).length : null;
  switch (metric) {
    case 'length': return [
      {name: 'Raw connection length', tone: 'reference', value: length},
      {name: 'Within stroke and reachable', tone: 'primary', value: pose => pose.feasible ? length(pose) : null},
    ];
    case 'hand': return [
      {name: 'Vertical hand effort (+ up)', tone: 'primary', value: pose => pose.feasible ? piston(pose)?.assistForce : null},
    ];
    case 'member': return [
      {name: 'Axial force (+ compression)', tone: 'primary', value: pose => pose.feasible ? pose.memberForces?.[memberId] : null},
    ];
    default: return [
      {name: 'Required hold in reachable poses (+ push)', tone: 'primary', value: pose => pose.feasible ? piston(pose)?.force : null},
      {name: 'Installed piston force', tone: 'installed', value: pose => pose.feasible ? piston(pose)?.configuredForce : null},
      ...(poses.some(pose => !pose.feasible && Number.isFinite(piston(pose)?.force))
        ? [{name: 'Hold demand outside installed travel', tone: 'reference', value: pose => piston(pose)?.force}]
        : []),
    ];
  }
}

function coordinateRange(values, includeZero = false) {
  const min = Math.min(...values, ...(includeZero ? [0] : []));
  const max = Math.max(...values, ...(includeZero ? [0] : []));
  const span = max - min;
  if (span > 0) return [min, max];
  const pad = Math.max(Math.abs(min) * 0.1, 1);
  return [min - pad, max + pad];
}

function line(points, x, y) {
  let penDown = false;
  const commands = [];
  for (const {angle, value} of points) {
    if (!Number.isFinite(value)) {
      penDown = false;
      continue;
    }
    commands.push((penDown ? 'L' : 'M') + x(angle).toFixed(1) + ' ' + y(value).toFixed(1));
    penDown = true;
  }
  return commands.join(' ');
}

export function pathChart(poses, {metric, pistonId, memberId, sampleIndex}, selectSample) {
  const figure = document.createElement('figure');
  figure.className = 'rm-path-figure';
  const series = plottedSeries(poses, metric, pistonId, memberId).map(set => ({
    ...set, points: poses.map(pose => ({angle: pose.angleDeg, value: set.value(pose)})),
  }));
  const values = series.flatMap(set => set.points.map(point => point.value)).filter(Number.isFinite);
  if (!values.length) {
    const empty = document.createElement('figcaption');
    empty.className = 'rm-note rm-chart-empty';
    empty.textContent = 'No values are available for this quantity over the sampled path.';
    figure.append(empty);
    return figure;
  }

  const units = metric === 'length' ? 'mm' : 'N';
  const [firstAngle, lastAngle] = coordinateRange(poses.map(pose => pose.angleDeg));
  const [min, max] = coordinateRange(values, units === 'N');
  const x = angle => LEFT + (angle - firstAngle) * (RIGHT - LEFT) / (lastAngle - firstAngle);
  const y = value => BOTTOM - (value - min) * (BOTTOM - TOP) / (max - min);
  const chart = svgNode('svg', {viewBox: '0 0 400 154', role: 'img', 'aria-label': 'Sampled path diagram. Values in ' + units + '. Use the pose sample slider to inspect exact values.'});
  const ticks = 4;
  for (let step = 0; step <= ticks; step++) {
    const value = max - (max - min) * step / ticks;
    const pos = y(value);
    chart.append(
      svgNode('line', {x1: LEFT, x2: RIGHT, y1: pos, y2: pos, class: 'rm-chart-grid'}),
      svgNode('text', {x: LEFT - 5, y: pos + 3, 'text-anchor': 'end'}, Math.abs(value) >= 100000 ? value.toExponential(1) : format(value, 1)),
    );
  }
  if (min < 0 && max > 0) chart.append(svgNode('line', {x1: LEFT, x2: RIGHT, y1: y(0), y2: y(0), class: 'rm-chart-zero'}));
  for (const angle of [firstAngle, (firstAngle + lastAngle) / 2, lastAngle]) {
    chart.append(svgNode('text', {x: x(angle), y: 146, 'text-anchor': 'middle'}, format(angle, 1) + '°'));
  }
  chart.append(svgNode('text', {x: LEFT - 5, y: BOTTOM + 17, 'text-anchor': 'end'}, units));
  for (const set of [...series].reverse()) {
    chart.append(svgNode('path', {d: line(set.points, x, y), class: 'rm-chart-series ' + set.tone}));
    if (set.points.filter(point => Number.isFinite(point.value)).length === 1) {
      const point = set.points.find(point => Number.isFinite(point.value));
      chart.append(svgNode('circle', {cx: x(point.angle), cy: y(point.value), r: 2.8, class: 'rm-chart-marker ' + set.tone}));
    }
  }
  for (const pose of poses) {
    chart.append(svgNode('circle', {cx: x(pose.angleDeg), cy: STATUS, r: 1.8, class: 'rm-chart-pose ' + (pose.feasible ? 'feasible' : 'blocked')}));
  }

  const selected = poses[sampleIndex];
  if (selected) {
    const markerX = x(selected.angleDeg);
    chart.append(svgNode('line', {x1: markerX, x2: markerX, y1: TOP, y2: STATUS + 2, class: 'rm-chart-cursor'}));
    for (const set of series) {
      const value = set.value(selected);
      if (Number.isFinite(value)) chart.append(svgNode('circle', {cx: markerX, cy: y(value), r: 3.8, class: 'rm-chart-selection ' + set.tone}));
    }
  }
  figure.append(chart);
  if (selectSample) {
    chart.classList.add('rm-chart-clickable');
    chart.addEventListener('click',event => {
      const matrix = chart.getScreenCTM();
      if (!matrix) return;
      const position = new DOMPoint(event.clientX,event.clientY).matrixTransform(matrix.inverse());
      const angle = firstAngle + (position.x - LEFT) * (lastAngle - firstAngle) / (RIGHT - LEFT);
      const nearest = poses.reduce((best,pose,index) => Math.abs(pose.angleDeg-angle) < Math.abs(poses[best].angleDeg-angle) ? index : best,0);
      selectSample(nearest);
    });
  }
  const caption = document.createElement('figcaption');
  for (const set of series.filter(entry => entry.points.some(point => Number.isFinite(point.value)))) {
    const key = document.createElement('span');
    key.className = 'rm-chart-key ' + set.tone;
    key.textContent = set.name;
    caption.append(key);
  }
  if (poses.some(pose => !pose.feasible)) {
    const key = document.createElement('span');
    key.className = 'rm-chart-key blocked';
    key.textContent = 'Unavailable pose';
    caption.append(key);
  }
  figure.append(caption);
  return figure;
}
