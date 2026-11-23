import { cloneMechanism, isFixed, isPiston, isRigidLink, normalizeMechanism } from "./mechanism-model.js";

export {
  cloneMechanism, createDemo, createMechanism, distance, isFixed, isPiston, nextId,
  normalizeMechanism, removeObject, sketchDriven, solveSketch, splitElement,
} from "./mechanism-model.js";

const EPSILON = 1e-9;
const GRAVITY = 9.81;

function axis(a, b) {
  const dx = b.x - a.x;
  const dy = b.y - a.y;
  const length = Math.hypot(dx, dy);
  return length > EPSILON ? { x: dx / length, y: dy / length, length } : { x: 1, y: 0, length };
}

function stroke(element, length) {
  const min = Math.max(0.001, element.minLength ?? length * 0.65);
  return [min, Math.max(min, element.maxLength ?? length)];
}

function clamp(value, min, max) {
  return Math.max(min, Math.min(max, value));
}

function indexJoints(model) {
  return new Map(model.joints.map(joint => [joint.id, joint]));
}

function massesOf(model) {
  const masses = new Map(model.joints.map(joint => [joint.id, Math.max(0.001, joint.mass ?? 0.05)]));
  for (const element of model.elements) {
    const half = Math.max(0, element.mass ?? 0) / 2;
    if (masses.has(element.a)) masses.set(element.a, masses.get(element.a) + half);
    if (masses.has(element.b)) masses.set(element.b, masses.get(element.b) + half);
  }
  return masses;
}

function poweredForce(element, direction, speed) {
  const supportedForce = Math.abs(element.force ?? 0);
  const resistance = (element.damping ?? 0) * speed;
  const pull = element.forceMode === "pull";
  if (element.kind === "actuator") return (pull ? -supportedForce : supportedForce) - resistance;
  const [min, max] = stroke(element, direction.length);
  const deflection = pull ? direction.length - min : max - direction.length;
  const springForce = supportedForce + (element.stiffness ?? 0) * Math.max(0, deflection);
  return pull ? Math.min(0, -springForce - resistance) : Math.max(0, springForce - resistance);
}

function axialForce(element, a, b, direction) {
  if (element.kind === "motor") return (element.torque ?? 0) / Math.max(direction.length, 0.02);
  if (!isPiston(element)) return element.axialForce ?? 0;
  const speed = ((b.vx ?? 0) - (a.vx ?? 0)) * direction.x + ((b.vy ?? 0) - (a.vy ?? 0)) * direction.y;
  return poweredForce(element, direction, speed);
}

export function effectiveForce(model, id) {
  const element = model.elements.find(item => item.id === id);
  if (!element) return 0;
  const joints = indexJoints(model);
  const a = joints.get(element.a);
  const b = joints.get(element.b);
  return a && b ? axialForce(element, a, b, axis(a, b)) : 0;
}

function neighborsOf(elements) {
  const neighbors = new Map();
  for (const { a, b } of elements) {
    if (!neighbors.has(a)) neighbors.set(a, []);
    if (!neighbors.has(b)) neighbors.set(b, []);
    neighbors.get(a).push(b);
    neighbors.get(b).push(a);
  }
  return neighbors;
}

function connectedFrom(starts, neighbors, canContinue = () => true) {
  const found = new Set(starts);
  const queue = [...found];
  for (const id of queue) {
    if (!canContinue(id)) continue;
    for (const neighbor of neighbors.get(id) ?? []) {
      if (found.has(neighbor)) continue;
      found.add(neighbor);
      queue.push(neighbor);
    }
  }
  return found;
}

function loadsOn(model, masses, joints, includePistons = false, omitId) {
  const gravity = model.options?.gravityEnabled === false ? 0 : model.options?.gravity ?? GRAVITY;
  const forces = new Map(model.joints.map(joint => [joint.id, { x: 0, y: -gravity * masses.get(joint.id) }]));
  const appliedAxial = new Map();
  for (const load of model.loads) {
    const force = forces.get(load.joint);
    if (force && load.enabled !== false) {
      force.x += load.fx ?? 0;
      force.y += load.fy ?? 0;
    }
  }
  for (const element of model.elements) {
    if (element.id === omitId || (element.kind !== "motor" && !(includePistons && isPiston(element)))) continue;
    const a = joints.get(element.a);
    const b = joints.get(element.b);
    if (!a || !b) continue;
    const direction = axis(a, b);
    const force = axialForce(element, a, b, direction);
    appliedAxial.set(element.id, force);
    const x = (element.kind === "motor" ? -direction.y : direction.x) * force;
    const y = (element.kind === "motor" ? direction.x : direction.y) * force;
    forces.get(a.id).x -= x;
    forces.get(a.id).y -= y;
    forces.get(b.id).x += x;
    forces.get(b.id).y += y;
  }
  return { forces, appliedAxial };
}

function torqueAt(joint, pivot, x, y) {
  return (joint.x - pivot.x) * y - (joint.y - pivot.y) * x;
}

function sizing(model, id) {
  const selected = model.elements.find(element => element.id === id && isPiston(element));
  if (!selected) return null;
  const joints = indexJoints(model);
  const a = joints.get(selected.a);
  const b = joints.get(selected.b);
  if (!a || !b || a.kind === "origin" || b.kind === "origin") return null;
  const moving = [a, b].filter(joint => !isFixed(joint)).map(joint => joint.id);
  const links = model.elements.filter(link => isRigidLink(link)
    && joints.get(link.a)?.kind !== "origin" && joints.get(link.b)?.kind !== "origin");
  const connected = connectedFrom(moving, neighborsOf(links), key => joints.has(key) && !isFixed(joints.get(key)));
  const candidates = [...connected].map(key => joints.get(key)).filter(joint => joint && isFixed(joint));
  const direction = axis(a, b);
  if (!candidates.length || direction.length <= EPSILON) return null;

  let pivot = null;
  let momentArm = 0;
  for (const candidate of candidates) {
    const armA = isFixed(a) ? 0 : torqueAt(a, candidate, -direction.x, -direction.y);
    const armB = isFixed(b) ? 0 : torqueAt(b, candidate, direction.x, direction.y);
    if (Math.abs(armA + armB) > Math.abs(momentArm)) {
      momentArm = armA + armB;
      pivot = candidate;
    }
  }
  if (!pivot || Math.abs(momentArm) < EPSILON) return null;
  const { forces } = loadsOn(model, massesOf(model), joints, true, id);
  const movingJoints = [...connected].map(key => joints.get(key)).filter(joint => joint && !isFixed(joint));
  const moment = movingJoints.reduce((sum, joint) => {
    const force = forces.get(joint.id);
    return sum + torqueAt(joint, pivot, force.x, force.y);
  }, 0);
  const force = -moment / momentArm;
  return {
    balance: { force, pivotId: pivot.id, moment, momentArm, mode: force < 0 ? "pull" : "push" },
    selected, pivot, movingJoints, a, b, direction,
  };
}

function legacyBalanceForce(model, id) {
  return sizing(model, id)?.balance ?? null;
}

function legacyAssistForce(model, id) {
  const result = sizing(model, id);
  if (!result) return null;
  const { balance, selected, pivot, movingJoints, a, b, direction } = result;
  let handle = null;
  let handleArm = 0;
  for (const joint of movingJoints) {
    const arm = joint.x - pivot.x;
    if (Math.abs(arm) > Math.abs(handleArm)) {
      handleArm = arm;
      handle = joint;
    }
  }
  if (!handle || Math.abs(handleArm) < 1e-6) return null;
  const configuredForce = axialForce(selected, a, b, direction);
  const residualMoment = balance.moment + balance.momentArm * configuredForce;
  return {
    handForce: -residualMoment / handleArm, handleId: handle.id, residualMoment, handleArm,
    configuredForce, requiredForce: balance.force, pivotId: balance.pivotId,
  };
}

function physicalModel(source) {
  const model = normalizeMechanism(source);
  const joints = model.joints.filter(joint => joint.kind !== "origin");
  const rank = joint => joint.kind === "rotation" ? 2 : joint.kind === "fixed" ? 1 : 0;
  const anchors = joints.filter(joint => isFixed(joint)).sort((a, b) => rank(b) - rank(a));
  const aliases = new Map(joints.map(joint => [joint.id, joint.id]));
  for (const anchor of anchors) {
    if (aliases.get(anchor.id) !== anchor.id) continue;
    for (const other of joints) {
      if (rank(other) >= rank(anchor) || aliases.get(other.id) !== other.id) continue;
      if (Math.hypot(anchor.x - other.x, anchor.y - other.y) > 1e-5) continue;
      aliases.set(other.id, anchor.id);
      if (!anchor.name) anchor.name = other.name;
    }
  }
  model.joints = joints.filter(joint => aliases.get(joint.id) === joint.id);
  model.elements = model.elements
    .filter(element => aliases.has(element.a) && aliases.has(element.b))
    .map(element => ({ ...element, a: aliases.get(element.a), b: aliases.get(element.b) }))
    .filter(element => element.a !== element.b);
  model.loads = model.loads
    .filter(load => aliases.has(load.joint))
    .map(load => ({ ...load, joint: aliases.get(load.joint) }));
  model.dimensions = model.dimensions
    .filter(dimension => aliases.has(dimension.a) && aliases.has(dimension.b))
    .map(dimension => ({ ...dimension, a: aliases.get(dimension.a), b: aliases.get(dimension.b) }))
    .filter(dimension => dimension.a !== dimension.b);
  model.constraints = [];
  return model;
}

function staticAxialForces(model, joints) {
  const jointRows = new Map(model.joints.map((joint, index) => [joint.id, index * 2]));
  const members = model.elements.filter(element => element.kind !== "motor");
  if (!members.length) return new Map();

  // Fixed pin reactions can absorb any force applied to ground. The free-joint
  // equilibrium equations are enough to estimate tension in the links.
  const columns = members.map(element => {
    const a = joints.get(element.a);
    const b = joints.get(element.b);
    const direction = axis(a, b);
    const column = Array(model.joints.length * 2).fill(0);
    if (!isFixed(a)) {
      column[jointRows.get(a.id)] = direction.x;
      column[jointRows.get(a.id) + 1] = direction.y;
    }
    if (!isFixed(b)) {
      column[jointRows.get(b.id)] = -direction.x;
      column[jointRows.get(b.id) + 1] = -direction.y;
    }
    return column;
  });
  const { forces } = loadsOn(model, massesOf(model), joints);
  const rhs = model.joints.flatMap(joint => {
    const force = forces.get(joint.id);
    return isFixed(joint) ? [0, 0] : [-force.x, -force.y];
  });
  const dot = (first, second) => first.reduce((sum, value, index) => sum + value * second[index], 0);
  const system = columns.map((column, row) => [
    ...columns.map((other, index) => dot(column, other) + (index === row ? 1e-9 : 0)),
    dot(column, rhs),
  ]);
  for (let col = 0; col < columns.length; col++) {
    let pivot = col;
    for (let row = col + 1; row < columns.length; row++) {
      if (Math.abs(system[row][col]) > Math.abs(system[pivot][col])) pivot = row;
    }
    [system[col], system[pivot]] = [system[pivot], system[col]];
    const divisor = system[col][col];
    if (Math.abs(divisor) < 1e-12) return new Map();
    for (let index = col; index <= columns.length; index++) system[col][index] /= divisor;
    for (let row = 0; row < columns.length; row++) {
      if (row === col) continue;
      const factor = system[row][col];
      for (let index = col; index <= columns.length; index++) system[row][index] -= factor * system[col][index];
    }
  }
  return new Map(members.map((element, index) => [element.id, system[index].at(-1)]));
}

function trackRange(ranges, id, value) {
  if (!Number.isFinite(value)) return;
  if (Object.hasOwn(ranges, id)) {
    ranges[id][0] = Math.min(ranges[id][0], value);
    ranges[id][1] = Math.max(ranges[id][1], value);
  } else {
    Object.defineProperty(ranges, id, { value: [value, value], enumerable: true, configurable: true });
  }
}

export function analyzeLegacyPath(source, samples = 73) {
  const model = physicalModel(source);
  const result = { method: "legacy-pivot-estimate", pivotId: null, forceRanges: {}, lengthRanges: {}, requiredLengthRanges: {}, assistForceRanges: {} };
  const joints = indexJoints(model);
  const fixed = model.joints.filter(isFixed);
  const neighbors = neighborsOf(model.elements.filter(isRigidLink));
  const candidates = fixed.filter(joint => (neighbors.get(joint.id) ?? []).some(id => !isFixed(joints.get(id))));
  const scores = new Map(candidates.map(joint => [joint.id, 0]));
  for (const element of model.elements) {
    const weight = Math.max(element.mass ?? 0, 0.1);
    for (const id of [element.a, element.b]) if (scores.has(id)) scores.set(id, scores.get(id) + weight);
  }
  const pivot = candidates.sort((a, b) => scores.get(b.id) - scores.get(a.id))[0];
  if (!pivot) return result;
  result.pivotId = pivot.id;

  const component = connectedFrom(neighbors.get(pivot.id) ?? [], neighbors, id => !isFixed(joints.get(id)));
  const moving = [...component].map(id => joints.get(id)).filter(joint => !isFixed(joint));
  const initial = moving.map(joint => [joint.x - pivot.x, joint.y - pivot.y]);
  const pistons = model.elements.filter(isPiston);
  const count = clamp(Math.trunc(samples) || 73, 1, 801);

  // Sweep around the main hinge, keeping the physical stroke and the raw
  // mounting-point travel separate. This estimate assumes a single pivot.
  for (let step = 0; step < count; step++) {
    const angle = count === 1 ? 0 : (-80 + 160 * step / (count - 1)) * Math.PI / 180;
    const sin = Math.sin(angle);
    const cos = Math.cos(angle);
    moving.forEach((joint, index) => {
      const [x, y] = initial[index];
      joint.x = pivot.x + x * cos - y * sin;
      joint.y = pivot.y + x * sin + y * cos;
    });
    let withinStroke = true;
    for (const element of pistons) {
      const { length } = axis(joints.get(element.a), joints.get(element.b));
      const [min, max] = stroke(element, length);
      trackRange(result.requiredLengthRanges, element.id, length);
      trackRange(result.lengthRanges, element.id, clamp(length, min, max));
      if (length < min - 1e-6 || length > max + 1e-6) withinStroke = false;
    }
    if (!withinStroke) continue;
    const forces = staticAxialForces(model, joints);
    for (const element of model.elements) {
      if (!isPiston(element)) {
        const force = forces.get(element.id);
        if (Math.abs(force) <= 1_000_000) trackRange(result.forceRanges, element.id, force);
        continue;
      }
      const balance = legacyBalanceForce(model, element.id);
      if (balance && Math.abs(balance.momentArm) >= 0.01 && Math.abs(balance.force) <= 1_000_000) {
        trackRange(result.forceRanges, element.id, balance.force);
      }
      const assist = legacyAssistForce(model, element.id);
      if (assist && Math.abs(assist.handForce) <= 1_000_000) trackRange(result.assistForceRanges, element.id, assist.handForce);
    }
  }
  return result;
}

function vectorDot(a, b) {
  return a.reduce((sum, value, index) => sum + value * b[index], 0);
}

function reduceRows(rows, width, tolerance = 1e-8) {
  const matrix = rows.map(row => [...row]);
  const columns = [];
  for (let column = 0; column < width && columns.length < matrix.length; column++) {
    const row = columns.length;
    let pivot = row;
    for (let index = row + 1; index < matrix.length; index++) {
      if (Math.abs(matrix[index][column]) > Math.abs(matrix[pivot][column])) pivot = index;
    }
    if (Math.abs(matrix[pivot][column]) <= tolerance) continue;
    [matrix[row], matrix[pivot]] = [matrix[pivot], matrix[row]];
    const divisor = matrix[row][column];
    for (let index = column; index < matrix[row].length; index++) matrix[row][index] /= divisor;
    for (let index = 0; index < matrix.length; index++) {
      if (index === row || matrix[index][column] === 0) continue;
      const factor = matrix[index][column];
      for (let entry = column; entry < matrix[index].length; entry++) {
        matrix[index][entry] -= factor * matrix[row][entry];
      }
    }
    columns.push(column);
  }
  return { matrix, columns };
}

function rigidMembers(model, joints) {
  const links = model.elements.filter(isRigidLink).map(element => ({
    element, a: joints.get(element.a), b: joints.get(element.b), length: element.length,
  })).filter(({ a, b }) => a && b && a.kind !== "origin" && b.kind !== "origin");
  return [...links, ...rigidBodyPairs(model, joints)];
}

function constraintsMatrix(members, offsets) {
  return members.map(({ a, b }) => {
    const row = Array(offsets.size * 2).fill(0);
    const direction = axis(a, b);
    for (const [joint, sign] of [[a, -1], [b, 1]]) {
      const offset = offsets.get(joint.id);
      if (offset === undefined) continue;
      row[offset] = sign * direction.x;
      row[offset + 1] = sign * direction.y;
    }
    return row;
  });
}

function equilibrium(members, points, forces, checkRank = true) {
  const offsets = new Map(points.filter(joint => !isFixed(joint)).map((joint, index) => [joint.id, index * 2]));
  const columns = constraintsMatrix(members, offsets);
  const rhs = [...offsets.keys()].flatMap(id => [-forces.get(id).x, -forces.get(id).y]);
  const system = columns.map((column, index) => [
    ...columns.map((other, j) => vectorDot(column, other) + (index === j ? 1e-12 : 0)),
    vectorDot(column, rhs),
  ]);
  const { matrix, columns: pivots } = reduceRows(system, columns.length, 1e-14);
  const values = Array(columns.length).fill(0);
  for (const [index, column] of pivots.entries()) values[column] = matrix[index].at(-1);
  const residuals = Array(rhs.length).fill(0);
  for (const [index, column] of columns.entries()) {
    column.forEach((component, j) => { residuals[j] += component * values[index]; });
  }
  let residual = 0;
  let scale = 1;
  for (let index = 0; index < rhs.length; index += 2) {
    residual = Math.max(residual, Math.hypot(residuals[index] - rhs[index], residuals[index + 1] - rhs[index + 1]));
    scale = Math.max(scale, Math.hypot(rhs[index], rhs[index + 1]));
  }
  return {
    values, residual, balanced: residual <= 0.01 + 1e-5 * scale,
    indeterminate: checkRank && reduceRows(columns, rhs.length).columns.length < members.length,
  };
}

export function currentForces(model, { dynamic = false } = {}) {
  const joints = indexJoints(model);
  const masses = massesOf(model);
  const physical = model.joints.filter(joint => joint.kind !== "origin");
  const bodyConnections = neighborsOf(model.elements.filter(element => element.kind === "body"));
  const bodyEstimate = element => element?.kind === "body"
    && ((bodyConnections.get(element.a)?.length ?? 0) > 1 || (bodyConnections.get(element.b)?.length ?? 0) > 1);
  const byJoint = Object.create(null);
  const byElement = Object.create(null);
  const arrows = [];
  for (const joint of physical) {
    byJoint[joint.id] = { applied: { x: 0, y: 0 }, member: { x: 0, y: 0 }, support: null, net: { x: 0, y: 0 } };
  }
  function add(joint, fx, fy, kind, details) {
    const result = byJoint[joint];
    if (!result) return;
    if (Math.abs(fx) > 1e-9 || Math.abs(fy) > 1e-9 || details?.torque) arrows.push({ joint, fx, fy, kind, ...details });
    const target = kind === "reaction" ? (result.support ??= { x: 0, y: 0 })
      : kind === "member" || kind === "stroke" ? result.member : result.applied;
    target.x += fx;
    target.y += fy;
    result.net.x += fx;
    result.net.y += fy;
  }
  function endpointForces(a, b, force, kind, element, tangent = false, metadata) {
    const direction = axis(a, b);
    const fx = (tangent ? -direction.y : direction.x) * force;
    const fy = (tangent ? direction.x : direction.y) * force;
    const details = element ? { element: element.id, ...metadata } : { ...metadata };
    const motor = tangent ? { ...details, torque: element.torque ?? 0, x: a.x, y: a.y } : details;
    add(a.id, -fx, -fy, kind, motor);
    add(b.id, fx, fy, kind, details);
  }
  const gravity = model.options?.gravityEnabled === false ? 0 : model.options?.gravity ?? GRAVITY;
  for (const joint of physical) add(joint.id, 0, -gravity * masses.get(joint.id), "gravity");
  for (const load of model.loads) {
    if (load.enabled !== false) add(load.joint, load.fx ?? 0, load.fy ?? 0, "load", { load: load.id });
  }
  for (const element of model.elements) {
    const a = joints.get(element.a);
    const b = joints.get(element.b);
    if (!a || !b || a.kind === "origin" || b.kind === "origin") continue;
    const direction = axis(a, b);
    const powered = element.kind === "motor" || isPiston(element);
    const force = dynamic ? element.axialForce ?? 0 : powered ? axialForce(element, a, b, direction) : null;
    const limitForce = dynamic && isPiston(element) ? element.limitForce ?? 0 : null;
    byElement[element.id] = { axialForce: force, limitForce, estimated: bodyEstimate(element) };
    if (powered) endpointForces(a, b, force, element.kind === "motor" ? "motor" : "piston", element, element.kind === "motor");
    else if (force != null) endpointForces(a, b, force, "member", element, false, { estimated: bodyEstimate(element) });
    if (dynamic && element.kind === "motor" && element.memberAxialForce) {
      byElement[element.id].memberAxialForce = element.memberAxialForce;
      endpointForces(a, b, element.memberAxialForce, "member", element);
    }
    if (limitForce) endpointForces(a, b, limitForce, "stroke", element);
  }
  if (dynamic) {
    for (const joint of physical) add(joint.id, joint.internalX ?? 0, joint.internalY ?? 0, "member", { constraint: true, estimated: true });
    for (const joint of physical.filter(isFixed)) add(joint.id, joint.reactionX ?? 0, joint.reactionY ?? 0, "reaction");
    // The integrator also tracks forces at internal rigid-body constraints.
    for (const joint of physical.filter(joint => !isFixed(joint))) {
      const { applied, member } = byJoint[joint.id];
      member.x = joint.reactionX ?? member.x;
      member.y = joint.reactionY ?? member.y;
      byJoint[joint.id].net = { x: applied.x + member.x, y: applied.y + member.y };
    }
    return { arrows, joints: byJoint, elements: byElement, balanceResidual: null, status: "dynamic" };
  }
  const members = rigidMembers(model, joints);
  const geometryViolation = members.some(({ a, b, length }) => Math.abs(axis(a, b).length - length) > 1e-6);
  let strokeViolation = false;
  const stops = model.elements.filter(isPiston).flatMap(element => {
    const a = joints.get(element.a);
    const b = joints.get(element.b);
    if (!a || !b || a.kind === "origin" || b.kind === "origin") return [];
    const { length } = axis(a, b);
    const [min, max] = stroke(element, length);
    strokeViolation ||= length < min - 1e-6 || length > max + 1e-6;
    if (Math.abs(max - min) <= 1e-6 && Math.abs(length - min) <= 1e-6) return [{ a, b, element, side: "locked" }];
    if (Math.abs(length - min) <= 1e-6) return [{ a, b, element, side: "min" }];
    if (Math.abs(length - max) <= 1e-6) return [{ a, b, element, side: "max" }];
    return [];
  });
  const applied = new Map(physical.map(joint => [joint.id, byJoint[joint.id].applied]));
  let active = [...members, ...stops];
  let solution = equilibrium(active, physical, applied);
  let valid = true;
  while (valid) {
    const filtered = active.filter((member, index) => member.side === undefined || member.side === "locked"
      || (member.side === "min" ? solution.values[index] >= -1e-6 : solution.values[index] <= 1e-6));
    if (filtered.length === active.length) break;
    active = filtered;
    solution = equilibrium(active, physical, applied);
    valid = solution.balanced;
  }
  if (solution.balanced && !strokeViolation && !geometryViolation) {
    for (const [index, { a, b, element, side }] of active.entries()) {
      const force = solution.values[index];
      if (element) {
        const property = side ? "limitForce" : element.kind === "motor" ? "memberAxialForce" : "axialForce";
        byElement[element.id][property] = force;
        byElement[element.id].estimated ||= solution.indeterminate;
      }
      endpointForces(a, b, force, side ? "stroke" : "member", element, false, {
        estimated: solution.indeterminate || !element || bodyEstimate(element), constraint: !element,
      });
    }
    for (const joint of physical.filter(isFixed)) {
      const { x, y } = byJoint[joint.id].net;
      add(joint.id, -x, -y, "reaction", { estimated: solution.indeterminate });
    }
  }
  return {
    arrows, joints: byJoint, elements: byElement, balanceResidual: solution.residual, strokeViolation, geometryViolation,
    indeterminate: solution.balanced && solution.indeterminate && !strokeViolation && !geometryViolation,
    status: solution.balanced && !strokeViolation && !geometryViolation ? "balanced" : "unbalanced",
  };
}

function drivenGroups(model, joints) {
  const members = rigidMembers(model, joints);
  const neighbors = neighborsOf(members.map(({ a, b }) => ({ a: a.id, b: b.id })));
  const visited = new Set();
  const groups = [];
  for (const joint of model.joints.filter(joint => !isFixed(joint))) {
    if (visited.has(joint.id)) continue;
    const all = connectedFrom([joint.id], neighbors, id => !isFixed(joints.get(id)));
    const moving = [...all].map(id => joints.get(id)).filter(item => !isFixed(item));
    moving.forEach(item => visited.add(item.id));
    const group = { all, moving, members: [], drivers: [] };
    const movingIds = new Set(moving.map(item => item.id));
    for (const member of members) {
      if (!movingIds.has(member.a.id) && !movingIds.has(member.b.id)) continue;
      group.members.push(member);
      if (!member.element || isFixed(member.a) === isFixed(member.b)) continue;
      const pivot = isFixed(member.a) ? member.a : member.b;
      const driven = isFixed(member.a) ? member.b : member.a;
      const element = member.element;
      group.drivers.push({
        id: element.id, pivotId: pivot.id, jointId: driven.id,
        label: `${element.name || `Member ${element.id}`} · ${pivot.name || `J${pivot.id}`}`,
      });
    }
    const originals = group.members.filter(member => member.element);
    const anchors = [...all].map(id => joints.get(id)).filter(isFixed);
    if (anchors.length === 1 && (originals.length === 1 || originals.every(member => member.element.kind === "body"))) {
      group.hinge = anchors[0];
      group.rest = moving.map(item => ({ joint: item, x: item.x - group.hinge.x, y: item.y - group.hinge.y }));
    }
    if (group.drivers.length) groups.push(group);
  }
  return groups;
}

export function pathDrivers(source) {
  const model = physicalModel(source);
  return drivenGroups(model, indexJoints(model)).flatMap(group => group.drivers);
}

function drivenTangent(group, offsets, pivot, driven) {
  if (group.hinge) return {
    mobility: 1,
    velocity: group.moving.flatMap(joint => [pivot.y - joint.y, joint.x - pivot.x]),
  };
  const width = offsets.size * 2;
  const { matrix, columns } = reduceRows(constraintsMatrix(group.members, offsets), width, 1e-6);
  const mobility = width - columns.length;
  if (mobility !== 1) return { mobility, velocity: null };
  const free = Array.from({ length: width }, (_, index) => index).find(index => !columns.includes(index));
  const velocity = Array(width).fill(0);
  velocity[free] = 1;
  for (const [index, column] of columns.entries()) velocity[column] = -matrix[index][free];
  const index = offsets.get(driven.id);
  const dx = driven.x - pivot.x;
  const dy = driven.y - pivot.y;
  const rate = (dx * velocity[index + 1] - dy * velocity[index]) / (dx * dx + dy * dy);
  if (!Number.isFinite(rate) || Math.abs(rate) < 1e-7) return { mobility, velocity: null };
  return { mobility, velocity: velocity.map(component => component / rate) };
}

function tangentWork(model, joints, masses, moving, offsets, velocity) {
  const pointVelocity = id => {
    const offset = offsets.get(id);
    return offset == null || !velocity ? [0, 0] : [velocity[offset], velocity[offset + 1]];
  };
  const { forces } = loadsOn(model, masses, joints);
  const externalWork = velocity ? moving.reduce((sum, joint) => {
    const [vx, vy] = pointVelocity(joint.id);
    const force = forces.get(joint.id);
    return sum + force.x * vx + force.y * vy;
  }, 0) : null;
  const handle = { id: null, arm: 0 };
  for (const joint of moving) {
    const [, arm] = pointVelocity(joint.id);
    if (Math.abs(arm) <= Math.abs(handle.arm)) continue;
    handle.id = joint.id;
    handle.arm = arm;
  }
  const pistons = new Map();
  let poweredWork = 0;
  for (const element of model.elements.filter(isPiston)) {
    if (!offsets.has(element.a) && !offsets.has(element.b)) continue;
    const a = joints.get(element.a);
    const b = joints.get(element.b);
    if (!a || !b || a.kind === "origin" || b.kind === "origin") continue;
    if (![a, b].every(joint => offsets.has(joint.id) || isFixed(joint))) continue;
    const direction = axis(a, b);
    const av = pointVelocity(a.id);
    const bv = pointVelocity(b.id);
    const arm = (bv[0] - av[0]) * direction.x + (bv[1] - av[1]) * direction.y;
    const configuredForce = poweredForce(element, direction, 0);
    poweredWork += configuredForce * arm;
    pistons.set(element.id, { arm, configuredForce, direction });
  }
  return {
    externalWork, residualWork: externalWork == null ? null : externalWork + poweredWork,
    handle, pistons, forces, pointVelocity,
  };
}

function currentHolding(model, id) {
  const joints = indexJoints(model);
  const selected = model.elements.find(element => element.id === id && isPiston(element));
  if (!selected) return null;
  const a = joints.get(selected.a);
  const b = joints.get(selected.b);
  if (!a || !b || a.kind === "origin" || b.kind === "origin") return null;
  const group = drivenGroups(model, joints).find(item => item.all.has(a.id) && item.all.has(b.id)
    || item.moving.some(joint => joint.id === a.id || joint.id === b.id) && (isFixed(a) || isFixed(b)));
  if (!group) return null;
  if (group.members.some(({ a: first, b: second, length }) => Math.abs(axis(first, second).length - length) > 1e-6)) return null;
  const offsets = new Map(group.moving.map((joint, index) => [joint.id, index * 2]));
  if (model.elements.some(element => isPiston(element) && [element.a, element.b].some(key => offsets.has(key))
    && ![element.a, element.b].every(key => offsets.has(key) || (joints.has(key) && isFixed(joints.get(key)))))) return null;
  const driver = group.drivers[0];
  const pivot = joints.get(driver.pivotId);
  const driven = joints.get(driver.jointId);
  const { velocity } = drivenTangent(group, offsets, pivot, driven);
  if (!velocity) return null;
  const work = tangentWork(model, joints, massesOf(model), group.moving, offsets, velocity);
  const piston = work.pistons.get(id);
  if (!piston || Math.abs(piston.arm) < 0.005) return null;
  const lineOfAction = group.moving.every(joint => {
    const [vx, vy] = work.pointVelocity(joint.id);
    return Math.hypot(vx + joint.y - pivot.y, vy - joint.x + pivot.x) < 1e-6;
  });
  const force = -work.externalWork / piston.arm;
  const balance = {
    force, pivotId: pivot.id, moment: work.externalWork, momentArm: piston.arm, mode: force < 0 ? "pull" : "push",
    lineOfAction, basis: "single-piston",
  };
  const assist = Math.abs(work.handle.arm) < 0.005 ? null : {
    handForce: -work.residualWork / work.handle.arm, handleId: work.handle.id, residualMoment: work.residualWork,
    handleArm: work.handle.arm, configuredForce: piston.configuredForce, requiredForce: force, pivotId: pivot.id,
  };
  return { balance, assist };
}

export function balanceForce(model, id) {
  return currentHolding(model, id)?.balance ?? null;
}

export function assistForce(model, id) {
  return currentHolding(model, id)?.assist ?? null;
}

function projectDrive(group, pivot, driven, base, length, angle) {
  if (group.hinge) {
    const cos = Math.cos(angle);
    const sin = Math.sin(angle);
    for (const { joint, x, y } of group.rest) {
      joint.x = pivot.x + x * cos - y * sin;
      joint.y = pivot.y + x * sin + y * cos;
    }
    return true;
  }
  driven.x = pivot.x + Math.cos(base + angle) * length;
  driven.y = pivot.y + Math.sin(base + angle) * length;
  for (let index = 0; index < 250; index++) {
    let residual = 0;
    for (const member of group.members) {
      const { a, b } = member;
      const direction = axis(a, b);
      const error = direction.length - member.length;
      residual = Math.max(residual, Math.abs(error));
      const wa = isFixed(a) || a === driven ? 0 : 1;
      const wb = isFixed(b) || b === driven ? 0 : 1;
      if (wa + wb === 0) continue;
      const correction = error / (wa + wb);
      a.x += direction.x * correction * wa;
      a.y += direction.y * correction * wa;
      b.x -= direction.x * correction * wb;
      b.y -= direction.y * correction * wb;
    }
    if (residual <= 1e-8) return true;
  }
  return group.members.every(({ a, b, length: rest }) => Math.abs(axis(a, b).length - rest) <= 1e-6);
}

function putPeak(result, id, force, length, angleDeg) {
  const previous = result.recommendations[id];
  if (previous && Math.abs(previous.signedForce) >= Math.abs(force)) return;
  result.recommendations[id] = { signedForce: force, mode: force < 0 ? "pull" : "push", peakLength: length, peakAngleDeg: angleDeg };
}

export function analyzePath(source, options = {}) {
  if (typeof options === "number") options = { samples: options };
  const model = physicalModel(source);
  const joints = indexJoints(model);
  const masses = massesOf(model);
  const groups = drivenGroups(model, joints);
  const drivers = groups.flatMap(group => group.drivers);
  const result = {
    method: "constrained", status: "unavailable", message: "", drivers, pivotId: null, driverId: null, poses: [],
    forceRanges: {}, lengthRanges: {}, requiredLengthRanges: {}, requiredForceRanges: {}, assistForceRanges: {}, recommendations: {},
  };
  const unavailable = (message, status = "unavailable") => ({ ...result, message, status });
  if (!drivers.length) return unavailable("Connect a rigid member to a physical hinge before sweeping the mechanism.");
  if (options.driverId == null && groups.length > 1) return unavailable("Select a drive member to choose which linkage to sweep.");
  const driver = options.driverId == null ? drivers[0] : drivers.find(item => String(item.id) === String(options.driverId));
  if (!driver) return unavailable("The selected drive member does not connect a moving joint to a fixed pivot.");
  const group = groups.find(item => item.drivers.includes(driver));
  const pivot = joints.get(driver.pivotId);
  const driven = joints.get(driver.jointId);
  const { length } = axis(pivot, driven);
  result.pivotId = pivot.id;
  result.driverId = driver.id;
  const offsets = new Map(group.moving.map((joint, index) => [joint.id, index * 2]));
  const initial = drivenTangent(group, offsets, pivot, driven);
  if (initial.mobility > 1) return unavailable(`This linkage has ${initial.mobility} free motions. Add a constraint or rigid connection to define one path.`);
  if (initial.mobility < 1) return unavailable("The selected linkage has no free motion.");
  if (!initial.velocity || length < 1e-6) return unavailable("The selected drive is at a singular pose.");
  const lower = clamp(Number(options.minAngle ?? -80), -360, 360);
  const upper = clamp(Number(options.maxAngle ?? 80), -360, 360);
  if (!Number.isFinite(lower) || !Number.isFinite(upper) || lower > upper) return unavailable("Enter a valid sweep range.");
  const count = lower === upper ? 1 : clamp(Math.trunc(options.samples) || 73, 1, 401);
  const radians = degrees => degrees * Math.PI / 180;
  const degrees = Array.from({ length: count }, (_, index) => count === 1 ? clamp(0, lower, upper) : lower + (upper - lower) * index / (count - 1));
  if (lower < 0 && upper > 0 && !degrees.some(value => Math.abs(value) < 1e-8)) degrees.push(0);
  const base = Math.atan2(driven.y - pivot.y, driven.x - pivot.x);
  const pistons = model.elements.filter(isPiston).filter(element => {
    const inGroup = id => offsets.has(id) || isFixed(joints.get(id));
    return (offsets.has(element.a) || offsets.has(element.b)) && inGroup(element.a) && inGroup(element.b);
  });
  const coupled = model.elements.filter(isPiston).some(element => !pistons.includes(element)
    && (offsets.has(element.a) || offsets.has(element.b)));
  const initialPositions = group.moving.map(joint => ({ x: joint.x, y: joint.y }));
  const positions = () => Object.fromEntries(model.joints.map(joint => [joint.id, { x: joint.x, y: joint.y }]));
  const pose = (angleDeg, geometry, reachable) => {
    const output = {
      angle: radians(angleDeg), angleDeg, positions: geometry ? positions() : null, feasible: false,
      status: geometry ? "ok" : "geometry", pistons: Object.create(null), memberForces: Object.create(null),
    };
    if (!geometry) return output;
    const { velocity } = drivenTangent(group, offsets, pivot, driven);
    const work = tangentWork(model, joints, masses, group.moving, offsets, velocity);
    let withinStroke = true;
    for (const element of pistons) {
      const { arm, configuredForce, direction } = work.pistons.get(element.id);
      const [min, max] = stroke(element, direction.length);
      const within = direction.length >= min - 1e-6 && direction.length <= max + 1e-6;
      withinStroke &&= within;
      const singular = !velocity || Math.abs(arm) < 0.005;
      const force = singular || coupled ? null : -work.externalWork / arm;
      const assist = !velocity || coupled || Math.abs(work.handle.arm) < 0.005 ? null : -work.residualWork / work.handle.arm;
      output.pistons[element.id] = {
        length: direction.length, force, assistForce: assist, handleId: work.handle.id, configuredForce, momentArm: arm,
        status: !within ? "out_of_stroke" : coupled ? "unavailable" : singular ? "singular" : "ok",
      };
      trackRange(result.requiredLengthRanges, element.id, direction.length);
      if (force != null) {
        trackRange(result.requiredForceRanges, element.id, force);
        putPeak(result, element.id, force, direction.length, angleDeg);
      }
    }
    output.withinStroke = withinStroke;
    output.feasible = withinStroke && reachable && !coupled;
    if (!output.feasible) {
      output.status = coupled ? "coupled" : withinStroke ? "unreachable" : "stroke";
      return output;
    }
    if (pistons.length === 0 || (pistons.length === 1 && output.pistons[pistons[0].id].status === "ok")) {
      const supportMembers = [...group.members, ...pistons.map(element => ({
        a: joints.get(element.a), b: joints.get(element.b), element,
      }))];
      const equilibriumResult = equilibrium(supportMembers, group.moving, work.forces);
      if (equilibriumResult.balanced && !equilibriumResult.indeterminate) {
        for (const [index, { element }] of supportMembers.entries()) {
          if (!element || isPiston(element) || element.kind === "motor") continue;
          const force = equilibriumResult.values[index];
          output.memberForces[element.id] = force;
          trackRange(result.forceRanges, element.id, force);
        }
      }
    }
    for (const [id, item] of Object.entries(output.pistons)) {
      trackRange(result.lengthRanges, id, item.length);
      if (item.force == null) continue;
      trackRange(result.forceRanges, id, item.force);
      trackRange(result.assistForceRanges, id, item.assistForce);
    }
    return output;
  };

  const startWithinStroke = pistons.every(element => {
    const { length: current } = axis(joints.get(element.a), joints.get(element.b));
    const [min, max] = stroke(element, current);
    return current >= min - 1e-6 && current <= max + 1e-6;
  });
  function sweep(values) {
    group.moving.forEach((joint, index) => Object.assign(joint, initialPositions[index]));
    let solved = true;
    let reachable = startWithinStroke;
    let previous = 0;
    for (const angle of values) {
      const subdivisions = Math.max(1, Math.ceil(Math.abs(angle - previous) / 2));
      for (let index = 1; index <= subdivisions && solved; index++) {
        const intermediate = previous + (angle - previous) * index / subdivisions;
        solved = projectDrive(group, pivot, driven, base, length, radians(intermediate));
        if (index !== subdivisions && solved) {
          reachable &&= pistons.every(element => {
            const { length: value } = axis(joints.get(element.a), joints.get(element.b));
            const [min, max] = stroke(element, value);
            return value >= min - 1e-6 && value <= max + 1e-6;
          });
        }
      }
      const row = pose(angle, solved, reachable);
      reachable &&= row.feasible;
      previous = angle;
      result.poses.push(row);
    }
  }
  sweep(degrees.filter(angle => angle >= 0).sort((a, b) => a - b));
  sweep(degrees.filter(angle => angle < 0).sort((a, b) => b - a));
  result.poses.sort((a, b) => a.angle - b.angle);
  for (const [id, [min, max]] of Object.entries(result.requiredLengthRanges)) {
    const recommendation = result.recommendations[id];
    const forceRange = result.requiredForceRanges[id];
    const arms = result.poses.flatMap(row => Number.isFinite(row.pistons[id]?.momentArm) ? [row.pistons[id].momentArm] : []);
    const singular = result.poses.some(row => row.pistons[id]?.force == null)
      || (arms.some(arm => arm < 0) && arms.some(arm => arm > 0));
    result.recommendations[id] = {
      ...recommendation, minLength: min, maxLength: max, stroke: max - min,
      mode: forceRange?.[0] < 0 && forceRange?.[1] > 0 ? "both" : recommendation?.mode,
      status: recommendation ? singular ? "partial" : "ok" : "unavailable",
    };
  }
  if (coupled) return unavailable("A piston connects two independently moving linkages. Select a model with one coupled degree of freedom.", "partial");
  if (!result.poses.some(row => row.feasible)) {
    return unavailable(startWithinStroke ? "No reachable poses in this sweep." : "The drawn pose lies outside a configured piston stroke.", "partial");
  }
  result.status = result.poses.every(row => row.feasible && Object.values(row.pistons).every(item => item.status === "ok")) ? "ok" : "partial";
  if (result.status === "partial") result.message = "The sweep includes unreachable, stroke-limited, or singular poses.";
  return result;
}

function rigidBodyPairs(model, joints) {
  const bodies = model.elements.filter(element => element.kind === "body");
  const neighbors = neighborsOf(bodies);
  const visited = new Set();
  const pairs = [];
  for (const start of neighbors.keys()) {
    if (visited.has(start)) continue;
    const group = [...connectedFrom([start], neighbors)];
    group.forEach(id => visited.add(id));
    for (const [index, first] of group.entries()) {
      for (const second of group.slice(index + 1)) {
        if (neighbors.get(first).includes(second)) continue;
        const a = joints.get(first);
        const b = joints.get(second);
        pairs.push({ a, b, length: axis(a, b).length });
      }
    }
  }
  return pairs;
}

export class Simulation {
  constructor(source) {
    this.initialModel = physicalModel(source);
    this.tracing = false;
    this.reset();
  }

  reset() {
    this.model = cloneMechanism(this.initialModel);
    this.joints = indexJoints(this.model);
    this.masses = massesOf(this.model);
    this.inverseMasses = new Map(this.model.joints.map(joint => [joint.id, isFixed(joint) ? 0 : 1 / this.masses.get(joint.id)]));
    this.members = this.model.elements.map(element => ({
      element, a: this.joints.get(element.a), b: this.joints.get(element.b), length: element.length,
    }));
    this.pistons = this.members.filter(({ element }) => isPiston(element));
    this.constraints = [
      ...this.members.filter(({ element }) => isRigidLink(element)),
      ...rigidBodyPairs(this.model, this.joints),
    ];
    this.hinge = this.findHinge();
    this.time = 0;
    this.traces = new Map();
    this.settle();
    this.refreshForces();
    this.recordTraces();
    return this.model;
  }

  findHinge() {
    const free = this.model.joints.filter(joint => !isFixed(joint));
    const bodyLinks = this.members.filter(({ element }) => element.kind === "body");
    const links = this.members.filter(({ element }) => isRigidLink(element));
    const fixed = this.model.joints.filter(joint => isFixed(joint) && links.some(({ a, b }) => a === joint || b === joint));
    if (fixed.length !== 1 || !free.length || (bodyLinks.length !== links.length && links.length !== 1)) return null;
    const pivot = fixed[0];
    const connected = connectedFrom([pivot.id], neighborsOf(links.map(({ a, b }) => ({ a: a.id, b: b.id }))));
    if (!free.every(joint => connected.has(joint.id))) return null;
    const rest = free.map(joint => ({ joint, x: joint.x - pivot.x, y: joint.y - pivot.y }));
    const inertia = rest.reduce((sum, { joint, x, y }) => sum + this.masses.get(joint.id) * (x * x + y * y), 0);
    return inertia > 1e-8 ? { angle: 0, speed: 0, pivot, rest, inertia } : null;
  }

  placeHinge(angle) {
    const { pivot, rest } = this.hinge;
    const cos = Math.cos(angle);
    const sin = Math.sin(angle);
    for (const { joint, x, y } of rest) {
      joint.x = pivot.x + x * cos - y * sin;
      joint.y = pivot.y + x * sin + y * cos;
    }
  }

  hingeCandidate(angle) {
    this.placeHinge(angle);
    for (const piston of this.pistons) {
      const { a, b, element } = piston;
      const { length } = axis(a, b);
      const [min, max] = stroke(element, length);
      if (length < min - 1e-8 || length > max + 1e-8) return piston;
    }
    return null;
  }

  moveHinge(target) {
    const start = this.hinge.angle;
    const steps = Math.max(1, Math.ceil(Math.abs(target - start) / 0.025));
    let last = start;
    for (let index = 1; index <= steps; index++) {
      const candidate = start + (target - start) * index / steps;
      const stop = this.hingeCandidate(candidate);
      if (!stop) {
        last = candidate;
        continue;
      }
      let outside = candidate;
      for (let iteration = 0; iteration < 32; iteration++) {
        const midpoint = (last + outside) / 2;
        if (this.hingeCandidate(midpoint)) outside = midpoint;
        else last = midpoint;
      }
      this.placeHinge(last);
      this.hinge.angle = last;
      return stop;
    }
    this.hinge.angle = target;
    return null;
  }

  integrateHinge(dt) {
    const { pivot, rest, inertia } = this.hinge;
    const torque = rest.reduce((sum, { joint }) => sum + torqueAt(joint, pivot, joint.fx, joint.fy), 0);
    const speed = (this.hinge.speed + torque / inertia * dt) * (1 - this.model.options.damping) ** (dt * 60);
    const stop = this.moveHinge(this.hinge.angle + clamp(speed * dt, -0.2, 0.2));
    const nextSpeed = stop ? 0 : speed;
    const acceleration = (nextSpeed - this.hinge.speed) / dt;
    this.hinge.speed = nextSpeed;
    let stopped = null;
    if (stop) {
      const { a, b, element } = stop;
      const direction = axis(a, b);
      const arm = (isFixed(b) ? 0 : torqueAt(b, pivot, direction.x, direction.y))
        - (isFixed(a) ? 0 : torqueAt(a, pivot, direction.x, direction.y));
      if (Math.abs(arm) > 1e-7) {
        element.limitForce = (inertia * acceleration - torque) / arm;
        stopped = { ...stop, direction, force: element.limitForce };
        this.addReaction(a, b, direction, element.limitForce);
      }
    }
    const residual = new Map();
    for (const { joint } of rest) {
      const x = joint.x - pivot.x;
      const y = joint.y - pivot.y;
      const mass = this.masses.get(joint.id);
      const fx = mass * (-x * nextSpeed * nextSpeed - y * acceleration);
      const fy = mass * (-y * nextSpeed * nextSpeed + x * acceleration);
      residual.set(joint.id, { x: joint.fx - fx, y: joint.fy - fy });
      if (stopped) {
        const sign = joint === stopped.a ? -1 : joint === stopped.b ? 1 : 0;
        residual.get(joint.id).x += sign * stopped.direction.x * stopped.force;
        residual.get(joint.id).y += sign * stopped.direction.y * stopped.force;
      }
      joint.vx = -y * nextSpeed;
      joint.vy = x * nextSpeed;
    }
    const solution = equilibrium(this.constraints, rest.map(({ joint }) => joint), residual, false);
    for (const [index, { element, a, b }] of this.constraints.entries()) {
      const force = solution.values[index];
      if (element?.kind === "motor") element.memberAxialForce = force;
      else if (element) element.axialForce = force;
      this.addReaction(a, b, axis(a, b), force, !element);
    }
  }

  setTracing(enabled) {
    this.tracing = Boolean(enabled);
    this.recordTraces();
  }

  clearTraces() {
    this.traces.clear();
  }

  recordTraces() {
    if (!this.tracing) return;
    for (const { id, x, y, kind } of this.model.joints) {
      if (kind !== "pin") continue;
      if (!this.traces.has(id)) this.traces.set(id, []);
      const points = this.traces.get(id);
      const previous = points.at(-1);
      if (!previous || Math.hypot(x - previous.x, y - previous.y) >= 0.002) points.push({ x, y });
      if (points.length > 1500) points.shift();
    }
  }

  poseJoint(id, x, y) {
    const joint = this.joints.get(id);
    if (!joint || isFixed(joint) || !Number.isFinite(x) || !Number.isFinite(y)) return { maxConstraintError: this.maxConstraintError() };
    if (this.hinge) {
      const { pivot, angle } = this.hinge;
      const rest = this.hinge.rest.find(item => item.joint === joint);
      const goal = Math.atan2(y - pivot.y, x - pivot.x) - Math.atan2(rest.y, rest.x);
      const delta = Math.atan2(Math.sin(goal - angle), Math.cos(goal - angle));
      this.moveHinge(angle + delta);
      this.settle();
      this.refreshForces();
      this.recordTraces();
      return { maxConstraintError: this.maxConstraintError() };
    }
    const startX = joint.x;
    const startY = joint.y;
    const steps = clamp(Math.ceil(Math.hypot(x - startX, y - startY) / 0.05), 1, 60);
    for (let step = 1; step <= steps; step++) {
      joint.x = startX + (x - startX) * step / steps;
      joint.y = startY + (y - startY) * step / steps;
      this.solvePositions(24, 1 / 60, joint.id);
    }
    const result = this.settle();
    this.refreshForces();
    this.recordTraces();
    return result;
  }

  settle(iterations = Math.max(72, this.model.options.iterations)) {
    if (!this.hinge || this.maxConstraintError() > 1e-6) {
      this.hinge = null;
      this.solvePositions(iterations, 1 / 60);
    }
    if (this.hinge) this.hinge.speed = 0;
    for (const joint of this.model.joints) {
      joint.vx = 0;
      joint.vy = 0;
    }
    return { maxConstraintError: this.maxConstraintError() };
  }

  refreshForces() {
    const { forces, appliedAxial } = loadsOn(this.model, this.masses, this.joints, true);
    for (const joint of this.model.joints) {
      const force = forces.get(joint.id);
      joint.fx = force.x;
      joint.fy = force.y;
      joint.reactionX = isFixed(joint) ? -force.x : 0;
      joint.reactionY = isFixed(joint) ? -force.y : 0;
      joint.internalX = 0;
      joint.internalY = 0;
    }
    for (const { element } of this.members) {
      element.axialForce = appliedAxial.get(element.id) ?? 0;
      element.limitForce = 0;
      if (element.kind === "motor") element.memberAxialForce = 0;
    }
  }

  step(dt = 1 / 60) {
    if (!(dt > 0) || !Number.isFinite(dt)) return { maxConstraintError: this.maxConstraintError() };
    const elapsed = Math.min(dt, 0.1);
    const steps = Math.ceil(elapsed * 120);
    for (let index = 0; index < steps; index++) this.integrate(elapsed / steps);
    this.time += elapsed;
    this.recordTraces();
    return { maxConstraintError: this.maxConstraintError() };
  }

  integrate(dt) {
    for (const { element, a, b } of this.pistons) {
      if (element.autoReturn === false) continue;
      const direction = axis(a, b);
      const [min, max] = stroke(element, direction.length);
      const tolerance = Math.min(1e-6, (max - min) * 0.01);
      if (max <= min + EPSILON) continue;
      if (element.forceMode !== "pull" && direction.length >= max - tolerance) element.forceMode = "pull";
      else if (element.forceMode === "pull" && direction.length <= min + tolerance) element.forceMode = "push";
    }
    this.refreshForces();
    if (this.hinge) {
      this.integrateHinge(dt);
      return;
    }
    const previous = new Map();
    const retention = (1 - this.model.options.damping) ** (dt * 60);
    for (const joint of this.model.joints) {
      if (isFixed(joint)) continue;
      previous.set(joint.id, { x: joint.x, y: joint.y });
      const inverseMass = this.inverseMasses.get(joint.id);
      joint.vx = (joint.vx + joint.fx * dt * inverseMass) * retention;
      joint.vy = (joint.vy + joint.fy * dt * inverseMass) * retention;
    }
    this.clampPistonVelocities(dt);
    for (const joint of this.model.joints) {
      if (isFixed(joint)) continue;
      joint.x += joint.vx * dt;
      joint.y += joint.vy * dt;
    }
    this.solvePositions(this.model.options.iterations, dt, undefined, true);
    for (const joint of this.model.joints) {
      const start = previous.get(joint.id);
      if (!start) continue;
      joint.vx = (joint.x - start.x) / dt;
      joint.vy = (joint.y - start.y) / dt;
    }
    this.clampPistonVelocities(dt, true);
  }

  clampPistonVelocities(dt, atStop = false) {
    for (const { element, a, b } of this.pistons) {
      const wa = this.inverseMasses.get(a.id);
      const wb = this.inverseMasses.get(b.id);
      if (wa + wb === 0) continue;
      const direction = axis(a, b);
      const [min, max] = stroke(element, direction.length);
      const speed = (b.vx - a.vx) * direction.x + (b.vy - a.vy) * direction.y;
      const lower = atStop && direction.length > min + 1e-7 ? -Infinity : (min - direction.length) / dt;
      const upper = atStop && direction.length < max - 1e-7 ? Infinity : (max - direction.length) / dt;
      const maxSpeed = atStop ? Infinity : Math.max((max - min) * 0.08, 0.002) / dt;
      const target = clamp(clamp(speed, -maxSpeed, maxSpeed), lower, upper);
      const impulse = (target - speed) / (wa + wb);
      a.vx -= direction.x * impulse * wa;
      a.vy -= direction.y * impulse * wa;
      b.vx += direction.x * impulse * wb;
      b.vy += direction.y * impulse * wb;
      if (speed < lower || speed > upper) {
        const force = (clamp(speed, lower, upper) - speed) / ((wa + wb) * dt);
        element.limitForce += force;
        this.addReaction(a, b, direction, force);
      }
    }
  }

  addReaction(a, b, direction, force, internal = false) {
    a.reactionX += (isFixed(a) ? 1 : -1) * direction.x * force;
    a.reactionY += (isFixed(a) ? 1 : -1) * direction.y * force;
    b.reactionX += (isFixed(b) ? -1 : 1) * direction.x * force;
    b.reactionY += (isFixed(b) ? -1 : 1) * direction.y * force;
    if (internal) {
      a.internalX -= direction.x * force;
      a.internalY -= direction.y * force;
      b.internalX += direction.x * force;
      b.internalY += direction.y * force;
    }
  }

  solvePositions(iterations, dt, preferredId, readForces = false) {
    const project = ({ element, a, b, length }, limited = false) => {
      const direction = axis(a, b);
      const target = limited ? clamp(direction.length, ...stroke(element, direction.length)) : length;
      const error = direction.length - target;
      if (Math.abs(error) <= EPSILON) return Math.abs(error);
      const wa = this.inverseMasses.get(a.id) * (a.id === preferredId ? 0.02 : 1);
      const wb = this.inverseMasses.get(b.id) * (b.id === preferredId ? 0.02 : 1);
      if (wa + wb === 0) return Math.abs(error);
      const correction = error / (wa + wb);
      a.x += direction.x * correction * wa;
      a.y += direction.y * correction * wa;
      b.x -= direction.x * correction * wb;
      b.y -= direction.y * correction * wb;
      if (readForces) {
        const force = -correction / (dt * dt);
        if (limited) element.limitForce += force;
        else if (element?.kind === "motor") element.memberAxialForce += force;
        else if (element) element.axialForce += force;
        this.addReaction(a, b, direction, force, !element);
      }
      return Math.abs(error);
    };
    for (let iteration = 0; iteration < iterations; iteration++) {
      let worst = 0;
      for (const member of this.constraints) worst = Math.max(worst, project(member));
      for (const piston of this.pistons) worst = Math.max(worst, project(piston, true));
      if (worst <= 1e-8) break;
    }
  }

  maxConstraintError() {
    let worst = 0;
    for (const { a, b, length } of this.constraints) worst = Math.max(worst, Math.abs(axis(a, b).length - length));
    for (const { element, a, b } of this.pistons) {
      const { length } = axis(a, b);
      worst = Math.max(worst, Math.abs(length - clamp(length, ...stroke(element, length))));
    }
    return worst;
  }
}
