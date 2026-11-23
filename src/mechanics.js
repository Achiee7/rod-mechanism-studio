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

export function balanceForce(model, id) {
  return sizing(model, id)?.balance ?? null;
}

export function assistForce(model, id) {
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

export function analyzePath(source, samples = 73) {
  const model = physicalModel(source);
  const result = { pivotId: null, forceRanges: {}, lengthRanges: {}, requiredLengthRanges: {}, assistForceRanges: {} };
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
      const balance = balanceForce(model, element.id);
      if (balance && Math.abs(balance.momentArm) >= 0.01 && Math.abs(balance.force) <= 1_000_000) {
        trackRange(result.forceRanges, element.id, balance.force);
      }
      const assist = assistForce(model, element.id);
      if (assist && Math.abs(assist.handForce) <= 1_000_000) trackRange(result.assistForceRanges, element.id, assist.handForce);
    }
  }
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
    this.time = 0;
    this.traces = new Map();
    this.settle();
    this.refreshForces();
    this.recordTraces();
    return this.model;
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
    this.solvePositions(iterations, 1 / 60);
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
    }
    for (const { element } of this.members) {
      element.axialForce = appliedAxial.get(element.id) ?? 0;
      element.limitForce = 0;
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
      const tolerance = Math.min(0.002, (max - min) * 0.01);
      if (max <= min + EPSILON) continue;
      if (element.forceMode !== "pull" && direction.length >= max - tolerance) element.forceMode = "pull";
      else if (element.forceMode === "pull" && direction.length <= min + tolerance) element.forceMode = "push";
    }
    this.refreshForces();
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

  addReaction(a, b, direction, force) {
    a.reactionX += (isFixed(a) ? 1 : -1) * direction.x * force;
    a.reactionY += (isFixed(a) ? 1 : -1) * direction.y * force;
    b.reactionX += (isFixed(b) ? -1 : 1) * direction.x * force;
    b.reactionY += (isFixed(b) ? -1 : 1) * direction.y * force;
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
        else if (element && element.kind !== "motor") element.axialForce += force;
        this.addReaction(a, b, direction, force);
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
