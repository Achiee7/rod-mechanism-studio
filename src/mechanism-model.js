const jointKinds = new Set(["origin", "pin", "fixed", "rotation"]);
const elementKinds = new Set(["rod", "body", "actuator", "gas_spring", "motor"]);
const dimensionKinds = new Set(["distance", "horizontal", "vertical"]);
const constraintKinds = new Set(["horizontal", "vertical", "coincident", "equal_length", "parallel"]);

export const isFixed = joint => joint.kind === "fixed" || joint.kind === "rotation" || joint.kind === "origin";
export const isPiston = element => element.kind === "actuator" || element.kind === "gas_spring";
export const isRigidLink = element => element.kind === "rod" || element.kind === "body" || element.kind === "motor";
export const cloneMechanism = model => structuredClone(model);

export function createMechanism() {
  return {
    version: 1,
    joints: [{ id: 0, kind: "origin", name: "Origin", x: 0, y: 0, mass: 0 }],
    elements: [],
    loads: [],
    dimensions: [],
    constraints: [],
    options: { gravity: 9.81, gravityEnabled: true, damping: 0.015, iterations: 16 },
  };
}

export function nextId(items) {
  let id = items.reduce((max, item) => Math.max(max, Number.isSafeInteger(item.id) ? item.id : 0), 0) + 1;
  while (items.some(item => String(item.id) === String(id))) id++;
  return id;
}

function fields(list, name, required = false) {
  if (list === undefined && !required) return [];
  if (!Array.isArray(list)) throw new Error(`The ${name} field must be a list.`);
  const ids = new Set();
  return list.map((item, index) => {
    if (!item || typeof item !== "object") throw new Error(`Invalid entry in ${name} at row ${index + 1}.`);
    if (typeof item.id !== "string" && !Number.isSafeInteger(item.id)) throw new Error(`Invalid ID in ${name} at row ${index + 1}.`);
    if (ids.has(String(item.id))) throw new Error(`Duplicate ID ${item.id} in ${name}.`);
    ids.add(String(item.id));
    return item;
  });
}

function numeric(value, fallback, label) {
  if (value == null) return fallback;
  const result = Number(value);
  if (!Number.isFinite(result)) throw new Error(`${label} must be a finite number.`);
  return result;
}

function kindOrThrow(value, kinds, label) {
  if (!kinds.has(value)) throw new Error(`${label} has an unknown type: ${value}.`);
  return value;
}

export function normalizeMechanism(data) {
  if (!data || typeof data !== "object" || Array.isArray(data)) throw new Error("Choose a mechanism JSON object.");
  const model = createMechanism();
  const jointItems = fields(data.joints, "joints", true);
  // The desktop editor used screen coordinates (positive Y down).
  const flipY = jointItems.some(joint => joint.position && joint.x == null) ? -1 : 1;
  model.joints = jointItems.map(joint => {
    const label = `Joint ${joint.id}`;
    const storedKind = joint.kind ?? "pin";
    const kind = kindOrThrow(joint.construction ? "origin" : joint.fixed && storedKind === "pin" ? "fixed" : storedKind, jointKinds, label);
    return {
      id: joint.id,
      kind,
      name: joint.name ?? (kind === "origin" ? "Origin" : ""),
      x: kind === "origin" ? 0 : numeric(joint.x ?? joint.position?.x, 0, `${label} X`),
      y: kind === "origin" ? 0 : flipY * numeric(joint.y ?? joint.position?.y, 0, `${label} Y`),
      mass: kind === "origin" ? 0 : Math.max(0, numeric(joint.mass, 0.05, `${label} mass`)),
      showForce: joint.showForce ?? joint.show_force ?? true,
    };
  });
  if (!model.joints.some(joint => joint.kind === "origin")) {
    const origin = createMechanism().joints[0];
    if (model.joints.some(joint => String(joint.id) === "0")) origin.id = nextId(model.joints);
    model.joints.unshift(origin);
  }
  const joints = new Map(model.joints.map(joint => [String(joint.id), joint]));
  function reference(id, label) {
    const joint = joints.get(String(id));
    if (!joint) throw new Error(`${label} references missing joint ${id}.`);
    return joint.id;
  }
  model.elements = fields(data.elements, "elements").map(element => {
    const label = `Element ${element.id}`;
    const a = reference(element.a, label);
    const b = reference(element.b, label);
    if (a === b) throw new Error(`${label} requires two different joints.`);
    const length = Math.max(0.001, numeric(element.length, distance(model, a, b), `${label} length`));
    const minLength = Math.max(0.001, numeric(element.minLength ?? element.min_length, length * 0.65, `${label} compressed length`));
    const maxLength = Math.max(minLength, numeric(element.maxLength ?? element.max_length, length, `${label} extended length`));
    const forceMode = element.forceMode ?? element.force_mode;
    return {
      id: element.id, a, b, length, minLength, maxLength,
      kind: kindOrThrow(element.kind ?? "rod", elementKinds, label),
      name: element.name ?? "",
      mass: Math.max(0, numeric(element.mass, 0, `${label} mass`)),
      force: Math.abs(numeric(element.force, 0, `${label} force`)),
      forceMode: forceMode === "pull" || Number(forceMode) < 0 ? "pull" : "push",
      torque: flipY * numeric(element.torque, 0, `${label} torque`),
      stiffness: Math.max(0, numeric(element.stiffness, 0, `${label} stiffness`)),
      damping: Math.max(0, numeric(element.damping, 0, `${label} damping`)),
      autoReturn: element.autoReturn ?? element.auto_return ?? true,
      showForce: element.showForce ?? element.show_force ?? true,
    };
  });
  model.loads = fields(data.loads, "loads").map(load => ({
    id: load.id,
    kind: "point",
    joint: reference(load.joint, `Load ${load.id}`),
    name: load.name ?? "",
    fx: numeric(load.fx ?? load.force?.x, 0, `Load ${load.id} X`),
    fy: flipY * numeric(load.fy ?? load.force?.y, 0, `Load ${load.id} Y`),
    enabled: load.enabled ?? true,
  }));
  model.dimensions = fields(data.dimensions, "dimensions").map(dimension => {
    const label = `Dimension ${dimension.id}`;
    const a = reference(dimension.a, label);
    const b = reference(dimension.b, label);
    const kind = kindOrThrow(dimension.kind ?? "distance", dimensionKinds, label);
    return {
      id: dimension.id, a, b, kind,
      name: dimension.name ?? "",
      offset: (kind === "vertical" ? 1 : flipY) * numeric(dimension.offset, 0.16, `${label} offset`),
      value: Math.max(kind === "distance" ? 0.001 : 0, numeric(dimension.value, distance(model, a, b), `${label} value`)),
      driving: dimension.driving ?? true,
      visible: dimension.visible ?? true,
    };
  });
  model.constraints = fields(data.constraints ?? data.sketch_constraints, "constraints").map(constraint => {
    const label = `Constraint ${constraint.id}`;
    const [a, b, c, d] = constraintEndpoints(model, constraint);
    const kind = kindOrThrow(constraint.kind ?? "horizontal", constraintKinds, label);
    return {
      id: constraint.id, kind,
      a: reference(a, label),
      b: reference(b, label),
      c: kind === "parallel" || kind === "equal_length" ? reference(c, label) : null,
      d: kind === "parallel" || kind === "equal_length" ? reference(d, label) : null,
      name: constraint.name ?? "",
      visible: constraint.visible ?? true,
    };
  });
  const options = data.options ?? {};
  model.options = {
    gravity: Math.max(0, numeric(options.gravity, 9.81, "Gravity")),
    gravityEnabled: options.gravityEnabled ?? options.gravity_enabled ?? true,
    damping: Math.min(1, Math.max(0, numeric(options.damping, 0.015, "Damping"))),
    iterations: Math.max(1, Math.min(200, Math.round(numeric(options.iterations ?? options.constraint_iterations, 16, "Solver iterations")))),
  };
  return model;
}

export function createDemo() {
  return normalizeMechanism({
    joints: [
      { id: 1, kind: "rotation", name: "Hinge", x: -1.2, y: 0 },
      { id: 2, kind: "pin", name: "Hatch tip", x: 0.45, y: 0.45, mass: 0.1 },
      { id: 3, kind: "fixed", name: "Gas base", x: -0.85, y: -0.7 },
      { id: 4, kind: "pin", name: "Gas rod", x: -0.1, y: 0.15 },
    ],
    elements: [
      { id: 1, kind: "body", a: 1, b: 2, name: "Hatch body", mass: 28 },
      { id: 2, kind: "rod", a: 2, b: 4, name: "Hatch mount", mass: 0.5 },
      { id: 3, kind: "gas_spring", a: 3, b: 4, name: "Gas spring", force: 420, damping: 18, minLength: Math.hypot(0.75, 0.85) - 0.3 },
    ],
    dimensions: [
      { id: 1, a: 1, b: 2, name: "Hatch length", offset: -0.16 },
      { id: 2, a: 2, b: 4, name: "Mount length", offset: -0.16 },
      { id: 3, a: 3, b: 4, name: "Gas spring length", offset: -0.16 },
    ],
    loads: [{ id: 1, name: "Extra load", joint: 2, fx: 0, fy: 0 }],
  });
}

export function distance(model, a, b) {
  const first = model.joints.find(joint => joint.id === a);
  const second = model.joints.find(joint => joint.id === b);
  return first && second ? Math.hypot(second.x - first.x, second.y - first.y) : NaN;
}

function constraintEndpoints(model, constraint) {
  const first = model.elements.find(element => element.id === constraint.element);
  const second = model.elements.find(element => element.id === constraint.otherElement);
  return [constraint.a ?? first?.a, constraint.b ?? first?.b, constraint.c ?? second?.a, constraint.d ?? second?.b];
}

export function sketchDriven(model, id) {
  return model.dimensions.some(dimension => dimension.driving !== false && (dimension.a === id || dimension.b === id))
    || model.constraints.some(constraint => constraintEndpoints(model, constraint).includes(id));
}

export function solveSketch(model, { draggedJointId, iterations = 80 } = {}) {
  const joints = new Map(model.joints.map(joint => [joint.id, joint]));
  const mobility = joint => {
    if (joint.kind === "origin") return 0;
    return (isFixed(joint) ? 0.001 : 1) * (joint.id === draggedJointId ? 0.25 : 1);
  };
  function project(a, b, dx, dy) {
    const wa = mobility(a);
    const wb = mobility(b);
    const total = wa + wb;
    if (!total) return 0;
    a.x += dx * wa / total;
    a.y += dy * wa / total;
    b.x -= dx * wb / total;
    b.y -= dy * wb / total;
    return Math.hypot(dx, dy);
  }
  function projectPair(a, b, kind, value) {
    const dx = b.x - a.x;
    const dy = b.y - a.y;
    if (kind === "horizontal") return project(a, b, dx - (dx < 0 ? -value : value), 0);
    if (kind === "vertical") return project(a, b, 0, dy - (dy < 0 ? -value : value));
    if (kind === "coincident") return project(a, b, dx, dy);
    const separation = Math.hypot(dx, dy);
    if (separation < 1e-12) return project(a, b, -value, 0);
    return project(a, b, dx * (1 - value / separation), dy * (1 - value / separation));
  }
  const constraints = model.constraints.map(constraint => ({
    kind: constraint.kind,
    joints: constraintEndpoints(model, constraint).map(id => joints.get(id)),
  }));
  for (let iteration = 0; iteration < iterations; iteration++) {
    let error = 0;
    for (const { kind, joints: [a, b, c, d] } of constraints) {
      if (!a || !b) continue;
      if (kind === "horizontal") error = Math.max(error, projectPair(a, b, "vertical", 0));
      if (kind === "vertical") error = Math.max(error, projectPair(a, b, "horizontal", 0));
      if (kind === "coincident") error = Math.max(error, projectPair(a, b, kind, 0));
      if (!c || !d) continue;
      const dx = b.x - a.x;
      const dy = b.y - a.y;
      const length = Math.hypot(dx, dy);
      if (kind === "equal_length") error = Math.max(error, projectPair(c, d, "distance", length));
      if (kind === "parallel" && length > 1e-12) {
        const tx = d.x - c.x;
        const ty = d.y - c.y;
        const sign = tx * dx + ty * dy < 0 ? -1 : 1;
        const scale = sign * Math.hypot(tx, ty) / length;
        error = Math.max(error, project(c, d, tx - dx * scale, ty - dy * scale));
      }
    }
    for (const dimension of model.dimensions) {
      const a = joints.get(dimension.a);
      const b = joints.get(dimension.b);
      if (a && b && dimension.driving !== false) error = Math.max(error, projectPair(a, b, dimension.kind, dimension.value));
    }
    if (error < 1e-9) break;
  }
  for (const element of model.elements) {
    const a = joints.get(element.a);
    const b = joints.get(element.b);
    if (a && b) element.length = Math.max(0.001, Math.hypot(b.x - a.x, b.y - a.y));
  }
  return model;
}

export function splitElement(model, id, point) {
  const element = model.elements.find(item => item.id === id);
  if (!element || !["rod", "body"].includes(element.kind)) return null;
  const a = model.joints.find(joint => joint.id === element.a);
  const b = model.joints.find(joint => joint.id === element.b);
  const dx = b.x - a.x;
  const dy = b.y - a.y;
  const length = Math.hypot(dx, dy);
  if (length < 0.002) return null;
  const fraction = ((point.x - a.x) * dx + (point.y - a.y) * dy) / (length * length);
  if (fraction * length <= 0.001) return a;
  if ((1 - fraction) * length <= 0.001) return b;
  const joint = {
    id: nextId(model.joints), kind: "pin", name: "", mass: 0.05,
    x: a.x + fraction * dx, y: a.y + fraction * dy,
  };
  const dimensions = model.dimensions.filter(item => (item.a === a.id && item.b === b.id) || (item.a === b.id && item.b === a.id));
  const index = model.elements.indexOf(element);
  const secondId = nextId(model.elements);
  const name = element.name || `${element.kind === "rod" ? "Rod" : "Body"} ${element.id}`;
  const first = { ...element, name: `${name} A`, b: joint.id, length: length * fraction, mass: (element.mass ?? 0) * fraction };
  const second = { ...element, id: secondId, name: `${name} B`, a: joint.id, length: length * (1 - fraction), mass: (element.mass ?? 0) * (1 - fraction) };
  model.joints.push(joint);
  model.elements.splice(index, 1, first, second);
  for (const constraint of model.constraints) {
    // Preserve sketch references to the whole pre-split segment.
    if (constraint.element === id) [constraint.a, constraint.b] = [a.id, b.id];
    if (constraint.otherElement === id) [constraint.c, constraint.d] = [a.id, b.id];
  }
  model.dimensions = model.dimensions.filter(dimension => !dimensions.includes(dimension));
  for (const part of [first, second]) {
    model.dimensions.push({
      id: nextId(model.dimensions), a: part.a, b: part.b, value: part.length,
      kind: "distance", name: `${part.name} length`, offset: 0.16, driving: true, visible: true,
    });
  }
  model.constraints.push({
    id: nextId(model.constraints), kind: "parallel", a: a.id, b: b.id, c: a.id, d: joint.id, visible: false,
  });
  return joint;
}

export function removeObject(model, kind, id) {
  if (kind === "joint") {
    const joint = model.joints.find(item => item.id === id);
    if (!joint || joint.kind === "origin") return false;
    model.constraints = model.constraints.filter(item => !constraintEndpoints(model, item).includes(id));
    model.joints = model.joints.filter(item => item.id !== id);
    model.elements = model.elements.filter(item => item.a !== id && item.b !== id);
    model.dimensions = model.dimensions.filter(item => item.a !== id && item.b !== id);
    model.loads = model.loads.filter(item => item.joint !== id);
    return true;
  }
  const collections = { element: "elements", load: "loads", dimension: "dimensions", constraint: "constraints" };
  const collection = collections[kind];
  if (!collection || !model[collection].some(item => item.id === id)) return false;
  if (kind === "element") {
    model.constraints = model.constraints.filter(item => item.element !== id && item.otherElement !== id);
  }
  model[collection] = model[collection].filter(item => item.id !== id);
  return true;
}
