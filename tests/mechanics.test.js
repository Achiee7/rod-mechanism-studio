import assert from "node:assert/strict";
import { test } from "node:test";
import {
  analyzeLegacyPath, analyzePath, assistForce, balanceForce, createDemo, currentForces, distance, effectiveForce, pathDrivers,
  handleForceBalance, normalizeMechanism, pistonLoads, poseValidity, Simulation, solveSketch, splitElement,
} from "../src/mechanics.js";

function close(actual, expected, tolerance = 1e-8) {
  assert.ok(Math.abs(actual - expected) <= tolerance, `Expected ${actual} to be within ${tolerance} of ${expected}`);
}

const point = (id, x, y, kind = "pin") => ({ id, x, y, kind });

test("hatch sizing and the pivot sweep match the desktop calculator", () => {
  const hatch = createDemo();
  const balance = balanceForce(hatch, 3);
  close(balance.force, 324.587358875402);
  close(balance.moment, -235.513575);
  close(balance.momentArm, 0.725578395339);
  assert.equal(balance.mode, "push");
  close(assistForce(hatch, 3).handForce, -41.957182449819);
  const path = analyzeLegacyPath(hatch);
  close(path.forceRanges[3][0], 310.043821207021);
  close(path.forceRanges[3][1], 324.587358875402);
  close(path.lengthRanges[3][1] - path.lengthRanges[3][0], 0.3);
  close(path.requiredLengthRanges[3][0], 0.327559573798);
  close(path.requiredLengthRanges[3][1], 1.835147344006);
  hatch.elements[2].force = 900;
  const changed = analyzeLegacyPath(hatch);
  assert.deepEqual(changed.forceRanges, path.forceRanges);
  assert.deepEqual(changed.requiredLengthRanges, path.requiredLengthRanges);
  assert.notDeepEqual(changed.assistForceRanges, path.assistForceRanges);
});

test("legacy JSON preserves the direction of loads, torque and the visible placement of dimensions", () => {
  const model = normalizeMechanism({
    version: 2,
    joints: [
      { id: 1, position: { x: 0, y: 0 }, construction: true },
      { id: 2, position: { x: 0, y: 1 }, kind: "pin", fixed: true },
      { id: 3, position: { x: 1, y: 2 } },
    ],
    elements: [
      { id: 1, a: 2, b: 3, kind: "gas_spring", force_mode: -1, min_length: 1, max_length: 2, auto_return: false },
      { id: 2, a: 2, b: 3, kind: "motor", torque: 45 },
    ],
    loads: [{ id: 1, joint: 3, force: { x: -10, y: 50 } }],
    dimensions: [
      { id: 1, a: 2, b: 3, kind: "vertical", value: 1, offset: 0.2 },
      { id: 2, a: 2, b: 3, kind: "distance", value: Math.SQRT2, offset: 0.1 },
    ],
    sketch_constraints: [{ id: 7, kind: "horizontal", a: 1, b: 2 }],
    options: { gravity_enabled: false, constraint_iterations: 32 },
  });
  assert.equal(model.joints[0].kind, "origin");
  assert.equal(model.joints[1].kind, "fixed");
  assert.equal(model.joints[2].y, -2);
  assert.equal(model.elements[0].forceMode, "pull");
  assert.equal(model.elements[0].autoReturn, false);
  assert.equal(model.elements[1].torque, -45);
  assert.equal(model.loads[0].fy, -50);
  assert.equal(model.dimensions[0].offset, 0.2);
  assert.equal(model.dimensions[1].offset, -0.1);
  assert.equal(model.constraints[0].id, 7);
  assert.equal(model.options.gravityEnabled, false);
  assert.equal(model.options.iterations, 32);
  model.elements[0].barrelEnd = "b";
  model.loads[0].showForce = false;
  assert.deepEqual(normalizeMechanism(model), model);
});

test("a sketch origin can drive a real hinge and does not act as a simulation support", () => {
  const model = normalizeMechanism({
    joints: [point(0, 0, 0, "origin"), point(1, 0, 0), point(2, 0.2, 1, "fixed"), point(3, 0, 0, "pin")],
    elements: [
      { id: 1, a: 1, b: 2, kind: "body", mass: 20 },
      { id: 2, a: 2, b: 3, kind: "gas_spring", minLength: 0.4, maxLength: 1.5 },
    ],
    dimensions: [{ id: 1, a: 0, b: 2, kind: "horizontal", value: 0.4 }],
    constraints: [{ id: 1, a: 0, b: 1, kind: "coincident" }],
  });
  solveSketch(model);
  close(model.joints[2].x, 0.4);
  assert.equal(balanceForce(model, 2), null);
  // This represents an overlap from an older workspace with a separate hinge object.
  model.joints.push(point(4, 0, 0, "rotation"));
  const simulation = new Simulation(model);
  assert.equal(simulation.model.joints.some(joint => joint.kind === "origin"), false);
  assert.equal(simulation.model.joints.some(joint => joint.id === 1), false);
  assert.equal(simulation.model.elements[0].a, 4);
});

test("driving piston dimensions change sketch geometry while leaving stroke endpoints in place", () => {
  const model = normalizeMechanism({
    joints: [point(1, 0, 0, "fixed"), point(2, 1, 1)],
    elements: [{ id: 1, a: 1, b: 2, kind: "actuator", minLength: 0.6, maxLength: 2 }],
    dimensions: [{ id: 1, a: 1, b: 2, value: 1.8 }],
  });
  solveSketch(model);
  close(distance(model, 1, 2), 1.8);
  assert.equal(model.elements[0].minLength, 0.6);
  assert.equal(model.elements[0].maxLength, 2);
});

test("connected weighted body segments preserve their angle and mass when split", () => {
  const model = normalizeMechanism({
    joints: [point(1, 0, 0, "rotation"), point(2, 1, 0), point(3, 1, 0.6)],
    elements: [{ id: 1, a: 1, b: 2, kind: "body", mass: 5 }, { id: 2, a: 2, b: 3, kind: "body", mass: 3 }],
  });
  const joint = splitElement(model, 1, { x: 0.4, y: 0.02 });
  close(joint.x, 0.4);
  assert.equal(joint.y, 0);
  assert.equal(model.elements.length, 3);
  close(model.elements.reduce((sum, element) => sum + element.mass, 0), 8);
  const simulation = new Simulation(model);
  simulation.setTracing(true);
  const end = simulation.model.joints.find(item => item.id === 3);
  simulation.poseJoint(3, end.x, end.y + 0.5);
  assert.ok(end.y > 0.6);
  for (let step = 0; step < 60; step++) simulation.step(1 / 60);
  close(distance(simulation.model, 1, 3), Math.hypot(1, 0.6), 1e-5);
  close(distance(simulation.model, 1, 2), 1, 1e-5);
  assert.ok(simulation.traces.get(3).length > 10);
  simulation.reset();
  assert.equal(simulation.time, 0);
  close(simulation.model.joints.find(item => item.id === 3).y, 0.6);
});

test("a motor applies torque with a constant lever radius", () => {
  const model = normalizeMechanism({
    joints: [point(1, 0, 0, "rotation"), point(2, 1, 0)],
    elements: [{ id: 1, a: 1, b: 2, kind: "motor", torque: 5 }],
    options: { gravityEnabled: false },
  });
  const simulation = new Simulation(model);
  simulation.step(1 / 60);
  assert.ok(simulation.model.joints[1].y > 0);
  close(distance(simulation.model, 1, 2), 1);
  close(simulation.model.elements[0].axialForce, 5, 1e-3);
  assert.equal(model.joints.find(joint => joint.id === 2).y, 0);
});

test("gas springs honor both stroke limits and report stop reaction", () => {
  for (const mode of ["push", "pull"]) {
    const model = normalizeMechanism({
      joints: [point(1, 0, 0, "fixed"), point(2, 0.75, 0)],
      elements: [{
        id: 1, kind: "gas_spring", a: 1, b: 2, minLength: 0.6, maxLength: 1,
        force: 200, forceMode: mode, autoReturn: false,
      }],
      options: { gravityEnabled: false },
    });
    const simulation = new Simulation(model);
    for (let step = 0; step < 30; step++) {
      simulation.step(1 / 60);
      const length = distance(simulation.model, 1, 2);
      assert.ok(length >= 0.6 - 1e-8 && length <= 1 + 1e-8, `stroke exceeded at ${length} m`);
    }
    const element = simulation.model.elements[0];
    close(distance(simulation.model, 1, 2), mode === "pull" ? 0.6 : 1);
    assert.equal(element.forceMode, mode);
    const expected = mode === "pull" ? 200 : -200;
    close(element.limitForce, expected, 2);
  }
});

test("auto-return reverses at a stroke stop and resets with the simulation", () => {
  const model = normalizeMechanism({
    joints: [point(1, 0, 0, "fixed"), point(2, 1, 0)],
    elements: [{ id: 1, kind: "actuator", a: 1, b: 2, minLength: 0.6, maxLength: 1, force: 15 }],
    options: { gravityEnabled: false },
  });
  const simulation = new Simulation(model);
  simulation.step(1 / 120);
  assert.equal(simulation.model.elements[0].forceMode, "pull");
  assert.ok(distance(simulation.model, 1, 2) < 1);
  simulation.reset();
  assert.equal(simulation.model.elements[0].forceMode, "push");
});

test("gas spring damping can oppose motion without reversing force direction", () => {
  const model = normalizeMechanism({
    joints: [point(1, 0, 0, "fixed"), point(2, 0.75, 0)],
    elements: [{ id: 1, kind: "gas_spring", a: 1, b: 2, minLength: 0.5, maxLength: 1, force: 2, damping: 40 }],
  });
  const moving = model.joints.find(joint => joint.id === 2);
  moving.vx = 1;
  assert.equal(effectiveForce(model, 1), 0);
  model.elements[0].forceMode = "pull";
  moving.vx = -1;
  assert.equal(effectiveForce(model, 1), 0);
});

test("the constrained hatch sweep distinguishes installed travel from the requested geometry and sizing", () => {
  const model = createDemo();
  const result = analyzePath(model);
  const spring = model.elements[2];
  assert.equal(result.status, "partial");
  assert.ok(result.poses.some(pose => pose.feasible));
  assert.ok(result.poses.some(pose => !pose.feasible && pose.status === "stroke" && pose.pistons[3].force != null));
  close(result.forceRanges[3][1], 324.587358875402);
  assert.ok(result.recommendations[3].stroke > spring.maxLength - spring.minLength);
  assert.ok(result.requiredForceRanges[3][1] > result.forceRanges[3][1]);
  assert.equal(result.recommendations[3].mode, "both");
  assert.equal(result.recommendations[3].status, "partial");
  const pose = result.poses.find(pose => pose.angleDeg === 0);
  assert.equal(pose.pistons[3].handleId, 2);
  close(pose.pistons[3].assistForce, -41.957182449819);
  spring.force *= 2;
  assert.deepEqual(analyzePath(model).requiredForceRanges, result.requiredForceRanges);
});

test("the hatch cycles through both piston stops while its rigid body remains intact", () => {
  const model = createDemo();
  model.elements[2].autoReturn = true;
  const simulation = new Simulation(model);
  const modes = new Set();
  let transitions = 0;
  let previous = simulation.model.elements[2].forceMode;
  for (let index = 0; index < 240; index++) {
    const { maxConstraintError } = simulation.step(1 / 60);
    assert.ok(maxConstraintError < 1.1e-8, `maximum constraint error was ${maxConstraintError} m`);
    const mode = simulation.model.elements[2].forceMode;
    modes.add(mode);
    if (mode !== previous) transitions++;
    previous = mode;
  }
  assert.deepEqual(modes, new Set(["push", "pull"]));
  assert.ok(transitions >= 4);
  assert.equal(currentForces(simulation.model, { dynamic: true }).status, "dynamic");
});

test("the four-bar stays on its assembly branch and reports geometric closure limits", () => {
  const model = normalizeMechanism({
    joints: [point(1, 0, 0, "rotation"), point(2, 0.5, 0.866), point(3, 1.55, 1.2), point(4, 2, 0, "fixed")],
    elements: [
      { id: 1, a: 1, b: 2, kind: "rod", mass: 20 }, { id: 2, a: 2, b: 3, kind: "rod", mass: 5 },
      { id: 3, a: 3, b: 4, kind: "rod", mass: 15 },
      { id: 4, a: 4, b: 2, kind: "actuator", minLength: 1.2, maxLength: 2.1, force: 10, autoReturn: false },
    ],
  });
  assert.deepEqual(pathDrivers(model).map(driver => driver.id), [1, 3]);
  const path = analyzePath(model, { driverId: 1, minAngle: -30, maxAngle: 50 });
  assert.equal(path.status, "partial");
  assert.ok(path.poses.some(pose => pose.status === "geometry" && pose.positions === null));
  assert.ok(path.poses.some(pose => pose.feasible));
  for (const pose of path.poses.filter(item => item.feasible)) {
    const pts = pose.positions;
    assert.deepEqual(pts[1], { x: 0, y: 0 });
    assert.deepEqual(pts[4], { x: 2, y: 0 });
    for (const { a, b, length } of model.elements.slice(0, 3)) {
      close(Math.hypot(pts[a].x - pts[b].x, pts[a].y - pts[b].y), length, 1e-6);
    }
  }
  assert.equal(balanceForce(model, 4).lineOfAction, false);
  const budget = handleForceBalance(model, { pistonId: 4 });
  assert.equal(budget.status, "available");
  assert.equal(budget.basis, "one-dof");
  close(budget.extra, path.poses.find(pose => pose.angleDeg === 0).pistons[4].assistForce);
  for (const physicsMode of ["real", "forced"]) {
    const simulation = new Simulation(model, { physicsMode });
    let result;
    for (let index = 0; index < 180; index++) {
      result = simulation.step();
      if (!result.accepted) break;
    }
    close(distance(simulation.model, 2, 4), physicsMode === "real" ? 1.2 : 2.1, 1e-6);
    assert.ok(result.maxConstraintError < 1e-6);
  }
});

test("underdefined mechanisms do not return an invented sizing path", () => {
  const model = createDemo();
  model.elements[1].kind = "rod";
  const result = analyzePath(model);
  assert.equal(result.status, "unavailable");
  assert.match(result.message, /2 free motions/);
  assert.equal(balanceForce(model, 3), null);
  assert.ok(analyzeLegacyPath(model).forceRanges[3]);
});

test("static force diagrams distinguish applied, member and support forces", () => {
  const model = normalizeMechanism({
    joints: [point(1, 0, 0, "fixed"), point(2, 0, 1)],
    elements: [
      { id: 1, a: 1, b: 2, kind: "rod" },
      { id: 2, a: 1, b: 2, kind: "actuator", minLength: 0.85, maxLength: 1.5, force: 10 },
    ],
    loads: [{ id: 1, joint: 2, fx: 0, fy: -100 }],
    options: { gravityEnabled: false },
  });
  const result = currentForces(model);
  assert.equal(result.status, "balanced");
  close(result.joints[1].support.y, 100);
  close(result.joints[2].applied.y, -90);
  close(result.joints[2].member.y, 90);
  close(result.joints[2].net.y, 0);
  close(result.elements[1].axialForce, 90);
  assert.ok(result.arrows.some(arrow => arrow.kind === "load" && arrow.fy === -100));
  assert.ok(result.arrows.some(arrow => arrow.kind === "piston" && arrow.fy === 10));
  model.loads[0].fx = 20;
  const unbalanced = currentForces(model);
  assert.equal(unbalanced.status, "unbalanced");
  assert.equal(unbalanced.joints[1].support, null);
  close(unbalanced.balanceResidual, 20);
  model.elements[1].minLength = 1.2;
  assert.equal(currentForces(model).strokeViolation, true);
});

test("manual hand demand counts all installed pistons without changing single-piston sizing", () => {
  const model = createDemo();
  const original = assistForce(model, 3);
  model.elements.push({ ...model.elements[2], id: 4, force: 50 });
  const both = assistForce(model, 3);
  close(both.requiredForce, original.requiredForce);
  assert.ok(both.handForce < original.handForce);
  const pose = analyzePath(model, { minAngle: 0, maxAngle: 0, samples: 1 }).poses[0];
  close(pose.pistons[3].assistForce, both.handForce);
});

test("redundant internal force distributions are flagged as estimates", () => {
  const model = normalizeMechanism({
    joints: [point(1, 0, 0, "fixed"), point(2, 0, 1)],
    elements: [{ id: 1, a: 1, b: 2, kind: "rod" }, { id: 2, a: 1, b: 2, kind: "rod" }],
    loads: [{ id: 1, joint: 2, fx: 0, fy: -100 }],
    options: { gravityEnabled: false },
  });
  const report = currentForces(model);
  assert.equal(report.status, "balanced");
  assert.equal(report.indeterminate, true);
  assert.equal(report.elements[1].estimated, true);
  close(report.elements[1].axialForce + report.elements[2].axialForce, 100);
  const hatch = currentForces(createDemo());
  assert.equal(hatch.elements[2].estimated, true);
  assert.ok(hatch.arrows.some(arrow => arrow.constraint && arrow.estimated));
});

test("ambiguous drive selection and independent links do not invent a shared path", () => {
  const model = createDemo();
  model.joints.push(point(5, 4, 0, "fixed"), point(6, 5, 0));
  model.elements.push({ id: 4, a: 5, b: 6, kind: "rod", length: 1 });
  assert.equal(analyzePath(model).status, "unavailable");
  const chosen = analyzePath(model, { driverId: 1, minAngle: -22, maxAngle: 0 });
  assert.equal(chosen.status, "ok");
  for (const pose of chosen.poses) assert.deepEqual(pose.positions[6], { x: 5, y: 0 });
  model.elements.push({ id: 5, a: 2, b: 6, kind: "actuator", minLength: 1, maxLength: 5, length: 1 });
  assert.match(analyzePath(model, { driverId: 1 }).message, /two independently moving linkages/);
  assert.equal(balanceForce(model, 3), null);
});

function straightPiston({ fixed = true, force = 10, forceMode = "push", kind = "actuator", y = 0.75 } = {}) {
  return normalizeMechanism({
    joints: [point(1, 0, 0, fixed ? "fixed" : "pin"), { ...point(2, 0, y), mass: 30 }],
    elements: [{ id: 1, a: 1, b: 2, kind, minLength: 0.6, maxLength: 1, force, forceMode, autoReturn: false }],
  });
}

test("rated playback backdrives an under-rated piston instead of supplying missing force", () => {
  const simulation = new Simulation(straightPiston());
  const start = currentForces(simulation.model).pistons[1];
  close(start.requiredForce, 294.3);
  close(start.ratedForce, 10);
  close(start.margin, -284.3);
  close(start.utilization, 29.43);
  assert.equal(start.overload, true);
  simulation.step();
  assert.ok(distance(simulation.model, 1, 2) < 0.75);
  assert.equal(pistonLoads(simulation.model)[1].driveState, "backdriven");
  for (let index = 0; index < 100; index++) assert.equal(simulation.step().accepted, true);
  close(distance(simulation.model, 1, 2), 0.6);
  const end = currentForces(simulation.model, { dynamic: true }).pistons[1];
  assert.equal(end.atStop, "min");
  close(end.appliedForce, 10);
  assert.equal(end.driveState, "stroke-stop");
});

test("forced motion advances a piston without an angular driver and stops at its physical limit", () => {
  for (const fixed of [true, false]) {
    for (const mode of ["push", "pull"]) {
      const simulation = new Simulation(straightPiston({ fixed, force: 0, forceMode: mode }), { physicsMode: "forced" });
      let steps = 0;
      while (steps < 200 && simulation.step().accepted) steps++;
      assert.ok(steps > 3 && steps < 200);
      assert.equal(simulation.validity.valid, true);
      close(distance(simulation.model, 1, 2), mode === "push" ? 1 : 0.6, 1e-6);
      if (fixed) assert.equal(simulation.model.joints[0].y, 0);
      assert.match(simulation.motionMessage, /stroke limit/);
      const report = currentForces(simulation.model, { dynamic: true });
      assert.equal(report.pistons[1].driveState, "stroke-stop");
      assert.equal(report.elements[1].axialForce, null);
      simulation.physicsMode = "real";
      assert.equal(simulation.step().accepted, true);
    }
  }
});

test("starting outside stroke or with an impossible barrel preserves the drawn pose and blocks motion", () => {
  for (const mode of ["real", "forced"]) {
    const outside = straightPiston({ y: 1.1 });
    const simulation = new Simulation(outside, { physicsMode: mode });
    assert.equal(simulation.validity.valid, false);
    assert.equal(simulation.validity.issues[0].code, "stroke");
    assert.equal(simulation.step().accepted, false);
    assert.equal(simulation.poseJoint(2, 0, 0.8).accepted, false);
    close(distance(simulation.model, 1, 2), 1.1);
    assert.equal(simulation.time, 0);
  }
  const incompatible = createDemo();
  incompatible.elements[2].minLength = 0.2;
  assert.equal(poseValidity(incompatible).issues[0].code, "piston-config");
  const path = analyzePath(incompatible, { minAngle: -12, maxAngle: 0 });
  assert.ok(!path.poses.some(pose => pose.feasible));
  assert.match(path.message, /barrel/);
  const reversed = straightPiston();
  reversed.elements[0].maxLength = 0.1;
  assert.equal(normalizeMechanism(reversed).elements[0].maxLength, 0.1);
  assert.equal(poseValidity(reversed).valid, false);
});

test("dragging a hinged assembly clamps at a real piston stop without deforming members", () => {
  const simulation = new Simulation(createDemo());
  const result = simulation.poseJoint(2, 0.48, -0.6);
  assert.equal(result.validity.valid, true);
  assert.equal(result.clamped, true);
  const spring = simulation.model.elements[2];
  close(distance(simulation.model, spring.a, spring.b), spring.minLength, 1e-7);
  close(result.maxConstraintError, 0, 1e-7);
});

test("multiple piston loads are directional and coupled stop reactions remain unknown", () => {
  const model = straightPiston();
  model.elements.push({ ...model.elements[0], id: 2, force: 50 });
  close(pistonLoads(model)[1].requiredForce, 244.3);
  close(pistonLoads(model)[2].requiredForce, 284.3);
  model.elements[1].minLength = 0.75;
  assert.equal(pistonLoads(model)[1].status, "unknown");
  assert.equal(pistonLoads(model)[1].overload, null);
  model.elements.pop();
  model.elements[0].forceMode = "pull";
  assert.equal(pistonLoads(model)[1].status, "assisted");
  assert.equal(pistonLoads(model)[1].overload, false);
});

test("opposite forced commands and rigidly blocked piston motion do not move the assembly", () => {
  const model = straightPiston();
  model.elements.push({ ...model.elements[0], id: 2, forceMode: "pull" });
  let simulation = new Simulation(model, { physicsMode: "forced" });
  assert.equal(simulation.step().accepted, false);
  assert.match(simulation.motionMessage, /blocks the commanded movement/);
  close(distance(simulation.model, 1, 2), 0.75);
  model.elements[1].kind = "rod";
  assert.equal(pistonLoads(model)[1].status, "blocked");
  simulation = new Simulation(model, { physicsMode: "forced" });
  assert.equal(simulation.step().accepted, false);
  close(distance(simulation.model, 1, 2), 0.75);
});

test("legacy springs default to passive and never reverse at a stop without explicit auto-return", () => {
  const model = createDemo();
  assert.equal(model.elements[2].autoReturn, false);
  const simulation = new Simulation(model);
  for (let index = 0; index < 60; index++) assert.equal(simulation.step().accepted, true);
  assert.equal(simulation.model.elements[2].forceMode, "push");
  const hatch = simulation.model.elements[2];
  close(distance(simulation.model, hatch.a, hatch.b), hatch.maxLength, 1e-7);
  assert.equal(pistonLoads(simulation.model)[hatch.id].atStop, "max");
});

test("forced return leaves the hatch spring rated for push and reports the needed external pull", () => {
  const model = createDemo();
  const simulation = new Simulation(model, { physicsMode: "forced" });
  const spring = simulation.model.elements[2];
  assert.equal(simulation.step().accepted, false);
  assert.equal(simulation.reverseForcedTravel(), true);
  assert.equal(simulation.forcedReversed, true);
  const report = pistonLoads(simulation.model)[spring.id];
  assert.equal(report.mode, "push");
  assert.equal(report.commandMode, "pull");
  assert.equal(report.ratedForce, 420);
  assert.equal(report.commandAvailableForce, 0);
  close(report.requiredForce, 324.587358875402);
  close(report.commandRequiredForce, -95.412641124598);
  assert.equal(report.commandAssessment, "externally-driven");
  assert.equal(report.overload, true);
  assert.equal(report.ratedOverload, false);
  const start = distance(simulation.model, spring.a, spring.b);
  assert.equal(simulation.step().accepted, true);
  const duringReturn = distance(simulation.model, spring.a, spring.b);
  assert.ok(duringReturn < start);
  assert.equal(spring.forceMode, "push");
  assert.equal(model.elements[2].forceMode, "push");
  assert.equal(simulation.reverseForcedTravel(), false);
  assert.equal(simulation.step().accepted, true);
  assert.ok(distance(simulation.model, spring.a, spring.b) > duringReturn);
  simulation.reverseForcedTravel();
  simulation.reset();
  assert.equal(simulation.forcedReversed, false);
  assert.equal(pistonLoads(simulation.model)[spring.id].commandMode, "push");
});

test("gravity can assist reverse travel rather than overloading its zero rated pull capacity", () => {
  const simulation = new Simulation(straightPiston({ y: 1 }), { physicsMode: "forced" });
  simulation.reverseForcedTravel();
  const report = pistonLoads(simulation.model)[1];
  assert.equal(report.commandAssessment, "load-assisted");
  assert.equal(report.overload, false);
  assert.equal(report.ratedOverload, true);
  close(report.commandRequiredForce, 284.3);
  assert.equal(simulation.step().accepted, true);
  assert.ok(distance(simulation.model, 1, 2) < 1);
  simulation.physicsMode = "real";
  assert.equal(simulation.forcedReversed, false);
  assert.equal(pistonLoads(simulation.model)[1].commandMode, "push");
  simulation.physicsMode = "forced";
  assert.equal(simulation.forcedReversed, false);
  assert.equal(pistonLoads(simulation.model)[1].commandMode, "push");
});

test("actuator auto-return cycles its forced commands without editing the physical rating", () => {
  const model = straightPiston({ y: 1 });
  model.elements[0].autoReturn = true;
  const simulation = new Simulation(model, { physicsMode: "forced" });
  const element = simulation.model.elements[0];
  assert.equal(simulation.step().accepted, true);
  assert.equal(element.commandMode, "pull");
  assert.equal(element.forceMode, "push");
  let reachedReturn = false;
  for (let index = 0; index < 250; index++) {
    assert.equal(simulation.step().accepted, true);
    assert.equal(element.forceMode, "push");
    if (element.commandMode === "push") reachedReturn = true;
  }
  assert.equal(element.force, 10);
  assert.equal(reachedReturn, true);
});

test("the ramp force budget separates raw weight, vertical effort, and axial command shortfall", () => {
  const model = normalizeMechanism({
    joints: [
      { ...point(1, 0, 0, "rotation"), mass: 400 },
      { ...point(2, 0.7453, -0.07), name: "Ramp tip" },
      { ...point(3, 0.748, -0.24928, "fixed"), mass: 900 },
    ],
    elements: [
      { id: 1, a: 1, b: 2, kind: "body", mass: 10 },
      { id: 2, a: 3, b: 2, kind: "gas_spring", force: 100, forceMode: "pull", minLength: 0.15, maxLength: 0.18 },
    ],
    loads: [{ id: 1, joint: 3, fx: 0, fy: -1000 }],
  });
  const simulation = new Simulation(model, { physicsMode: "forced" });
  simulation.reverseForcedTravel();
  const { handleBalance: budget, pistons } = currentForces(simulation.model, { pistonId: 2, physicsMode: "forced" });
  assert.equal(budget.status, "available");
  assert.equal(budget.handleName, "Ramp tip");
  assert.equal(budget.forceBasis, "configured-in-forced-pose");
  close(budget.downwardExternalLoad, 98.5905);
  close(budget.pointDownwardLoad, 0);
  close(budget.equivalentExternal, 49.5405);
  close(budget.pistonContribution, -99.847228764);
  close(budget.extra, 149.387728764);
  close(pistons[2].demandWithoutPiston, 49.616299434);
  close(pistons[2].commandDemandWithoutPiston, 49.616299434);
  assert.equal(pistons[2].commandCapacity, 0);
  close(pistons[2].commandContribution, -100);
  close(pistons[2].commandShortfall, 149.616299434);
  close(pistons[2].signedCommandShortfall, 149.616299434);
  assert.equal(pistons[2].appliedForce, null);
});

test("inclined pistons contribute through the same handle lever and can cancel each other", () => {
  const model = normalizeMechanism({
    joints: [point(1, 0, 0, "rotation"), point(2, 1, 0.5), point(3, 0, -0.5, "fixed")],
    elements: [
      { id: 1, a: 1, b: 2, kind: "body", mass: 12 },
      { id: 2, a: 3, b: 2, kind: "actuator", minLength: 1.25, maxLength: 1.55, force: 100 },
      { id: 3, a: 3, b: 2, kind: "gas_spring", minLength: 1.25, maxLength: 1.55, force: 40, forceMode: "pull" },
    ],
    loads: [
      { id: 1, joint: 2, fx: 10, fy: -20 },
      { id: 2, joint: 2, fx: 0, fy: -30, enabled: false },
    ],
  });
  const budget = handleForceBalance(model, { pistonId: 3, handleId: "2" });
  assert.equal(budget.status, "available");
  assert.equal(budget.handleName, "2");
  close(budget.gravityLoad, 118.2105);
  close(budget.pointDownwardLoad, 20);
  close(budget.downwardExternalLoad, 138.2105);
  close(budget.equivalentExternal, 84.3505);
  close(budget.pistons[0].contribution, 25 * Math.SQRT2);
  close(budget.pistons[1].contribution, -10 * Math.SQRT2);
  close(budget.pistonContribution, 15 * Math.SQRT2);
  close(budget.extra, 84.3505 - 15 * Math.SQRT2);
  const otherHandle = handleForceBalance(createDemo(), { pistonId: 3, handleId: 4 });
  assert.equal(otherHandle.handleName, "Gas rod");
  close(otherHandle.extra, assistForce(createDemo(), 3).handForce * 1.65 / 1.1);
});

test("rated playback uses its measured piston output while a posed sketch uses configured capacity", () => {
  const model = straightPiston({ force: 100 });
  model.options.gravityEnabled = false;
  model.elements[0].damping = 50;
  model.loads.push({ id: 1, joint: 2, fx: 0, fy: -206 });
  const simulation = new Simulation(model);
  simulation.model.joints.find(joint => joint.id === 2).vy = 1;
  simulation.step(1 / 120);
  const staticReport = currentForces(simulation.model, { pistonId: 1 });
  assert.equal(staticReport.handleBalance.basis, "vertical-axis");
  assert.equal(staticReport.handleBalance.forceBasis, "configured-at-pose");
  close(staticReport.handleBalance.extra, 106);
  const live = currentForces(simulation.model, { dynamic: true, pistonId: 1 });
  assert.equal(live.handleBalance.forceBasis, "measured-rated");
  close(live.handleBalance.equivalentExternal, 206);
  close(live.handleBalance.pistonContribution, 50);
  close(live.handleBalance.extra, 156);
  close(live.pistons[1].commandDemandWithoutPiston, 206);
  close(live.pistons[1].commandContribution, 50);
  close(live.pistons[1].commandCapacity, 100);
  close(live.pistons[1].commandShortfall, 156);
});

test("vertical effort stays unavailable for inclined free travel, singular handles, and multiple free modes", () => {
  const free = straightPiston();
  free.joints.find(joint => joint.id === 2).x = 0.2;
  assert.equal(handleForceBalance(free, { pistonId: 1 }).status, "unavailable");
  const legacy = createDemo();
  legacy.elements[1].kind = "rod";
  const indeterminate = handleForceBalance(legacy, { pistonId: 3 });
  assert.equal(indeterminate.status, "unavailable");
  assert.equal(indeterminate.extra, null);
  assert.ok(indeterminate.downwardExternalLoad > 0);
  const singular = normalizeMechanism({
    joints: [point(1, 0, 0, "rotation"), point(2, 0, 1)],
    elements: [{ id: 1, a: 1, b: 2, kind: "body", mass: 12 }],
  });
  assert.match(handleForceBalance(singular, { handleId: 2 }).reason, /cannot move vertically/);
});

test("weightless joints carry no downward force while integration retains finite mass", () => {
  const model = normalizeMechanism({
    joints: [
      { ...point(1, 0, 0, "rotation"), mass: 0 },
      { ...point(2, 1, 0), mass: 0 },
      { ...point(3, 1, -0.8, "fixed"), mass: 0 },
    ],
    elements: [
      { id: 1, a: 1, b: 2, kind: "rod" },
      { id: 2, a: 3, b: 2, kind: "actuator", minLength: 0.7, maxLength: 1.1, force: 0 },
    ],
  });
  const report = currentForces(model, { pistonId: 2 });
  assert.equal(report.handleBalance.status, "available");
  assert.equal(report.handleBalance.gravityLoad, 0);
  assert.equal(report.handleBalance.downwardExternalLoad, 0);
  assert.equal(report.handleBalance.equivalentExternal, 0);
  assert.equal(report.handleBalance.extra, 0);
  close(report.pistons[2].demandWithoutPiston, 0);
  assert.ok(!report.arrows.some(arrow => arrow.kind === "gravity"));
  const simulation = new Simulation(model);
  assert.equal(simulation.step(1 / 120).accepted, true);
  assert.equal(simulation.model.joints.find(joint => joint.id === 2).y, 0);
  simulation.model.elements.find(element => element.id === 2).force = 1;
  assert.equal(simulation.step(1 / 120).accepted, true);
  const tip = simulation.model.joints.find(joint => joint.id === 2);
  assert.ok(tip.y > 0 && Number.isFinite(tip.vy));
  assert.ok(!currentForces(simulation.model, { dynamic: true }).arrows.some(arrow => arrow.kind === "gravity"));
});
