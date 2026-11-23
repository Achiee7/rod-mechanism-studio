import assert from "node:assert/strict";
import { test } from "node:test";
import {
  analyzePath, assistForce, balanceForce, createDemo, distance, effectiveForce,
  normalizeMechanism, Simulation, solveSketch, splitElement,
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
  const path = analyzePath(hatch);
  close(path.forceRanges[3][0], 310.043821207021);
  close(path.forceRanges[3][1], 324.587358875402);
  close(path.lengthRanges[3][1] - path.lengthRanges[3][0], 0.3);
  close(path.requiredLengthRanges[3][0], 0.327559573798);
  close(path.requiredLengthRanges[3][1], 1.835147344006);
  hatch.elements[2].force = 900;
  const changed = analyzePath(hatch);
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
        id: 1, kind: "gas_spring", a: 1, b: 2, minLength: 0.5, maxLength: 1,
        force: 200, forceMode: mode, autoReturn: false,
      }],
      options: { gravityEnabled: false },
    });
    const simulation = new Simulation(model);
    for (let step = 0; step < 30; step++) {
      simulation.step(1 / 60);
      const length = distance(simulation.model, 1, 2);
      assert.ok(length >= 0.5 - 1e-8 && length <= 1 + 1e-8, `stroke exceeded at ${length} m`);
    }
    const element = simulation.model.elements[0];
    close(distance(simulation.model, 1, 2), mode === "pull" ? 0.5 : 1);
    assert.equal(element.forceMode, mode);
    const expected = mode === "pull" ? 200 : -200;
    close(element.limitForce, expected, 2);
  }
});

test("auto-return reverses at a stroke stop and resets with the simulation", () => {
  const model = normalizeMechanism({
    joints: [point(1, 0, 0, "fixed"), point(2, 1, 0)],
    elements: [{ id: 1, kind: "actuator", a: 1, b: 2, minLength: 0.5, maxLength: 1, force: 15 }],
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
