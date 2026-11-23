import assert from 'node:assert/strict';
import test from 'node:test';
import { assistArrowLabel, marqueeSelection, pistonGeometry, pistonShortfallLabel } from '../src/canvas.js';

test('a window requires containment while a crossing window includes intersected links', () => {
  const positions = {
    0: { x: 5, y: 5 },
    1: { x: -5, y: 4 },
    2: { x: 15, y: 4 },
    3: { x: 2, y: 6 },
    4: { x: 8, y: 6 },
    5: { x: 8, y: -5 },
    6: { x: 15, y: 2 },
  };
  const model = {
    joints: Object.keys(positions).map((id) => ({ id: Number(id), kind: id === '0' ? 'origin' : 'pin' })),
    elements: [{ id: 1, a: 1, b: 2 }, { id: 2, a: 3, b: 4 }, { id: 3, a: 5, b: 6 }],
    loads: [{ id: 1, joint: 3 }],
  };
  const position = (id) => positions[id];
  const topLeft = { x: 0, y: 0 };
  const bottomRight = { x: 10, y: 10 };
  const joints = [{ type: 'joint', id: 3 }, { type: 'joint', id: 4 }];
  assert.deepEqual(marqueeSelection(model, topLeft, bottomRight, position), [
    ...joints, { type: 'element', id: 2 }, { type: 'load', id: 1 },
  ]);
  assert.deepEqual(marqueeSelection(model, bottomRight, topLeft, position, { loads: false }), [
    ...joints, { type: 'element', id: 1 }, { type: 'element', id: 2 },
  ]);
});

test('the piston retains its barrel and end fittings at either stroke limit', () => {
  const piston = { length: .9, minLength: .8, maxLength: 1.3 };
  const compressed = pistonGeometry(piston, piston.minLength);
  const extended = pistonGeometry(piston, piston.maxLength);
  assert.deepEqual(compressed, extended);
  assert.equal(compressed.valid, true);
  assert.equal(compressed.problem, undefined);
  assert(compressed.barrelLength > 0);
  assert(Math.abs(compressed.barrelLength + 2 * compressed.fittingLength - piston.minLength) < 1e-9);
});

test('a piston flags incompatible stroke and poses beyond configured reach', () => {
  const piston = { length: .9, minLength: .8, maxLength: 1.3 };
  const below = pistonGeometry(piston, .7);
  const beyond = pistonGeometry(piston, 1.4);
  assert(below.configurationValid && !below.valid);
  assert(beyond.configurationValid && !beyond.valid);
  assert.match(below.problem, /Below compressed/);
  assert.match(beyond.problem, /Beyond extended/);
  const impossible = pistonGeometry({ ...piston, maxLength: 1.8 }, .95);
  assert(!impossible.configurationValid && !impossible.valid);
  assert.match(impossible.problem, /Stroke exceeds/);
  assert.match(pistonGeometry({ ...piston, maxLength: .7 }, .8).problem, /Invalid compressed or extended/);
});

test('a piston badge names the extra axial push and its accessible text explains the aiding force', () => {
  const label = pistonShortfallLabel({
    overload: true, commandMode: 'push', commandShortfall: 149.6, signedCommandShortfall: 149.6,
    demandWithoutPiston: 324.6, commandContribution: 175,
  });
  assert.equal(label.text, '149,6 N more push');
  assert.match(label.detail, /along the piston axis/);
  assert.match(label.detail, /324,6 N of push/);
  assert.match(label.detail, /supplies 175 N toward this motion/);
});

test('reverse overload shows more pull even at zero command capacity and identifies the opposing push', () => {
  const label = pistonShortfallLabel({
    overload: true, mode: 'push', commandMode: 'pull', commandCapacity: 0, commandShortfall: 146.4,
    signedCommandShortfall: -146.4, demandWithoutPiston: 3.6, commandContribution: -150,
  });
  assert.equal(label.text, '146,4 N more pull');
  assert.match(label.detail, /3,6 N of push/);
  assert.match(label.detail, /opposes this motion with 150 N/);
});

test('a piston badge handles zero capacity, zero shortfall, and unavailable force data', () => {
  const noCapacity = pistonShortfallLabel({ overload: true, mode: 'push', requiredForce: 40, availableForce: 0 });
  assert.equal(noCapacity.text, '40 N more push');
  assert.match(noCapacity.detail, /supplies no force/);
  assert.equal(pistonShortfallLabel({ overload: true, commandMode: 'push', commandShortfall: 0 }), null);
  assert.equal(pistonShortfallLabel({ overload: null, status: 'unknown' }), null);
  assert.equal(pistonShortfallLabel({ overload: true }).text, 'Piston overloaded');
});

test('manual assist labels report actual up, down, and inclined drawing directions', () => {
  assert.equal(assistArrowLabel({ fx: 0, fy: 42 }, 'Hatch tip'), 'Manual lift · 42 N up at Hatch tip');
  assert.equal(assistArrowLabel({ fx: 0, fy: 42, atStop: true }, 'Hatch tip'), 'Release from stop · 42 N up at Hatch tip');
  assert.equal(assistArrowLabel({ fx: 0, fy: -42 }, 'Hatch tip'), 'Manual force · 42 N down at Hatch tip');
  assert.equal(assistArrowLabel({ fx: -30, fy: 40 }, 'Hatch tip'), 'Manual force · 50 N up and left at Hatch tip');
  assert.equal(assistArrowLabel({ fx: 0, fy: 0 }, 'Hatch tip'), null);
});
