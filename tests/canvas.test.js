import assert from 'node:assert/strict';
import test from 'node:test';
import { marqueeSelection, pistonGeometry } from '../src/canvas.js';

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
  assert.equal(compressed.problem, undefined);
  assert(compressed.barrelLength > 0);
  assert(Math.abs(compressed.barrelLength + 2 * compressed.fittingLength - piston.minLength) < 1e-9);
});

test('a piston flags incompatible stroke and poses beyond configured reach', () => {
  const piston = { length: .9, minLength: .8, maxLength: 1.3 };
  assert.match(pistonGeometry(piston, .7).problem, /Below compressed/);
  assert.match(pistonGeometry(piston, 1.4).problem, /Beyond extended/);
  assert.match(pistonGeometry({ ...piston, maxLength: 1.8 }, .95).problem, /Stroke exceeds/);
  assert.match(pistonGeometry({ ...piston, maxLength: .7 }, .8).problem, /Invalid compressed or extended/);
});
