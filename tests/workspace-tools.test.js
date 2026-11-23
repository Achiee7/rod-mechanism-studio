import test from 'node:test';
import assert from 'node:assert/strict';
import { PlaybackClock, translateSelection, pathCsv } from '../src/workspace-tools.js';
import { normalizeMechanism } from '../src/mechanism-model.js';

test('playback speed and display frequency only change the number of fixed steps', () => {
  for (const hz of [30, 60, 120]) for (const speed of [0.1, 0.25, 1, 4]) {
    const clock = new PlaybackClock();
    const steps = [];
    for (let i = 0; i <= hz; i++) clock.advance(i * 1000 / hz, speed, {step: dt => steps.push(dt)});
    assert.equal(steps.length, Math.round(speed * 120));
    assert.ok(steps.every(dt => dt === 1 / 120));
    clock.reset();
    clock.advance(10000, 4, {step: () => assert.fail('Unexpected jump after pause')});
  }
});

for (const stop of [{accepted:false}, {accepted:true, settled:true}]) {
  test('playback pauses and resumes without a time jump after ' + (stop.settled ? 'settling' : 'a refused step'), () => {
    const clock = new PlaybackClock();
    let calls = 0;
    const sim = {step: () => {calls++; return stop;}};
    clock.advance(0, 4, sim);
    assert.equal(clock.advance(100, 4, sim), stop);
    assert.equal(calls, 1);
    clock.advance(3000, 4, sim);
    assert.equal(calls, 1);
  });
}

test('group movement preserves internal dimensions and rejects external driving dimensions', () => {
  const model = normalizeMechanism({joints:[{id:1,x:0,y:0},{id:2,x:1,y:0}],
    elements:[{id:1,kind:'rod',a:1,b:2}],dimensions:[{id:1,kind:'distance',a:1,b:2,value:1}]});
  translateSelection(model, [{type:'element',id:1}], {x:0.1,y:0.2});
  assert.equal(model.joints.find(j=>j.id===2).x,1.1);
  assert.equal(model.elements[0].length,1);
  assert.throws(()=>translateSelection(model,[{type:'joint',id:2}],{x:0,y:1}), /outside dimension/);
});

test('sweep CSV preserves signed values and leaves unavailable values empty', () => {
  const model={joints:[{id:1}],elements:[{id:2,kind:'gas_spring'}]};
  const csv=pathCsv({poses:[{angleDeg:-4,feasible:false,positions:{1:{x:0,y:.2}},pistons:{2:{length:.5,force:null,assistForce:-10}}}]},model);
  assert.match(csv,/\r\n-4,true,,false,"",0,200,500,,-10$/);
});
