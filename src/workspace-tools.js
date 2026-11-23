import { solveSketch } from './mechanism-model.js';

const physicsStep = 1 / 120;

// Advance the same sequence of physics steps at every display refresh rate and speed.
export class PlaybackClock {
  reset() {
    this.previous = null;
    this.remainder = 0;
  }
  constructor() { this.reset(); }
  advance(now, speed, simulation) {
    if (this.previous !== null) {
      // Ignore time spent suspended in a browser or OS dialog.
      const elapsed = Math.min(0.1, Math.max(0, (now - this.previous) / 1000));
      this.remainder += elapsed * speed;
      const steps = Math.floor((this.remainder + 1e-12) / physicsStep);
      for (let i = 0; i < steps; i++) {
        const result = simulation.step(physicsStep);
        if (result?.accepted === false || result?.settled) {
          this.reset();
          return result;
        }
      }
      this.remainder -= steps * physicsStep;
    }
    this.previous = now;
  }
}

export function translateSelection(model, items, delta) {
  if (!Number.isFinite(delta.x) || !Number.isFinite(delta.y)) throw Error('Invalid movement.');
  const moving = new Set(items.filter(p => p.type === 'joint').map(p => p.id));
  const explicitJoints = new Set(moving);
  for (const item of items.filter(p => p.type === 'element')) {
    const member = model.elements.find(e => e.id === item.id);
    if (member) { moving.add(member.a); moving.add(member.b); }
  }
  const origin = model.joints.find(j => j.kind === 'origin');
  moving.delete(origin?.id);
  if (!moving.size) throw Error('Select a movable joint or component.');
  if (model.joints.some(j => ['fixed','rotation'].includes(j.kind) && moving.has(j.id) && !explicitJoints.has(j.id))) {
    throw Error('Include the grounded joint in the selection to move its mounting point.');
  }
  const relations = [...model.dimensions.filter(d => d.driving !== false), ...model.constraints];
  for (const relation of relations) {
    const ids = [relation.a,relation.b,relation.c,relation.d].filter(id => id != null);
    if (ids.some(id => moving.has(id)) && ids.some(id => !moving.has(id))) {
      throw Error('An outside dimension or constraint holds the selection. Include its other endpoint or edit the relation.');
    }
  }
  for (const joint of model.joints) {
    if (moving.has(joint.id)) { joint.x += delta.x; joint.y += delta.y; }
  }
  return solveSketch(model);
}

export function pathCsv(path, model) {
  const pistons = model.elements.filter(p => p.kind === 'actuator' || p.kind === 'gas_spring');
  const members = model.elements.filter(p => !pistons.includes(p));
  const joints = model.joints.filter(j => j.kind !== 'origin');
  const header = ['drive_angle_deg', 'geometry_solved', 'within_stroke', 'reachable', 'status'];
  for (const j of joints) header.push('joint_' + j.id + '_x_mm','joint_' + j.id + '_y_mm');
  for (const p of pistons) header.push('piston_' + p.id + '_length_mm','piston_' + p.id + '_hold_N','piston_' + p.id + '_hand_N');
  for (const m of members) header.push('member_' + m.id + '_axial_N');
  const rows = [header];
  const mm = value => Number.isFinite(value) ? value * 1000 : null;
  for (const pose of path.poses || []) {
    const row = [pose.angleDeg, Boolean(pose.positions), pose.withinStroke, Boolean(pose.feasible), pose.status || ''];
    for (const j of joints) {
      const point = pose.positions?.[j.id];
      row.push(mm(point?.x), mm(point?.y));
    }
    for (const p of pistons) {
      const sample = pose.pistons?.[p.id];
      row.push(mm(sample?.length), sample?.force, sample?.assistForce);
    }
    for (const m of members) row.push(pose.memberForces?.[m.id]);
    rows.push(row);
  }
  const escape = v => typeof v === 'string' ? '"' + v.replaceAll('"', '""') + '"' : v == null ? '' : String(v);
  return rows.map(row => row.map(escape).join(',')).join('\r\n');
}
