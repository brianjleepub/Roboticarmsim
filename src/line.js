// Two-robot transfer line.
//   Robot 1 picks parts off its side of the table and sets them in the shared transfer tray.
//   Robot 2 takes them out of the tray and sets them on the conveyor, which drops them in the bin.
// Only one arm may work over the transfer tray at a time (Zone interlock in motion.js).
import { CONVEYOR } from './scene.js';
import { HOME } from './kinematics.js';

export class Line {
  constructor({ feeder, unloader, parts, zone }) {
    this.feeder = feeder; // robot 1
    this.unloader = unloader; // robot 2
    this.parts = parts;
    this.zone = zone;
    this.running = false;
    this.reserved = new Map(); // part id -> transfer slot promised to it
    this.claimed = new Set(); // parts already assigned to a robot
    this.listeners = {};
    for (const c of [feeder, unloader]) {
      c.on('fault', (text) => { if (this.running) this.halt(`Line stopped. ${text}`); });
    }
  }

  on(ev, fn) { (this.listeners[ev] ||= []).push(fn); }
  emit(ev, ...a) { for (const fn of this.listeners[ev] || []) fn(...a); }

  // Belt runs while the line runs, and keeps running until it is empty.
  get conveyorOn() { return this.running || this.parts.list.some((p) => this.parts.onBelt(p)); }

  start() {
    if (this.running) return;
    this.running = true;
    this.reserved.clear();
    this.claimed.clear();
    this.emit('state', true);
  }

  halt(reason) {
    if (!this.running) return;
    this.running = false;
    this.feeder.stop();
    this.unloader.stop();
    this.reserved.clear();
    this.claimed.clear();
    this.emit('state', false, reason);
  }

  // Parts robot 1 can still feed: resting on the table (not in a tray, not on the belt).
  feedable() {
    return this.parts.list.filter((p) => {
      if (p.state !== 'rest' || this.claimed.has(p.id)) return false;
      if (this.parts.whereIs(p).kind !== 'table') return false;
      const g = this.feeder.graspPose(p);
      return this.feeder.canReach(g.x, g.y, g.zGrip, g.yaw);
    });
  }

  waitingInTransfer() {
    return this.parts.list.filter((p) => p.state === 'rest' && !this.claimed.has(p.id)
      && this.parts.whereIs(p).tray?.id === 'transfer');
  }

  update() {
    if (!this.running) return;
    const { feeder: f, unloader: u, parts, zone } = this;
    if (f.estop || u.estop) return;

    // Robot 1: next part from the table into a free transfer slot
    if (!f.busy) {
      const next = this.feedable().sort((a, b) => a.mesh.position.x - b.mesh.position.x)[0];
      const slot = next && parts.freeSlot('transfer', [...this.reserved.values()]);
      if (next && slot) {
        this.claimed.add(next.id);
        this.reserved.set(next.id, slot);
        f.pickAndPlace(next, { x: slot[0], y: slot[1] }, { placeZone: zone });
        f.enqueue({ type: 'call', fn: () => { this.reserved.delete(next.id); this.claimed.delete(next.id); } });
      } else if (!next && !this.atHome(f)) {
        f.enqueue({ type: 'ptp', q: HOME.slice(), label: 'Home' });
      }
    }

    // Robot 2: transfer tray -> conveyor infeed, once the infeed is clear
    if (!u.busy) {
      const ready = this.waitingInTransfer()[0];
      if (ready && parts.infeedClear()) {
        this.claimed.add(ready.id);
        const [x, y] = CONVEYOR.infeed;
        u.pickAndPlace(ready, { x, y, yaw: -Math.PI / 2 }, { pickZone: zone, home: true });
        u.enqueue({ type: 'call', fn: () => this.claimed.delete(ready.id) });
      }
    }

    // finished: nothing left to feed, nothing in transit, both arms parked
    const left = this.feedable().length + this.waitingInTransfer().length;
    const moving = parts.list.some((p) => p.state === 'held' || p.state === 'falling' || parts.onBelt(p));
    if (!left && !moving && !f.busy && !u.busy) {
      this.running = false;
      const binned = parts.list.filter((p) => parts.whereIs(p).kind === 'bin').length;
      this.emit('state', false, `Line finished: ${binned} part${binned === 1 ? '' : 's'} in the bin`);
    }
  }

  atHome(c) { return c.q.every((v, i) => Math.abs(v - HOME[i]) < 1e-3); }
}
