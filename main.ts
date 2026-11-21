// Sokoban push-optimal solver.
// Strategy: split the board into single-door rooms, solve each room with
// push-level A* guided by a box-to-goal assignment heuristic built from
// reverse-pull push distances, then stitch room solutions together with
// hallway walks. Falls back to whole-board search when no clean split exists.
// Fully deterministic: budgets count expanded nodes, never wall time.

import fs from 'node:fs';
import path from 'node:path';

const DIRS = [
  { dr: -1, dc: 0, ch: 'u' },
  { dr: 1, dc: 0, ch: 'd' },
  { dr: 0, dc: -1, ch: 'l' },
  { dr: 0, dc: 1, ch: 'r' },
];

// ---------- level parsing ----------

function parseLevel(text) {
  const rawRows = text.split(/\r?\n/);
  while (rawRows.length > 0 && rawRows[rawRows.length - 1] === '') rawRows.pop();
  const h = rawRows.length;
  let w = 0;
  for (const row of rawRows) if (row.length > w) w = row.length;
  const wall = new Uint8Array(w * h).fill(1);
  const goal = new Uint8Array(w * h);
  const boxes = [];
  let player = -1;
  for (let r = 0; r < h; r++) {
    const row = rawRows[r];
    for (let c = 0; c < w; c++) {
      const ch = c < row.length ? row[c] : '#';
      const idx = r * w + c;
      if (ch === '#') continue;
      wall[idx] = 0;
      if (ch === '.' || ch === '*' || ch === '+') goal[idx] = 1;
      if (ch === '$' || ch === '*') boxes.push(idx);
      if (ch === '@' || ch === '+') player = idx;
    }
  }
  boxes.sort((a, b) => a - b);
  const goals = [];
  for (let i = 0; i < w * h; i++) if (goal[i]) goals.push(i);
  return { w, h, wall, goal, boxes, goals, player };
}

// ---------- reverse-pull distances ----------

// For one goal: dist[cell] = fewest pushes to bring a box from cell to goal,
// ignoring other boxes. A pull from cell X in direction d needs the cell
// behind X (opposite d) free for the player to stand on.
function pullDistances(lvl, goalIdx) {
  const { w, h, wall } = lvl;
  const dist = new Int32Array(w * h).fill(-1);
  const queue = [goalIdx];
  dist[goalIdx] = 0;
  let head = 0;
  while (head < queue.length) {
    const cur = queue[head++];
    const r = (cur / w) | 0;
    const c = cur % w;
    for (const d of DIRS) {
      // box moves from prev into cur along d; player stood at prev - d
      const pr = r - d.dr;
      const pc = c - d.dc;
      const sr = pr - d.dr;
      const sc = pc - d.dc;
      if (pr < 0 || pr >= h || pc < 0 || pc >= w) continue;
      if (sr < 0 || sr >= h || sc < 0 || sc >= w) continue;
      const prev = pr * w + pc;
      const stand = sr * w + sc;
      if (wall[prev] || wall[stand]) continue;
      if (dist[prev] !== -1) continue;
      dist[prev] = dist[cur] + 1;
      queue.push(prev);
    }
  }
  return dist;
}

// ---------- assignment heuristic ----------

function matchingCost(cost, n) {
  // cost[i][j]; exact min-cost perfect matching. DP over subsets for n<=10,
  // Hungarian otherwise. Returns Infinity when no finite matching exists.
  if (n === 0) return 0;
  if (n <= 10) {
    const size = 1 << n;
    const dp = new Float64Array(size).fill(Infinity);
    dp[0] = 0;
    for (let mask = 0; mask < size; mask++) {
      const i = popcount(mask);
      if (i >= n || dp[mask] === Infinity) continue;
      for (let j = 0; j < n; j++) {
        if (mask & (1 << j)) continue;
        const v = cost[i][j];
        if (v === Infinity) continue;
        const nm = mask | (1 << j);
        if (dp[mask] + v < dp[nm]) dp[nm] = dp[mask] + v;
      }
    }
    return dp[size - 1];
  }
  return hungarian(cost, n);
}

function popcount(x) {
  let k = 0;
  while (x) { x &= x - 1; k++; }
  return k;
}

function hungarian(cost, n) {
  const BIG = 1e9;
  const u = new Float64Array(n + 1);
  const v = new Float64Array(n + 1);
  const p = new Int32Array(n + 1);
  const way = new Int32Array(n + 1);
  for (let i = 1; i <= n; i++) {
    p[0] = i;
    let j0 = 0;
    const minv = new Float64Array(n + 1).fill(Infinity);
    const used = new Uint8Array(n + 1);
    do {
      used[j0] = 1;
      const i0 = p[j0];
      let delta = Infinity;
      let j1 = -1;
      for (let j = 1; j <= n; j++) {
        if (used[j]) continue;
        const cij = cost[i0 - 1][j - 1] === Infinity ? BIG : cost[i0 - 1][j - 1];
        const cur = cij - u[i0] - v[j];
        if (cur < minv[j]) { minv[j] = cur; way[j] = j0; }
        if (minv[j] < delta) { delta = minv[j]; j1 = j; }
      }
      for (let j = 0; j <= n; j++) {
        if (used[j]) { u[p[j]] += delta; v[j] -= delta; } else minv[j] -= delta;
      }
      j0 = j1;
    } while (p[j0] !== 0);
    do {
      const j1 = way[j0];
      p[j0] = p[j1];
      j0 = j1;
    } while (j0);
  }
  let total = 0;
  for (let j = 1; j <= n; j++) {
    if (!p[j]) continue;
    const cij = cost[p[j] - 1][j - 1];
    if (cij === Infinity) return Infinity;
    total += cij;
  }
  return total;
}

// ---------- push-level A* over one region ----------

// region: Uint8Array mask of cells the player/boxes may occupy.
// boxes/goals: arrays of cell indices inside the region.
// Returns { moves, pushes, expanded } or null.
function solveRegion(lvl, region, startBoxes, regionGoals, startPlayer, cap, greedy) {
  const { w, h, wall, goal } = lvl;
  const n = startBoxes.length;
  if (n === 0) return { moves: '', pushes: 0, expanded: 0 };
  if (regionGoals.length !== n) return null;

  // Per-goal push distances restricted to the region.
  const dists = regionGoals.map((g) => pullDistancesRegion(lvl, region, g));
  const dead = new Uint8Array(w * h);
  for (let i = 0; i < w * h; i++) {
    if (wall[i] || goal[i]) continue;
    let ok = false;
    for (const d of dists) if (d[i] !== -1) { ok = true; break; }
    if (!ok) dead[i] = 1;
  }

  const hMemo = new Map();
  const heuristic = (boxes) => {
    const memoKey = boxes.join(',');
    const hit = hMemo.get(memoKey);
    if (hit !== undefined) return hit;
    const cost = [];
    for (let i = 0; i < n; i++) {
      const row = [];
      for (let j = 0; j < n; j++) {
        const d = dists[j][boxes[i]];
        row.push(d === -1 ? Infinity : d);
      }
      cost.push(row);
    }
    const value = matchingCost(cost, n);
    hMemo.set(memoKey, value);
    return value;
  };

  // 2x2 freeze: a solid block of walls/boxes with a box off-goal can never move.
  const hasFreeze = (boxes) => {
    const set = new Set(boxes);
    for (const b of boxes) {
      if (goal[b]) continue;
      const r = (b / w) | 0;
      const c = b % w;
      for (const [or, oc] of [[0, 0], [0, -1], [-1, 0], [-1, -1]]) {
        const r0 = r + or;
        const c0 = c + oc;
        if (r0 < 0 || c0 < 0 || r0 + 1 >= h || c0 + 1 >= w) continue;
        let solid = true;
        for (let dr = 0; dr < 2 && solid; dr++) {
          for (let dc = 0; dc < 2; dc++) {
            const idx = (r0 + dr) * w + (c0 + dc);
            if (!wall[idx] && !set.has(idx)) { solid = false; break; }
          }
        }
        if (solid) return true;
      }
    }
    return false;
  };

  const startH = heuristic(startBoxes);
  if (startH === Infinity) return null;

  const nodes = []; // {boxes, player, g, parent, seg}
  const best = new Map(); // stateKey -> best g seen
  const heap = new MinHeap();
  let seq = 0;

  const pushNode = (boxes, player, g, parent, seg) => {
    const norm = normalizePlayer(lvl, region, boxes, player);
    const key = boxes.join(',') + '|' + norm;
    const prevBest = best.get(key);
    if (prevBest !== undefined && prevBest <= g) return -1;
    best.set(key, g);
    if (hasFreeze(boxes)) return -1;
    const hh = heuristic(boxes);
    if (hh === Infinity) return -1;
    const id = nodes.length;
    nodes.push({ boxes, player, g, parent, seg });
    const f = greedy ? hh : g + hh;
    heap.push(f, hh, seq++, id);
    return id;
  };

  pushNode(startBoxes.slice(), startPlayer, 0, -1, '');
  let expanded = 0;

  while (heap.size > 0) {
    const id = heap.pop();
    const node = nodes[id];
    // stale check: a cheaper path to this state may have superseded it
    const norm = normalizePlayer(lvl, region, node.boxes, node.player);
    const key = node.boxes.join(',') + '|' + norm;
    if (best.get(key) < node.g) continue;

    if (isSolved(node.boxes, goal)) {
      return { moves: rebuildMoves(nodes, id), pushes: node.g, expanded };
    }
    if (expanded >= cap) return null;
    expanded++;

    const reach = floodPlayer(lvl, region, node.boxes, node.player);
    const boxSet = new Set(node.boxes);
    for (let bi = 0; bi < n; bi++) {
      const b = node.boxes[bi];
      const br = (b / w) | 0;
      const bc = b % w;
      for (const d of DIRS) {
        const destR = br + d.dr;
        const destC = bc + d.dc;
        const standR = br - d.dr;
        const standC = bc - d.dc;
        if (destR < 0 || destR >= h || destC < 0 || destC >= w) continue;
        if (standR < 0 || standR >= h || standC < 0 || standC >= w) continue;
        const dest = destR * w + destC;
        const stand = standR * w + standC;
        if (!region[dest] || wall[dest] || boxSet.has(dest)) continue;
        if (dead[dest]) continue;
        if (!reach.prev.has(stand)) continue;
        const walk = walkMoves(reach, stand, w);
        const newBoxes = node.boxes.slice();
        newBoxes[bi] = dest;
        newBoxes.sort((a, b2) => a - b2);
        pushNode(newBoxes, b, node.g + 1, id, walk + d.ch);
      }
    }
  }
  return null;
}

function pullDistancesRegion(lvl, region, goalIdx) {
  const { w, h, wall } = lvl;
  const dist = new Int32Array(w * h).fill(-1);
  if (!region[goalIdx]) return dist;
  const queue = [goalIdx];
  dist[goalIdx] = 0;
  let head = 0;
  while (head < queue.length) {
    const cur = queue[head++];
    const r = (cur / w) | 0;
    const c = cur % w;
    for (const d of DIRS) {
      const pr = r - d.dr;
      const pc = c - d.dc;
      const sr = pr - d.dr;
      const sc = pc - d.dc;
      if (pr < 0 || pr >= h || pc < 0 || pc >= w) continue;
      if (sr < 0 || sr >= h || sc < 0 || sc >= w) continue;
      const prev = pr * w + pc;
      const stand = sr * w + sc;
      if (!region[prev] || !region[stand] || wall[prev] || wall[stand]) continue;
      if (dist[prev] !== -1) continue;
      dist[prev] = dist[cur] + 1;
      queue.push(prev);
    }
  }
  return dist;
}

function isSolved(boxes, goal) {
  for (const b of boxes) if (!goal[b]) return false;
  return true;
}

function floodPlayer(lvl, region, boxes, player) {
  const { w, h, wall } = lvl;
  const boxSet = new Set(boxes);
  const prev = new Map();
  const queue = [player];
  prev.set(player, -1);
  let head = 0;
  while (head < queue.length) {
    const cur = queue[head++];
    const r = (cur / w) | 0;
    const c = cur % w;
    for (const d of DIRS) {
      const nr = r + d.dr;
      const nc = c + d.dc;
      if (nr < 0 || nr >= h || nc < 0 || nc >= w) continue;
      const ni = nr * w + nc;
      if (!region[ni] || wall[ni] || boxSet.has(ni) || prev.has(ni)) continue;
      prev.set(ni, cur);
      queue.push(ni);
    }
  }
  return { prev };
}

function normalizePlayer(lvl, region, boxes, player) {
  const { prev } = floodPlayer(lvl, region, boxes, player);
  let min = player;
  for (const k of prev.keys()) if (k < min) min = k;
  return min;
}

function walkMoves(reach, target, w) {
  const chars = [];
  let cur = target;
  while (reach.prev.get(cur) !== -1 && reach.prev.get(cur) !== undefined) {
    const p = reach.prev.get(cur);
    const dr = ((cur / w) | 0) - ((p / w) | 0);
    const dc = (cur % w) - (p % w);
    chars.push(dr === -1 ? 'u' : dr === 1 ? 'd' : dc === -1 ? 'l' : 'r');
    cur = p;
  }
  chars.reverse();
  return chars.join('');
}

function rebuildMoves(nodes, id) {
  const parts = [];
  let cur = id;
  while (cur !== -1) {
    parts.push(nodes[cur].seg);
    cur = nodes[cur].parent;
  }
  parts.reverse();
  return parts.join('');
}

class MinHeap {
  constructor() {
    this.f = [];
    this.hh = [];
    this.seq = [];
    this.id = [];
  }
  get size() { return this.id.length; }
  less(a, b) {
    if (this.f[a] !== this.f[b]) return this.f[a] < this.f[b];
    if (this.hh[a] !== this.hh[b]) return this.hh[a] < this.hh[b];
    return this.seq[a] < this.seq[b];
  }
  swap(a, b) {
    for (const arr of [this.f, this.hh, this.seq, this.id]) {
      const t = arr[a]; arr[a] = arr[b]; arr[b] = t;
    }
  }
  push(f, hh, seq, id) {
    this.f.push(f); this.hh.push(hh); this.seq.push(seq); this.id.push(id);
    let i = this.id.length - 1;
    while (i > 0) {
      const parent = (i - 1) >> 1;
      if (!this.less(i, parent)) break;
      this.swap(i, parent);
      i = parent;
    }
  }
  pop() {
    const top = this.id[0];
    const last = this.id.length - 1;
    this.swap(0, last);
    this.f.pop(); this.hh.pop(); this.seq.pop(); this.id.pop();
    let i = 0;
    const n = this.id.length;
    for (;;) {
      const l = 2 * i + 1;
      const r = 2 * i + 2;
      let m = i;
      if (l < n && this.less(l, m)) m = l;
      if (r < n && this.less(r, m)) m = r;
      if (m === i) break;
      this.swap(i, m);
      i = m;
    }
    return top;
  }
}

// ---------- room decomposition ----------

// Find single cells whose removal cuts off a box/goal-balanced component.
// Returns list of rooms {door, cells:Set, boxes, goals} or null.
function findRooms(lvl) {
  const { w, h, wall, goal, boxes, player } = lvl;
  const boxSet = new Set(boxes);
  const floorCells = [];
  for (let i = 0; i < w * h; i++) if (!wall[i]) floorCells.push(i);

  const candidates = [];
  for (const door of floorCells) {
    if (door === player || boxSet.has(door) || goal[door]) continue;
    const comps = componentsWithout(lvl, door);
    for (const comp of comps) {
      if (comp.has(player)) continue;
      let nb = 0;
      let ng = 0;
      for (const cell of comp) {
        if (boxSet.has(cell)) nb++;
        if (goal[cell]) ng++;
      }
      if (nb > 0 && nb === ng) {
        candidates.push({ door, cells: comp, nb });
      }
    }
  }
  // Smallest components first: real rooms are the tightest balanced cuts.
  candidates.sort((a, b) => a.cells.size - b.cells.size || a.door - b.door);
  const claimed = new Set();
  const rooms = [];
  for (const cand of candidates) {
    let overlap = false;
    for (const cell of cand.cells) if (claimed.has(cell)) { overlap = true; break; }
    if (overlap) continue;
    for (const cell of cand.cells) claimed.add(cell);
    const roomBoxes = boxes.filter((b) => cand.cells.has(b));
    const roomGoals = [];
    for (const cell of cand.cells) if (goal[cell]) roomGoals.push(cell);
    roomGoals.sort((a, b) => a - b);
    rooms.push({ door: cand.door, cells: cand.cells, boxes: roomBoxes, goals: roomGoals });
  }
  if (rooms.length === 0) return null;
  // Validate: every box and goal claimed exactly once.
  if (claimed.size === 0) return null;
  for (const b of boxes) if (!claimed.has(b)) return null;
  for (let i = 0; i < w * h; i++) if (goal[i] && !claimed.has(i)) return null;
  rooms.sort((a, b) => a.door - b.door);
  return rooms;
}

function componentsWithout(lvl, removed) {
  const { w, h, wall } = lvl;
  const seen = new Uint8Array(w * h);
  const comps = [];
  for (let i = 0; i < w * h; i++) {
    if (wall[i] || i === removed || seen[i]) continue;
    const comp = new Set();
    const queue = [i];
    seen[i] = 1;
    let head = 0;
    while (head < queue.length) {
      const cur = queue[head++];
      comp.add(cur);
      const r = (cur / w) | 0;
      const c = cur % w;
      for (const d of DIRS) {
        const nr = r + d.dr;
        const nc = c + d.dc;
        if (nr < 0 || nr >= h || nc < 0 || nc >= w) continue;
        const ni = nr * w + nc;
        if (wall[ni] || ni === removed || seen[ni]) continue;
        seen[ni] = 1;
        queue.push(ni);
      }
    }
    comps.push(comp);
  }
  return comps;
}

// Walk (no pushes) from `from` to `to` on the full board, boxes blocked.
// Returns move string or null.
function walkBetween(lvl, boxes, from, to) {
  const region = new Uint8Array(lvl.w * lvl.h).fill(1);
  const reach = floodPlayer(lvl, region, boxes, from);
  if (!reach.prev.has(to)) return null;
  return walkMoves(reach, to, lvl.w);
}

// ---------- whole-level solve ----------

function solveLevel(lvl) {
  const { w, h } = lvl;
  const fullRegion = new Uint8Array(w * h).fill(1);
  const rooms = findRooms(lvl);

  if (rooms) {
    let moves = '';
    let curBoxes = lvl.boxes.slice();
    let curPlayer = lvl.player;
    let ok = true;
    let totalExpanded = 0;
    for (const room of rooms) {
      const region = new Uint8Array(w * h);
      for (const cell of room.cells) region[cell] = 1;
      region[room.door] = 1;
      const walkIn = walkBetween(lvl, curBoxes, curPlayer, room.door);
      if (walkIn === null) { ok = false; break; }
      moves += walkIn;
      curPlayer = room.door;
      const roomBoxes = curBoxes.filter((b) => room.cells.has(b));
      let res = solveRegion(lvl, region, roomBoxes, room.goals, curPlayer, 3000000, false);
      if (!res) res = solveRegion(lvl, region, roomBoxes, room.goals, curPlayer, 1500000, true);
      if (!res) { ok = false; break; }
      totalExpanded += res.expanded;
      moves += res.moves;
      // Apply the room solution to track box/player positions.
      const after = applyMoves(lvl, curBoxes, curPlayer, res.moves);
      if (!after) { ok = false; break; }
      curBoxes = after.boxes;
      curPlayer = after.player;
    }
    if (ok && isSolved(curBoxes, lvl.goal)) {
      return { moves, expanded: totalExpanded, rooms: rooms.length };
    }
  }

  // Whole-board fallback.
  let res = solveRegion(lvl, fullRegion, lvl.boxes, lvl.goals, lvl.player, 3000000, false);
  if (!res) res = solveRegion(lvl, fullRegion, lvl.boxes, lvl.goals, lvl.player, 1500000, true);
  if (res) return { moves: res.moves, expanded: res.expanded, rooms: 0 };
  return null;
}

// ---------- replay / verification ----------

function applyMoves(lvl, startBoxes, startPlayer, moves) {
  const { w, h, wall } = lvl;
  const boxes = new Set(startBoxes);
  let player = startPlayer;
  let pushes = 0;
  for (const chRaw of moves) {
    const ch = chRaw.toLowerCase();
    const d = DIRS.find((x) => x.ch === ch);
    if (!d) {
      if (ch === ' ' || ch === '\n' || ch === '\t' || ch === '\r') continue;
      return null;
    }
    const r = (player / w) | 0;
    const c = player % w;
    const nr = r + d.dr;
    const nc = c + d.dc;
    if (nr < 0 || nr >= h || nc < 0 || nc >= w) return null;
    const ni = nr * w + nc;
    if (wall[ni]) return null;
    if (boxes.has(ni)) {
      const br = nr + d.dr;
      const bc = nc + d.dc;
      if (br < 0 || br >= h || bc < 0 || bc >= w) return null;
      const bi = br * w + bc;
      if (wall[bi] || boxes.has(bi)) return null;
      boxes.delete(ni);
      boxes.add(bi);
      pushes++;
    }
    player = ni;
  }
  return { boxes: [...boxes].sort((a, b) => a - b), player, pushes };
}

function verifySolution(lvl, moves) {
  const after = applyMoves(lvl, lvl.boxes, lvl.player, moves);
  if (!after) return null;
  if (!isSolved(after.boxes, lvl.goal)) return null;
  return after.pushes;
}

// ---------- driver ----------

function main() {
  const inputDir = process.env.MAKEFUN_INPUT_DIR || '/input';
  const outputDir = process.env.MAKEFUN_OUTPUT_DIR || '/output';
  let entries = [];
  try {
    entries = fs.readdirSync(inputDir, { withFileTypes: true })
      .filter((e) => e.isDirectory())
      .map((e) => e.name)
      .sort();
  } catch {
    entries = [];
  }
  for (const id of entries) {
    const outPath = path.join(outputDir, id);
    let solution = '';
    try {
      const levelPath = path.join(inputDir, id, 'level.txt');
      const text = fs.readFileSync(levelPath, 'utf8');
      const lvl = parseLevel(text);
      const res = solveLevel(lvl);
      if (res && verifySolution(lvl, res.moves) !== null) {
        solution = res.moves;
      }
    } catch {
      solution = '';
    }
    try {
      fs.mkdirSync(outPath, { recursive: true });
      fs.writeFileSync(path.join(outPath, 'solution.txt'), solution, 'utf8');
    } catch {
      // Nothing else we can do for this instance; keep going.
    }
  }
}

main();
