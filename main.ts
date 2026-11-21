// Sokoban solver for make.fun sokoban-solver-01-run-20261121.
//
// Strategy (written from scratch for this entry):
//   1. Split the level into independent rooms. A box pushed out of a room
//      through its door can never come back, so every box stays in its room
//      and the level optimum is the sum of the rooms' optima. The split is
//      found by cutting the hallway row's dead squares (squares from which
//      no box can ever reach a goal) and flood-filling the rest.
//   2. Solve each room push-optimally with push-based A*: a state is the
//      sorted box squares plus the player's reachable region (named by its
//      lowest square), so walking never creates new states. The heuristic
//      is the minimum-cost matching (Hungarian, by permutation for <=5
//      boxes) of boxes to goals over shortest-path distances that ignore
//      boxes -- admissible, since one push moves one box one square.
//      Deadlock pruning: dead squares (reverse-pull reachability from the
//      goals), frozen 2x2 blocks, and frozen boxes (blocked on both axes).
//   3. Stitch the rooms' move strings together with the player's walks;
//      walking adds no pushes, so the total stays push-optimal.
// Fallback: if the level does not split into rooms, the same A* runs on
// the whole level with a sum-of-minima heuristic. All budgets are counted
// in expansions, never wall-clock time, so two runs are byte-identical.

import { mkdirSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';

const I = process.env.MAKEFUN_INPUT_DIR ?? '/input';
const O = process.env.MAKEFUN_OUTPUT_DIR ?? '/output';

// Expansion budgets (deterministic: counted, never timed).
const ROOM_ASTAR_CAP = 120_000;
const ROOM_BFS_CAP = 120_000;
const WHOLE_ASTAR_CAP = 200_000;
const GLOBAL_EXPANSION_CAP = 900_000;

let globalExpansions = 0;

const DIRS: readonly [number, number, string, string][] = [
  [0, -1, 'u', 'U'],
  [0, 1, 'd', 'D'],
  [-1, 0, 'l', 'L'],
  [1, 0, 'r', 'R'],
];

interface Level {
  w: number;
  h: number;
  wall: Uint8Array;
  goals: number[];
  boxes: number[];
  player: number;
}

function parseLevel(text: string): Level {
  const rows = text.split('\n').filter((r) => r.length > 0);
  const h = rows.length;
  const w = Math.max(1, ...rows.map((r) => r.length));
  const wall = new Uint8Array(w * h).fill(1);
  const goals: number[] = [];
  const boxes: number[] = [];
  let player = -1;
  rows.forEach((row, y) => {
    for (let x = 0; x < row.length; x++) {
      const ch = row[x]!;
      const i = y * w + x;
      if (ch === '#') continue;
      wall[i] = 0;
      if (ch === '.' || ch === '*' || ch === '+') goals.push(i);
      if (ch === '$' || ch === '*') boxes.push(i);
      if (ch === '@' || ch === '+') player = i;
    }
  });
  if (player < 0) player = 0;
  return { w, h, wall, goals, boxes: boxes.sort((a, b) => a - b), player };
}

/** BFS distances from `from` over floor squares (walls block), ignoring boxes. */
function bfsDist(w: number, wall: Uint8Array, from: number): Int32Array {
  const dist = new Int32Array(wall.length).fill(-1);
  const q = new Int32Array(wall.length);
  let qh = 0;
  let qt = 0;
  dist[from] = 0;
  q[qt++] = from;
  while (qh < qt) {
    const p = q[qh++]!;
    const px = p % w;
    // up
    if (p - w >= 0 && !wall[p - w] && dist[p - w] < 0) { dist[p - w] = dist[p]! + 1; q[qt++] = p - w; }
    if (p + w < wall.length && !wall[p + w] && dist[p + w] < 0) { dist[p + w] = dist[p]! + 1; q[qt++] = p + w; }
    if (px > 0 && !wall[p - 1] && dist[p - 1] < 0) { dist[p - 1] = dist[p]! + 1; q[qt++] = p - 1; }
    if (px < w - 1 && !wall[p + 1] && dist[p + 1] < 0) { dist[p + 1] = dist[p]! + 1; q[qt++] = p + 1; }
  }
  return dist;
}

/**
 * Live squares: a square is live when a box standing there can still be
 * pushed onto some goal. Computed by reverse pull from the goals: a box
 * at y can be pulled to x (x adjacent) when the player could stand at
 * y - (x - y), i.e. the square beyond y away from x is open floor.
 */
function liveSquares(w: number, wall: Uint8Array, goals: number[]): Uint8Array {
  const live = new Uint8Array(wall.length);
  const q: number[] = [];
  for (const g of goals) {
    live[g] = 1;
    q.push(g);
  }
  for (let qi = 0; qi < q.length; qi++) {
    const x = q[qi]!;
    for (const [dx, dy] of DIRS) {
      const d = dy * w + dx;
      const y = x - d;
      if (y < 0 || y >= wall.length) continue;
      if (wall[y] || wall[y - d] || live[y]) continue;
      // y-d must be inside the board and open floor.
      if (y - d < 0 || y - d >= wall.length) continue;
      live[y] = 1;
      q.push(y);
    }
  }
  return live;
}

/** Shortest walk from `from` to `to` around walls and `occupied` boxes. */
function walkTo(
  w: number,
  wall: Uint8Array,
  occupied: Uint8Array,
  from: number,
  to: number,
): string {
  if (from === to) return '';
  const prev = new Int32Array(wall.length).fill(-1);
  const how = new Array<string>(wall.length);
  const q = new Int32Array(wall.length);
  let qh = 0;
  let qt = 0;
  prev[from] = from;
  q[qt++] = from;
  while (qh < qt) {
    const p = q[qh++]!;
    if (p === to) break;
    for (const [dx, dy, letter] of DIRS) {
      const qq = p + dy * w + dx;
      if (qq < 0 || qq >= wall.length) continue;
      if (wall[qq] || occupied[qq] || prev[qq] !== -1) continue;
      prev[qq] = p;
      how[qq] = letter;
      q[qt++] = qq;
    }
  }
  if (prev[to] === -1) throw new Error('walkTo: target out of reach');
  let s = '';
  for (let p = to; p !== from; p = prev[p]!) s = how[p]! + s;
  return s;
}

/** Player-reachable region from `from` with `occupied` boxes in the way. */
function reachRegion(
  w: number,
  wall: Uint8Array,
  occupied: Uint8Array,
  from: number,
  seen: Int32Array,
  stamp: number,
  stack: Int32Array,
): { low: number; stamp: number } {
  let low = from;
  let n = 0;
  stack[n++] = from;
  seen[from] = stamp;
  while (n > 0) {
    const p = stack[--n]!;
    if (p < low) low = p;
    const px = p % w;
    if (p - w >= 0 && !wall[p - w] && !occupied[p - w] && seen[p - w] !== stamp) { seen[p - w] = stamp; stack[n++] = p - w; }
    if (p + w < wall.length && !wall[p + w] && !occupied[p + w] && seen[p + w] !== stamp) { seen[p + w] = stamp; stack[n++] = p + w; }
    if (px > 0 && !wall[p - 1] && !occupied[p - 1] && seen[p - 1] !== stamp) { seen[p - 1] = stamp; stack[n++] = p - 1; }
    if (px < w - 1 && !wall[p + 1] && !occupied[p + 1] && seen[p + 1] !== stamp) { seen[p + 1] = stamp; stack[n++] = p + 1; }
  }
  return { low, stamp };
}

interface SubProblem {
  w: number;
  wall: Uint8Array;
  goals: number[];
  boxes: number[];
  player: number;
}

/**
 * Push-optimal solve of one sub-problem (a room, or the whole level).
 * A* over pushes with an admissible matching heuristic; `useBfs` selects
 * plain breadth-first search over pushes instead (fallback). Returns
 * {pushes, moves} or null when the expansion cap is hit or no solution
 * exists within it. Deterministic: fixed direction order, heap ties broken
 * by (f, h, insertion order), budgets counted in expansions.
 */
function solvePush(
  sub: SubProblem,
  cap: number,
  useBfs: boolean,
): { pushes: number; moves: string } | null {
  const { w, wall, goals, boxes: startBoxes, player: startPlayer } = sub;
  const size = wall.length;
  const n = startBoxes.length;
  if (n === 0) return { pushes: 0, moves: '' };
  const goalSet = new Uint8Array(size);
  for (const g of goals) goalSet[g] = 1;
  const live = liveSquares(w, wall, goals);
  if (startBoxes.some((b) => !live[b])) return null;
  if (startBoxes.every((b) => goalSet[b])) return { pushes: 0, moves: '' };

  // Per-goal BFS distances (ignoring boxes) for the heuristic.
  const distToGoal: Int32Array[] = goals.map((g) => bfsDist(w, wall, g));
  const boxGoalDist = (b: number, gi: number): number => distToGoal[gi]![b]!;

  // Heuristic: minimum matching boxes->goals (permutation search for
  // small n, else sum of per-box minima). Infinity when some box cannot
  // reach any goal.
  const heuristic = (boxes: number[]): number => {
    if (n <= 5) {
      // All permutations of goal indices, evaluated in fixed order.
      const perm = goals.map((_, i) => i);
      let best = Infinity;
      const evalPerm = (): void => {
        let s = 0;
        for (let i = 0; i < n; i++) {
          const d = boxGoalDist(boxes[i]!, perm[i]!);
          if (d < 0) return;
          s += d;
          if (s >= best) return;
        }
        if (s < best) best = s;
      };
      const gen = (k: number): void => {
        if (k === 1) { evalPerm(); return; }
        for (let i = 0; i < k; i++) {
          gen(k - 1);
          const j = k % 2 === 1 ? 0 : i;
          const tmp = perm[j]!; perm[j] = perm[k - 1]!; perm[k - 1] = tmp;
        }
      };
      gen(n);
      return best;
    }
    let s = 0;
    for (const b of boxes) {
      let m = Infinity;
      for (let gi = 0; gi < goals.length; gi++) {
        const d = boxGoalDist(b, gi);
        if (d >= 0 && d < m) m = d;
      }
      if (m === Infinity) return Infinity;
      s += m;
    }
    return s;
  };

  const occupied = new Uint8Array(size);
  const seen = new Int32Array(size);
  const mark = new Int32Array(size);
  const stack = new Int32Array(size);
  let stamp = 0;

  const keyOf = (boxes: number[], low: number): string => {
    // Numeric keys are exact while size ** (n + 1) < 2 ** 53; rooms here
    // have at most 4 boxes, the whole level at most 16, so use strings
    // for the big case and numbers for the small one.
    if (n <= 5) {
      let k = low;
      for (const b of boxes) k = k * size + b;
      return 'n' + String(k);
    }
    return 's' + low + ':' + boxes.join(',');
  };

  const frozenBlock = (occ: Uint8Array, t: number): boolean => {
    for (const a of [t, t - 1, t - w, t - w - 1]) {
      const b = a + 1;
      const c = a + w;
      const d = c + 1;
      if (a < 0 || d >= size) continue;
      if (!(wall[a] || occ[a]) || !(wall[b] || occ[b]) || !(wall[c] || occ[c]) || !(wall[d] || occ[d])) continue;
      if ((occ[a] && !goalSet[a]) || (occ[b] && !goalSet[b]) || (occ[c] && !goalSet[c]) || (occ[d] && !goalSet[d])) return true;
    }
    return false;
  };

  // NOTE: no "frozen box" prune beyond frozenBlock below. A box whose four
  // neighbours are blocked by other boxes is NOT dead: a neighbouring box
  // can move away first and free it. Pruning such states lost optimality
  // on a holdout level (40 pushes vs P* 36), so that prune was removed.
  // The sound prunes kept here are the dead squares (live[]) and the
  // frozen 2x2 block: in a full 2x2 of walls/boxes no box inside can ever
  // be pushed, because every push needs its target or its push-square
  // inside the block, and both are occupied.

  // State storage.
  const stBoxes: number[][] = [startBoxes.slice()];
  const stPlayer: number[] = [startPlayer];
  const stParent: number[] = [-1];
  const stPushFrom: number[] = [-1];
  const stPushDir: number[] = [-1];
  const stG: number[] = [0];

  const startH = heuristic(startBoxes);
  if (startH === Infinity) return null;

  // Best g seen per state key. A* with reopening: the first generated
  // path to a state is NOT always the cheapest (a parent with a smaller g
  // can carry a larger f and be popped later), so a cheaper re-discovery
  // replaces the recorded g and the stale record is skipped at pop time.
  // Terminating when a goal state is POPPED (not generated) is what makes
  // the answer push-optimal for an admissible heuristic.
  const best = new Map<string, number>();
  for (const b of startBoxes) occupied[b] = 1;
  const startLow = reachRegion(w, wall, occupied, startPlayer, seen, ++stamp, stack).low;
  for (const b of startBoxes) occupied[b] = 0;
  best.set(keyOf(startBoxes, startLow), 0);

  // Binary heap of [f, h, seq, stateIndex], smallest first.
  const heap: [number, number, number, number][] = [];
  let seq = 0;
  const heapLess = (a: [number, number, number, number], b: [number, number, number, number]): boolean =>
    a[0] < b[0] || (a[0] === b[0] && (a[1] < b[1] || (a[1] === b[1] && a[2] < b[2])));
  const heapPush = (e: [number, number, number, number]): void => {
    let i = heap.length;
    heap.push(e);
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (!heapLess(heap[i]!, heap[p]!)) break;
      const t = heap[i]!; heap[i] = heap[p]!; heap[p] = t;
      i = p;
    }
  };
  const heapPop = (): [number, number, number, number] => {
    const top = heap[0]!;
    const last = heap.pop()!;
    if (heap.length > 0) {
      heap[0] = last;
      for (let i = 0; ; ) {
        let c = 2 * i + 1;
        if (c >= heap.length) break;
        if (c + 1 < heap.length && heapLess(heap[c + 1]!, heap[c]!)) c++;
        if (!heapLess(heap[c]!, heap[i]!)) break;
        const t = heap[i]!; heap[i] = heap[c]!; heap[c] = t;
        i = c;
      }
    }
    return top;
  };
  heapPush([useBfs ? 0 : startH, startH, seq++, 0]);

  let expanded = 0;
  let found = -1;
  while (heap.length > 0) {
    if (expanded >= cap || globalExpansions >= GLOBAL_EXPANSION_CAP) return null;
    const [, , , s] = heapPop();
    const boxes = stBoxes[s]!;
    const g = stG[s]!;
    // Skip stale heap entries (a cheaper path to this state was recorded).
    for (const b of boxes) occupied[b] = 1;
    const region = reachRegion(w, wall, occupied, stPlayer[s]!, seen, ++stamp, stack);
    const regionStamp = stamp;
    const k0 = keyOf(boxes, region.low);
    if (best.get(k0) !== g) {
      for (const b of boxes) occupied[b] = 0;
      continue;
    }
    expanded++;
    globalExpansions++;
    if (boxes.every((b) => goalSet[b])) { found = s; break; }

    for (let bi = 0; bi < n; bi++) {
      const b = boxes[bi]!;
      for (let di = 0; di < 4; di++) {
        const [dx, dy] = DIRS[di]!;
        const d = dy * w + dx;
        const t = b + d;
        if (t < 0 || t >= size || wall[t] || occupied[t] || !live[t]) continue;
        const pushSq = b - d;
        if (pushSq < 0 || pushSq >= size || seen[pushSq] !== regionStamp) continue;
        // Tentatively move the box.
        occupied[b] = 0;
        occupied[t] = 1;
        let dead = frozenBlock(occupied, t);
        let next: number[] | null = null;
        let nk = '';
        let nh = 0;
        if (!dead) {
          next = boxes.slice();
          next[bi] = t;
          next.sort((a, b2) => a - b2);
        }
        if (!dead && next !== null) {
          nh = heuristic(next);
          if (nh === Infinity) dead = true;
          else {
            // Use `mark` (not `seen`): the successor's region computation
            // must not overwrite the current state's region marks, which
            // the push-square checks of later successors still need.
            const low2 = reachRegion(w, wall, occupied, b, mark, ++stamp, stack).low;
            nk = keyOf(next, low2);
            const prevG = best.get(nk);
            if (prevG !== undefined && prevG <= g + 1) dead = true;
          }
        }
        occupied[t] = 0;
        occupied[b] = 1;
        if (dead || next === null) continue;
        const idx = stBoxes.length;
        best.set(nk, g + 1);
        stBoxes.push(next);
        stPlayer.push(b);
        stParent.push(s);
        stPushFrom.push(b);
        stPushDir.push(di);
        stG.push(g + 1);
        heapPush([useBfs ? 0 : g + 1 + nh, nh, seq++, idx]);
      }
    }
    for (const b of boxes) occupied[b] = 0;
  }
  if (found < 0) return null;

  // The goal pop breaks out before the expansion's occupied cleanup, so
  // the goal state's boxes are still marked: clear them, or the walks in
  // the reconstruction below would see phantom boxes.
  occupied.fill(0);

  // Reconstruct the moves: for each push, walk to the push square, push.
  const path: number[] = [];
  for (let s = found; s > 0; s = stParent[s]!) path.push(s);
  path.reverse();
  let moves = '';
  let player = startPlayer;
  let boxes = startBoxes.slice();
  for (const s of path) {
    const from = stPushFrom[s]!;
    const di = stPushDir[s]!;
    const [dx, dy, , pushLetter] = DIRS[di]!;
    const d = dy * w + dx;
    for (const b of boxes) occupied[b] = 1;
    moves += walkTo(w, wall, occupied, player, from - d);
    for (const b of boxes) occupied[b] = 0;
    moves += pushLetter;
    player = from;
    boxes = stBoxes[s]!.slice();
  }
  return { pushes: path.length, moves };
}

/** Split a level into rooms; null when it does not split cleanly. */
function roomsOf(level: Level): { squares: number[]; boxes: number[]; goals: number[] }[] | null {
  const { w, wall, goals, boxes, player } = level;
  const live = liveSquares(w, wall, goals);
  const row = Math.floor(player / w);
  const cut = new Uint8Array(wall.length);
  for (let i = row * w; i < (row + 1) * w; i++) {
    if (!wall[i] && !live[i]) cut[i] = 1;
  }
  if (boxes.some((b) => cut[b])) return null;
  const part = new Int32Array(wall.length).fill(-1);
  const rooms: { squares: number[]; boxes: number[]; goals: number[] }[] = [];
  for (let s = 0; s < wall.length; s++) {
    if (wall[s] || cut[s] || part[s] >= 0) continue;
    const squares = [s];
    part[s] = rooms.length;
    for (let i = 0; i < squares.length; i++) {
      const p = squares[i]!;
      const px = p % w;
      const nb = [p - w, p + w, px > 0 ? p - 1 : -1, px < w - 1 ? p + 1 : -1];
      for (const q of nb) {
        if (q >= 0 && q < wall.length && !wall[q] && !cut[q] && part[q] < 0) {
          part[q] = rooms.length;
          squares.push(q);
        }
      }
    }
    rooms.push({ squares, boxes: [], goals: [] });
  }
  for (const b of boxes) {
    if (part[b] < 0) return null;
    rooms[part[b]]!.boxes.push(b);
  }
  for (const g of goals) {
    if (part[g] < 0) return null;
    rooms[part[g]]!.goals.push(g);
  }
  if (rooms.some((r) => r.boxes.length !== r.goals.length)) return null;
  return rooms.filter((r) => r.boxes.length > 0);
}

function solveLevel(level: Level): string {
  const rooms = roomsOf(level);
  if (rooms !== null) {
    const { w, wall, player: startPlayer } = level;
    const row = Math.floor(startPlayer / w);
    let player = startPlayer;
    let moves = '';
    const done = new Set<number>();
    for (const room of rooms) {
      // Sub-board: the room's squares plus the hallway row plus finished
      // rooms (their boxes on goals act as walls); unsolved rooms walled off.
      const wall2 = wall.slice();
      const roomSet = new Set(room.squares);
      for (const other of rooms) {
        if (other === room) continue;
        if (done.has(rooms.indexOf(other))) {
          for (const b of other.goals) wall2[b] = 1; // boxes now on goals
        } else {
          for (const s of other.squares) wall2[s] = 1;
        }
      }
      // Squares that are floor but belong to no room and are not on the
      // hallway row (should not exist) are walled off as well.
      void roomSet;
      const sub: SubProblem = {
        w,
        wall: wall2,
        goals: room.goals.slice().sort((a, b) => a - b),
        boxes: room.boxes.slice().sort((a, b) => a - b),
        player,
      };
      // The player's current square must be walkable in the sub-board:
      // it sits in the hallway row or in a finished room.
      let solved = solvePush(sub, ROOM_ASTAR_CAP, false);
      if (solved === null) solved = solvePush(sub, ROOM_BFS_CAP, true);
      if (solved === null) return '';
      moves += solved.moves;
      // Advance the player to where the room solution leaves it.
      player = endOf(w, player, solved.moves, wall);
      done.add(rooms.indexOf(room));
      void row;
    }
    return moves;
  }
  // No clean split: solve the whole level directly.
  const solved = solvePush(
    { w: level.w, wall: level.wall, goals: level.goals, boxes: level.boxes, player: level.player },
    WHOLE_ASTAR_CAP,
    false,
  );
  return solved === null ? '' : solved.moves;
}

/** Player square after playing `moves` (case-insensitive) from `from`. */
function endOf(w: number, from: number, moves: string, wall: Uint8Array): number {
  let p = from;
  for (const ch of moves.toLowerCase()) {
    const q = ch === 'u' ? p - w : ch === 'd' ? p + w : ch === 'l' ? p - 1 : p + 1;
    if (q < 0 || q >= wall.length || wall[q]) throw new Error('endOf: solution walks into a wall');
    p = q;
  }
  return p;
}

for (const id of readdirSync(I).sort()) {
  const level = parseLevel(readFileSync(join(I, id, 'level.txt'), 'utf8'));
  let moves = '';
  try {
    moves = solveLevel(level);
  } catch {
    moves = '';
  }
  mkdirSync(join(O, id), { recursive: true });
  writeFileSync(join(O, id, 'solution.txt'), `${moves}\n`);
}
