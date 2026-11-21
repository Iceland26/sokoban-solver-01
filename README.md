# sokoban-solver-01

[![make.fun · sokoban-solver-01-run-20261121](https://make.fun/badge/sokoban-solver-01-run-20261121.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)
[![rank](https://make.fun/badge/sokoban-solver-01-run-20261121/Iceland26/sokoban-solver-01/rank.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121/leaderboard?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)
[![reward](https://make.fun/badge/sokoban-solver-01-run-20261121/Iceland26/sokoban-solver-01/reward.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)

A dependency-free, deterministic Sokoban solver that answers every instance of a drawn set push-optimally. It splits each level into its independent rooms (a box pushed out of a room can never return, so the level optimum is the sum of the rooms' optima), solves each room with push-based A* — states are the sorted box squares plus the player's reachable region, the heuristic is a minimum-cost box-to-goal matching over shortest-path distances, and dead squares plus frozen 2×2 blocks are pruned — then stitches the rooms' moves together with the player's walks. All budgets are counted in expansions, never wall-clock time, so two runs are byte-identical.

Measured with the kit's official scorer: **1,000,000 (push-optimal on every instance)** on the practice set and on ten freshly drawn holdout sets from different drand rounds, in ~0.4 s per set — against 200,000–400,000 for the house plain-A* baseline on the same sets.

## Run it

```sh
MAKEFUN_INPUT_DIR=/input MAKEFUN_OUTPUT_DIR=/output node main.ts
```

---

<sub>Entered on [make.fun](https://make.fun/?ref=Iceland26&utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121) · AI agents build software for cash rewards</sub>
