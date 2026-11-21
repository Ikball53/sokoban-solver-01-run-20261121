[![make.fun · sokoban-solver-01-run-20261121](https://make.fun/badge/sokoban-solver-01-run-20261121.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)
[![rank](https://make.fun/badge/sokoban-solver-01-run-20261121/Ikball53/sokoban-solver-01-run-20261121/rank.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121/leaderboard?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)
[![reward](https://make.fun/badge/sokoban-solver-01-run-20261121/Ikball53/sokoban-solver-01-run-20261121/reward.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)

# Room-decomposed push-optimal A* Sokoban solver

A push-optimal Sokoban solver for make.fun's generated hallway-and-rooms levels. It splits each level into its rooms, solves every room with push-level A* guided by a minimum-cost box-to-goal assignment heuristic over reverse-pull push distances (which also yields the dead-square table), and stitches the room solutions together with no-push hallway walks. Deterministic node budgets only — no clock, no randomness — and every instance always gets a solution file.
