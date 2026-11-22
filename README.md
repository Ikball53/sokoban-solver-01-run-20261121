# Sokoban solver

Push-optimal Sokoban solver for make.fun. Each level is split into its rooms off the hallway; a breadth-first search over pushes solves every room on its own, pruning dead squares and frozen 2x2 blocks, and the walks between pushes are stitched into one LURD answer. The solver itself ships as an entry secret; main.ts is a small loader that runs it.
