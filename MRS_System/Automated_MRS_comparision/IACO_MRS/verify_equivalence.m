% verify_equivalence.m
clear; clc;

R_1x1 = 1/sqrt(2);
R_2x2 = sqrt(2);
R_3x3 = sqrt(2) + 1/sqrt(2);   % = 3/sqrt(2) — tu fórmula original ✓
R_4x4 = 2 * sqrt(2);
k_rep = 1.0;

obs(1) = struct('x', 3.0, 'y', 3.0, 'r', R_1x1, 'colour', 'black', 'k_rep', k_rep);
obs(2) = struct('x', 8.5, 'y', 9.5, 'r', R_2x2, 'colour', 'black', 'k_rep', k_rep);
obs(3) = struct('x',12.0, 'y', 15, 'r', R_3x3, 'colour', 'black', 'k_rep', k_rep);
obs(4) = struct('x',16.5, 'y', 1.5, 'r', R_4x4, 'colour', 'black', 'k_rep', k_rep);
m = buildObstacleMap(20, 20, obs);
imagesc(0:19, 0:19, m)
set(gca, 'YDir', 'normal')