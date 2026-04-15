clc; clear; close all;

% Definir puntos de control (trayectoria original)
X = [0, 1, 3, 6, 8, 10];
Y = [0, 2, 1, 3, 0, -1];

% Definir el parámetro de la spline (normalizado)
t = linspace(0, 1, length(X)); 

% Grado de la B-spline
k = 3; % Spline cúbica

% Crear la B-spline usando interpolación
sx = spapi(k, t, X);
sy = spapi(k, t, Y);

% Evaluar la spline en un conjunto denso de puntos
t_fine = linspace(0, 1, 100);
X_smooth = fnval(sx, t_fine);
Y_smooth = fnval(sy, t_fine);

% Graficar resultados
figure;
plot(X, Y, 'ro-', 'LineWidth', 1.5, 'MarkerSize', 8); % Puntos de control
hold on;
plot(X_smooth, Y_smooth, 'b-', 'LineWidth', 2); % B-spline suavizada
legend('Puntos de control', 'B-spline suavizada');
xlabel('X');
ylabel('Y');
title('Optimización de Trayectoria con B-spline');
grid on;
axis equal;
