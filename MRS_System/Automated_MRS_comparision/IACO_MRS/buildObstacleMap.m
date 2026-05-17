function mapa = buildObstacleMap(n_filas, n_cols, obstacles)
%BUILDOBSTACLEMAP  Convierte obstáculos APF (círculo circunscrito al bloque)
%                  a mapa binario de celdas para ACO.
%
%  Convención: x → columnas (0..n_cols-1), y → filas (0..n_filas-1)
%  mapa(i,j) = 1 libre | 0 ocupada
%
%  Radio APF = semi-diagonal del bloque (circunscrito)
%  Radio detección = r/sqrt(2) = semi-lado del bloque (inscrito)

     mapa = ones(n_filas, n_cols);
    [Xc, Yc] = meshgrid(1:n_cols, 1:n_filas);   % coordenadas 1-based

    for k = 1:length(obstacles)
        x0 = obstacles(k).x + 1;   % columna MATLAB (1..n_cols)
        y0 = obstacles(k).y + 1;   % fila    MATLAB (1..n_filas)

        % Validación
        if (x0 < 1 || x0 > n_cols || y0 < 1 || y0 > n_filas)
            fprintf('Obstáculo %d fuera del mapa: (%.1f, %.1f)\n', k, x0, y0);
            return
        end

        r_apf    = obstacles(k).r;
        r_detect = r_apf / sqrt(2);

        % dist = sqrt((Xc - x0).^2 + (Yc - y0).^2);
        % mapa(dist <= r_detect) = 0;
        dist_cheb = max(abs(Xc - x0), abs(Yc - y0));
        mapa(dist_cheb <= r_detect) = 0;
    end
end