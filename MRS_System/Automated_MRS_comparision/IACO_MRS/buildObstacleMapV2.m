function mapa = buildObstacleMapV2(n_filas, n_cols, obstacles)
    % Mapa extendido con borde libre de 1 celda en cada lado
    n_filas_ext = n_filas + 2;
    n_cols_ext  = n_cols  + 2;

    mapa = ones(n_filas_ext, n_cols_ext);
    [Xc, Yc] = meshgrid(1:n_cols_ext, 1:n_filas_ext);

    for k = 1:length(obstacles)
        % +2: +1 por 1-based MATLAB, +1 por borde añadido
        x0 = obstacles(k).x + 2;
        y0 = obstacles(k).y + 2;

        if (x0 < 2 || x0 > n_cols_ext-1 || y0 < 2 || y0 > n_filas_ext-1)
            fprintf('Obstáculo %d fuera del mapa: (%.1f, %.1f)\n', k, x0, y0);
            return
        end

        r_detect  = obstacles(k).r / sqrt(2);
        dist_cheb = max(abs(Xc - x0), abs(Yc - y0));
        mapa(dist_cheb <= r_detect) = 0;
    end
end