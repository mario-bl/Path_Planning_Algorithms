function obstacles = generateScenario(n_filas, n_cols, n_obs, trajectories, rng_seed)
%GENERATESCENARIO  Genera un escenario aleatorio de obstáculos compatible
%                  con los formatos APF y ACO.
%
%  Entradas:
%    n_filas      - número de filas del mapa
%    n_cols       - número de columnas del mapa
%    n_obs        - número de obstáculos a colocar
%    trajectories - array de structs: struct('x0',v,'y0',v,'xObj',v,'yObj',v)
%    rng_seed     - (opcional) semilla para reproducibilidad
%
%  Salida:
%    obstacles    - array de structs: struct('x','y','r','colour','k_rep')
%
%  Restricciones:
%    · (n_filas·n_cols) / n_obs >= 40
%    · Distribución: 1/10 → 4×4 | 4/10 → 3×3 | 5/10 → 2×2
%    · Centros en [1, dim-1] (bloques pares: entero+0.5, impares: entero)
%    · Distancia mínima obstáculo ↔ punto trayectoria: r_detect + 1.5
%    · Distancia mínima entre obstáculos (borde a borde): 2.0 celdas

    % ── Validación proporción mapa/obstáculos ──────────────────────────
    if (n_filas * n_cols) / n_obs < 40
        error(['Proporción mapa/obstáculos insuficiente: ' ...
               '(%d·%d)/%d = %.1f < 40. Reduce n_obs o aumenta el mapa.'], ...
               n_filas, n_cols, n_obs, (n_filas*n_cols)/n_obs);
    end

    % ── Semilla aleatoria opcional ─────────────────────────────────────
    if nargin >= 5
        rng(rng_seed);
    end

    % ── Radios APF y de detección ──────────────────────────────────────
    %   r_detect = r_apf / sqrt(2)  →  semi-lado del bloque
    R.apf  = [2*sqrt(2),          sqrt(2)+1/sqrt(2), sqrt(2)        ];
    R.det  = [2.0,                1.5,               1.0            ];
    R.off  = [0.5,                0.0,               0.5            ]; % offset coord.
    %          4×4 (par)           3×3 (impar)        2×2 (par)

    k_rep = 1.0;

    % ── Distribución de obstáculos ─────────────────────────────────────
    n_4x4 = round(n_obs * 1/10);
    n_3x3 = round(n_obs * 4/10);
    n_2x2 = n_obs - n_4x4 - n_3x3;   % el resto garantiza que sumen n_obs

    counts = [n_4x4, n_3x3, n_2x2];

    % Lista de tipos (1=4×4, 2=3×3, 3=2×2) en orden aleatorio
    type_list = [];
    for t = 1:3
        type_list = [type_list, repmat(t, 1, counts(t))]; %#ok<AGROW>
    end
    type_list = type_list(randperm(n_obs));

    % ── Parámetros de restricción ──────────────────────────────────────
    MIN_DIST_TRAJ = 1.5;   % margen mínimo borde obstáculo ↔ punto trayectoria
    MIN_DIST_OBS  = 2.0;   % margen mínimo borde ↔ borde entre obstáculos
    MAX_ATTEMPTS  = 1000;

    obstacles = struct('x',{},'y',{},'r',{},'colour',{},'k_rep',{});

    % ── Colocación iterativa ───────────────────────────────────────────
    for i = 1:n_obs
        t      = type_list(i);
        r_apf  = R.apf(t);
        r_det  = R.det(t);
        offset = R.off(t);

        placed = false;

        for attempt = 1:MAX_ATTEMPTS

            % Coordenada candidata en [1, dim-1]
            if offset == 0.5
                % Bloque par: centro en entero+0.5 → [1.5, dim-1.5]
                xi = randi([1, n_cols-2]) + 0.5;
                yi = randi([1, n_filas-2]) + 0.5;
            else
                % Bloque impar: centro en entero → [1, dim-1]
                xi = randi([1, n_cols-1]);
                yi = randi([1, n_filas-1]);
            end

            valid = true;

            % ── Restricción: puntos de trayectoria ──────────────────
            for tr = 1:length(trajectories)
                pts = [trajectories(tr).x0,   trajectories(tr).y0;
                       trajectories(tr).xObj,  trajectories(tr).yObj];
                for p = 1:size(pts, 1)
                    d = sqrt((xi - pts(p,1))^2 + (yi - pts(p,2))^2);
                    if d < r_det + MIN_DIST_TRAJ
                        valid = false;
                        break;
                    end
                end
                if ~valid, break; end
            end

            if ~valid, continue; end

            % ── Restricción: distancia entre obstáculos ──────────────
            for j = 1:length(obstacles)
                r_det_j = obstacles(j).r / sqrt(2);
                d = sqrt((xi - obstacles(j).x)^2 + (yi - obstacles(j).y)^2);
                if (d - r_det - r_det_j) < MIN_DIST_OBS
                    valid = false;
                    break;
                end
            end

            if valid
                obstacles(end+1) = struct('x', xi, 'y', yi, ...
                                          'r', r_apf,         ...
                                          'colour', 'black',  ...
                                          'k_rep', k_rep);    %#ok<AGROW>
                placed = true;
                break;
            end
        end

        if ~placed
            error(['No se pudo colocar el obstáculo %d (%dx%d) tras %d intentos.\n' ...
                   'Considera reducir n_obs o aumentar las dimensiones del mapa.'], ...
                   i, 2*(t+1), 2*(t+1), MAX_ATTEMPTS);
        end
    end
end