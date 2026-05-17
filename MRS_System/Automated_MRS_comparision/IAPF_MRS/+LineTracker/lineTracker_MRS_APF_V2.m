function [logs]= lineTracker_MRS_APF_V2(Robots, obstacles)
% lineTracker_MRS_APF_V1  Simulación/animación de seguimiento de trayectorias
%                         para un sistema Multi-Robot usando el controlador
%                         giro/avance por tramos (misma lógica que la versión mono-robot).
%
% INPUTS
%   Robots(k).path   -> [N_k x 2] matriz de puntos [x y] en metros
%   Robots(k).theta  -> orientación inicial (rad) opcional; si falta se asume 0
%   obstacles(i).x, .y, .r, .colour  -> discos
%
% OUTPUT
%   logs(k) con tray_x, tray_y, wi_log, wd_log, theta_log

    % ---------- Parámetros del robot y control ----------
    L=0.65/2;%6.5/2*0.01;    % [m] semi‑track (centro a rueda)  NOTE: Si prefieres distancia total, duplica.
    c=0.35/2;%    % [m] radio rueda
    v_nom   = 13.5*0.01;     % [m/s] velocidad nominal (ya escalada 0.8)
    w_max   = (v_nom/c)/2;   % [rad/s] límite de giro (moderado)
    k_alpha = 2;
    k_v     = 1;
    dt      = 0.05;          % [s]
    epsilon_d     = 0.05;    % [m] umbral distancia punto alcanzado
    epsilon_alpha = deg2rad(5);

    % ---------- Inicialización de logs ----------
    n_robots = numel(Robots);
    logs = struct('tray_x',{},'tray_y',{},'wi_log',{},'wd_log',{},'theta_log',{});
    logs(n_robots).tray_x = [];  % preasigna campo final

    % ---------- Simulación offline de cada robot ----------
    for r = 1:n_robots
        trayectoria = Robots(r).path;
        if ~isempty(Robots(r).log_parallel)
            f=1;
            filas_parada=Robots(r).log_parallel; %En estos puntos forzamos al seguidor a que se detenga en la imlementacion real mandaremos algun comando especial
            n_ptos_parada=length(filas_parada);
            
            fila_p=filas_parada(f);
            flag_paradas=true;
        else
            flag_paradas=false;
        end

        if isempty(trayectoria) || size(trayectoria,2) ~= 2
            error('Robots(%d).path debe ser Nx2 [x y].', r);
        end
        if isfield(Robots(r),'theta')
            theta = Robots(r).theta;
        else
            theta = 0;
        end

        % Estado inicial
        x = trayectoria(1,1);
        y = trayectoria(1,2);

        % Logs indiv.
        tx = x;  % guardamos primer punto
        ty = y;
        wli = []; wri = []; th = theta;

        % Recorremos la trayectoria saltando puntos si es muy densa (como en tu V1)
        % Si no quieres salto, cambia 100 por 1.
        stride = 100;%100;
        %fprintf("Robot %d tamaño trayectoria: %d\n",r,length(trayectoria));
        for i = 2:stride:size(trayectoria,1)
            
            if flag_paradas && ((fila_p-50<=i)&&(i<=fila_p+50))  %lo primero es ver si tenemos que poner al robot a esperar
                %medida para la simulacion en la puesta en punto habra que
                %pensar en otra gestión
                tx(end+1:end+200)=tx(end);
                ty(end+1:end+200)=ty(end);
                wli(end+1:end+200) = 0; %#ok<AGROW>
                wri(end+1:end+200) = 0; %#ok<AGROW>
                th(end+1:end+200)= th(end); 



                f=f+1;
                if f<=n_ptos_parada
                    fila_p=filas_parada(f);
                else
                    flag_paradas=false;%Ya no hay mas necesidad de paradas
                end
            
            else %Todos los demas casos que no sea necesario parar el robot
                x_target = trayectoria(i,1);
                y_target = trayectoria(i,2);
                goal_reached = false;
    
                while ~goal_reached
                    dx = x_target - x;
                    dy = y_target - y;
                    d  = hypot(dx,dy);
    
                    theta_des = atan2(dy,dx);
                    alpha = wrapToPi(theta_des - theta);
    
                    [wi,wd] = LineTracker.controlador(alpha, epsilon_alpha, w_max, k_alpha, k_v, v_nom, c, L);
    
                    v = c*(wd+wi)/2;
                    omega = c*(wd-wi)/L;
    
                    % Integración cinemática
                    x = x + v*cos(theta)*dt;
                    y = y + v*sin(theta)*dt;
                    theta = wrapToPi(theta + omega*dt);
    
                    % Log
                    tx(end+1) = x; %#ok<AGROW>
                    ty(end+1) = y; %#ok<AGROW>
                    wli(end+1) = wi; %#ok<AGROW>
                    wri(end+1) = wd; %#ok<AGROW>
                    th(end+1)  = theta; %#ok<AGROW>
    
                    if d < epsilon_d
                        goal_reached = true;
                    end
                end
            end  
        end

        % Guardar en logs de salida
        logs(r).tray_x   = tx(:);
        logs(r).tray_y   = ty(:);
        logs(r).wi_log   = wli(:);
        logs(r).wd_log   = wri(:);
        logs(r).theta_log= th(:);
    end

    % ---------- Representación gráfica/animación ----------
    % Escala del mapa: puedes parametrizar si quieres
    mapWidth  = 20.5;
    mapHeight = 20.7;

    % Dimensiones visuales del robot
    L_robot = 0.9;%0.08*10;
    W_robot = 0.7;%0.05*10;
    L_wheel = 0.35;%0.035*10;
    W_wheel = 0.1;%0.01*10;
    wheel_offset = W_robot/2 + W_wheel/2;

    % Colores de trayectorias deseadas para cada robot
    path_colors = lines(n_robots);  % matriz Nx3

    figure;
    hold on;
    xlim([0,mapWidth]);
    ylim([0,mapHeight]);
    axis equal;
    grid on;
    title("IAPF-MRS FLAGS SOLUTION")
    xlabel('X (dm)')
    ylabel('Y (dm)')

    % Obstáculos
    for i = 1:numel(obstacles)
        xo = obstacles(i).x;
        yo = obstacles(i).y;
        ro = obstacles(i).r;
        co = obstacles(i).colour;
        rectangle('Position',[xo-ro, yo-ro, 2*ro, 2*ro],...
                  'Curvature',[1 1],...
                  'FaceColor',co,...
                  'EdgeColor','k');
    end

    %Puntos objetivos y de inicio
    for r=1:n_robots
        plot(Robots(r).goal(1), Robots(r).goal(2), '*', 'MarkerSize', 15 , 'MarkerEdgeColor',path_colors(r,:),'LineWidth',2);
        plot(Robots(r).path(1,1), Robots(r).path(1,2), 'o', 'MarkerSize', 15 , 'MarkerEdgeColor',path_colors(r,:),'LineWidth',2);
    end

    % Traza de trayectorias planificadas
    h_path_plan = gobjects(n_robots,1);
    for r = 1:n_robots
        pth = Robots(r).path;
        h_path_plan(r) = plot(pth(:,1), pth(:,2), '-', ...
                              'Color', path_colors(r,:), ...
                              'LineWidth', 1.0, ...
                              'DisplayName', sprintf('Path R%d',r));
    end

    % Traza recorrida (punteada negra)
    h_path_run = gobjects(n_robots,1);
    for r = 1:n_robots
        h_path_run(r) = plot(NaN, NaN, 'k:', 'LineWidth', 1.5,...
                             'DisplayName', sprintf('Run R%d',r));
    end

    % Patches de cuerpos y ruedas
    h_body        = gobjects(n_robots,1);
    h_wheel_left  = gobjects(n_robots,1);
    h_wheel_right = gobjects(n_robots,1);

    for r = 1:n_robots
        h_body(r)        = fill(NaN,NaN,'k','EdgeColor','none');
        h_wheel_left(r)  = fill(NaN,NaN,'b','EdgeColor','none');
        h_wheel_right(r) = fill(NaN,NaN,'b','EdgeColor','none');
    end

    
    % Número de frames: el mayor log de los robots
    max_frames = max(arrayfun(@(s) numel(s.tray_x), logs));

    % Animación
    for k = 1:5:max_frames
        for r = 1:n_robots
            % Accede al último estado válido si k supera longitud
            idx = min(k, numel(logs(r).tray_x));

            xr = logs(r).tray_x(idx);
            yr = logs(r).tray_y(idx);
            tr = logs(r).theta_log(idx);

            % Cuerpo
            [xb,yb] = rect_from_pose(xr, yr, tr, L_robot, W_robot);
            set(h_body(r), 'XData', xb, 'YData', yb);

            % Ruedas (izq = +rotación > y local; definimos igual que en mono-robot)
            [xwl,ywl] = rect_from_pose(xr - wheel_offset*sin(tr), ...
                                       yr + wheel_offset*cos(tr), ...
                                       tr, L_wheel, W_wheel);
            [xwr,ywr] = rect_from_pose(xr + wheel_offset*sin(tr), ...
                                       yr - wheel_offset*cos(tr), ...
                                       tr, L_wheel, W_wheel);

            % Colores de ruedas según signo de velocidad
            if idx > 1
                wi = logs(r).wi_log(idx-1); % velocidad que llevó al estado idx
                wd = logs(r).wd_log(idx-1);
            else
                wi = 0; wd = 0;
            end
            col_l = ternColor(wi >= 0);
            col_r = ternColor(wd >= 0);

            set(h_wheel_left(r),  'XData', xwl, 'YData', ywl, 'FaceColor', col_l);
            set(h_wheel_right(r), 'XData', xwr, 'YData', ywr, 'FaceColor', col_r);

            % Trayectoria recorrida hasta k
            set(h_path_run(r), 'XData', logs(r).tray_x(1:idx), ...
                               'YData', logs(r).tray_y(1:idx));
        end

        pause(0.01);
        drawnow;
    end
end

% ---------- Utilidades locales ----------
function [x_rect, y_rect] = rect_from_pose(x, y, theta, L, W)
    % Rectángulo centrado en (x,y), orientación theta
    corners = [-L/2, -W/2;
                L/2, -W/2;
                L/2,  W/2;
               -L/2,  W/2]';
    R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
    rotated = R * corners;
    x_rect = rotated(1,:) + x;
    y_rect = rotated(2,:) + y;
end

function col = ternColor(isPos)
    % Azul si positivo, naranja si negativo
    if isPos
        col = [0 0 1];
    else
        col = [1 0.5 0];
    end
end