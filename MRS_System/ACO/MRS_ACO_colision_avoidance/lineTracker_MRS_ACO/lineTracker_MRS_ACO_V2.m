function lineTracker_MRS_ACO_V2(Robots, mapa)
% lineTracker_MRS_ACO_V2  Simulación/animación de seguimiento de trayectorias
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
    c=0.35/2;    % [m] radio rueda
    v_nom   = 13.5*0.01;     % [m/s] velocidad nominal (ya escalada 0.8)
    w_max   = (v_nom/c)/2;   % [rad/s] límite de giro (moderado)
    k_alpha = 2;
    k_v     = 1;
    dt      = 0.05;          % [s]
    epsilon_d     = 0.05;    % [m] umbral distancia punto alcanzado
    epsilon_alpha = deg2rad(5);
    n_Robots=size(Robots,2);
    % ---------- Simulación offline de cada robot ----------
    for r = 1:n_Robots
        trayectoria = Robots(r).smoothedPath;
        if ~isempty(Robots(r).log_stop)
            f=1;
            filas_parada=Robots(r).log_stop; %En estos puntos forzamos al seguidor a que se detenga en la imlementacion real mandaremos algun comando especial
            n_ptos_parada=length(filas_parada);
            
            fila_p=filas_parada(f);
            flag_paradas=true;
        else
            flag_paradas=false;
        end

        if isempty(trayectoria) || size(trayectoria,2) ~= 2
            error('Robots(%d).path debe ser Nx2 [x y].', r);
        end
        if isfield(Robots(r),'orientacion')
            theta = Robots(r).orientacion;
        else
            theta = 0;
        end

        % Estado inicial
        x = trayectoria(1,2);
        y = trayectoria(1,1);

        tray_x = [];
        tray_y = [];
        wi_log=[];
        wd_log=[];
        theta_log=[];

        % Recorremos la trayectoria saltando puntos si es muy densa (como en tu V1)
        % Si no quieres salto, cambia 100 por 1.
        % stride = 22;
        fprintf("Robot %d tamaño trayectoria: %d\n",r,length(trayectoria))
        for i = 2:size(trayectoria,1)

            %Hay que incluir aqui la gestion para la simulacion
            
            x_target = trayectoria(i,2);
            y_target = trayectoria(i,1);
            goal_reached = false;
    
            while ~goal_reached
                %if flag_paradas && ((fila_p-22<=i)&&(i<=fila_p+22)) 
                if flag_paradas && ((fila_p-2<=i)&&(i<=fila_p+2)) 
                    tray_x(end+1:end+600)=tray_x(end);
                    tray_y(end+1:end+600)=tray_y(end);
                    wi_log(end+1:end+600) = 0; %#ok<AGROW>
                    wd_log(end+1:end+600) = 0; %#ok<AGROW>
                    theta_log(end+1:end+600)= theta_log(end); 

                        f=f+1;
                    if f<=n_ptos_parada
                        fila_p=filas_parada(f);
                    else
                        flag_paradas=false;%Ya no hay mas necesidad de paradas
                    end
                    
                else
                    dx = x_target - x;
                    dy = y_target - y;
                    d  = sqrt(dx^2+dy^2);
                    theta_des = atan2(dy,dx);
                    alpha = wrapToPi(theta_des - theta);
                    [wi,wd] = controlador(alpha, epsilon_alpha, w_max, k_alpha, k_v, v_nom, c, L);
                    v = c*(wd+wi)/2;
                    omega = c*(wd-wi)/L;
    
                    x = x + v*cos(theta)*dt;
                    y = y + v*sin(theta)*dt;
                    theta = wrapToPi(theta + omega*dt);
        
                    % Log
                    tray_x(end+1)=x;
                    tray_y(end+1)=y;
                    wi_log(end+1)=wi;
                    wd_log(end+1)=wd;
                    theta_log(end+1)=theta;
        
                    if d < epsilon_d
                        goal_reached = true;
                    end
                end
            end
        end
        Robots(r).updateSimRecord(tray_x,tray_y,wi_log,wd_log,theta_log);
    end

    
    % Dimensiones visuales del robot
    L_robot = 0.9;%0.08*10;
    W_robot = 0.7;%0.05*10;
    L_wheel = 0.35;%0.035*10;
    W_wheel = 0.1;%0.01*10;
    % --- Inicializar figura ---
    figure;
    axis equal;
    grid on;
    imagesc(mapa);
    colormap(gray);
    title('IACO-MRS');
    xlabel('X (dm)')
    ylabel('Y (dm)')
    hold on
    
    % --- Dibujo de trayectorias deseadas ---
    for i = 1:n_Robots
        plot(Robots(i).smoothedPath(:,2), Robots(i).smoothedPath(:,1), 'r-', 'LineWidth', 1);
    end
    % --- Determinar número de frames ---
    n_frames = max(cellfun(@(x) length(x), {Robots.tray_x}));
    % --- Inicializar gráficos para cada robot ---
    for i = 1:n_Robots
        % Posición inicial
        x = Robots(i).tray_x(1);
        y = Robots(i).tray_y(1);
        theta = Robots(i).theta_log(1);
        wi = Robots(i).wi_log(1);
        wd = Robots(i).wd_log(1);
    
        % Cuerpo
        [xb, yb] = rect_from_pose(x, y, theta, L_robot, W_robot);
        h(i).body = fill(xb, yb, 'k');
    
        % Ruedas
        offset = W_robot / 2 + W_wheel / 2;
        [xwl, ywl] = rect_from_pose(x - offset * sin(theta), y + offset * cos(theta), theta, L_wheel, W_wheel);
        [xwr, ywr] = rect_from_pose(x + offset * sin(theta), y - offset * cos(theta), theta, L_wheel, W_wheel);
    
        color_l = ternColor(wi >= 0);
        color_r = ternColor(wd >= 0);
    
        h(i).wheel_left  = fill(xwl, ywl, color_l);
        h(i).wheel_right = fill(xwr, ywr, color_r);
    
        % Trayectoria recorrida
        h(i).path = plot(Robots(i).tray_x(1), Robots(i).tray_y(1), 'k:', 'LineWidth', 2.5);
    end

    % --- Animación ---
    for f = 1:5:n_frames
        for i = 1:n_Robots
            if f <= length(Robots(i).tray_x)
                x = Robots(i).tray_x(f);
                y = Robots(i).tray_y(f);
                theta = Robots(i).theta_log(f);
                wi = Robots(i).wi_log(f);
                wd = Robots(i).wd_log(f);
    
                % Cuerpo
                [xb, yb] = rect_from_pose(x, y, theta, L_robot, W_robot);
                set(h(i).body, 'XData', xb, 'YData', yb);
    
                % Ruedas
                offset = W_robot / 2 + W_wheel / 2;
                [xwl, ywl] = rect_from_pose(x - offset * sin(theta), y + offset * cos(theta), theta, L_wheel, W_wheel);
                [xwr, ywr] = rect_from_pose(x + offset * sin(theta), y - offset * cos(theta), theta, L_wheel, W_wheel);
    
                color_l = [0 0 1] * (wi >= 0) + [1 0.5 0] * (wi < 0);
                color_r = [0 0 1] * (wd >= 0) + [1 0.5 0] * (wd < 0);
    
                set(h(i).wheel_left,  'XData', xwl, 'YData', ywl, 'FaceColor', color_l);
                set(h(i).wheel_right, 'XData', xwr, 'YData', ywr, 'FaceColor', color_r);
    
                % Trayectoria recorrida
                set(h(i).path, 'XData', Robots(i).tray_x(1:f), 'YData', Robots(i).tray_y(1:f));
            end
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