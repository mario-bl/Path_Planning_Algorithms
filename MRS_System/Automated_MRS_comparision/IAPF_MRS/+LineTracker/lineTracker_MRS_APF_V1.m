
function logs = lineTracker_MRS_APF_V1(Robots, obstacles)
    mapWidth = 20.5;  % Width of the map
    mapHeight = 20.7; % Height of the map
    % Parámetros del robot y control
    L=6.5/2*0.01;  % distancia de las ruedas al centro
    c=3.5/2*0.01;  % radio de las ruedas
    v_nom=13.5*0.01;
    w_max=(v_nom/c)/2;
    k_alpha=2;
    k_v=1;
    dt=0.05;
    epsilon_d=0.05;
    epsilon_alpha=deg2rad(5);

    n_robots = length(Robots);
    logs = struct('tray_x',{},'tray_y',{},'wi_log',{},'wd_log',{},'theta_log',{});


    for r=1:n_robots
        % Inicializar logs
        logs(r).tray_x = [];
        logs(r).tray_y = [];
        logs(r).wi_log = [];
        logs(r).wd_log = [];
        logs(r).theta_log = [];
        %estados iniciales
        trayectoria=Robots(r).path;
        x=trayectoria(1,1);
        y=trayectoria(1,2);
        theta=0;

        for i=2:100:size(trayectoria,1)
            x_target=trayectoria(i,1);
            y_target=trayectoria(i,2);
            goal_reached=false;
            while ~goal_reached
                dx=x_target-x;
                dy=y_target-y;
                d=sqrt(dx^2+dy^2);
                theta_des=atan2(dy,dx);
                alpha=wrapToPi(theta_des-theta);
                [wi,wd]=controlador(alpha,epsilon_alpha,w_max,k_alpha,k_v,v_nom,c,L);
                v=c*(wd+wi)/2;
                omega=c*(wd-wi)/L;
        
                x=x+v*cos(theta)*dt;
                y=y+v*sin(theta)*dt;
                theta=wrapToPi(theta+omega*dt);
                %Actualizamos registros
                logs(r).tray_x(end+1)=x;
                logs(r).tray_y(end+1)=y;
                logs(r).wi_log(end+1)=wi;
                logs(r).wd_log(end+1)=wd;
                logs(r).theta_log(end+1)=theta;
                if d<epsilon_d
                    goal_reached=true;
                end
            end
        end
   end

    % % Inicializar estados de cada robot
    % states = struct();
    % for r = 1:n_robots
    %     path = Robots(r).path;
    %     states(r).x = path(1,1);
    %     states(r).y = path(1,2);
    %     states(r).theta = Robots(r).theta;
    %     states(r).path = path;
    %     states(r).path_idx = 2;
    % end

    % Dimensiones del robot y ruedas
    L_robot = 0.08*10;
    W_robot = 0.05*10;
    L_wheel = 0.035*10;
    W_wheel = 0.01*10;

    % --- Graficar mapa y animar ---
    figure;
    xlim([0,mapWidth])
    ylim([0,mapHeight])
    title('Line Tracker with IAPF path planning - multi-robot simulation')
    xlabel('X')
    ylabel('Y')
    hold on
    for i = 1:length(obstacles)
        x = obstacles(i).x;
        y = obstacles(i).y;
        r = obstacles(i).r;
        c = obstacles(i).colour;
        rectangle('Position', [x - r, y - r, 2 * r, 2 * r], ...
                  'Curvature', [1, 1], ...
                  'FaceColor', c, ...
                  'EdgeColor', 'black');
    end

    % Colores para los robots
    robot_colors = lines(n_robots);

    % Graficar trayectorias deseadas
    for r = 1:n_robots
        plot(states(r).path(:,1), states(r).path(:,2), '-', 'Color', robot_colors(r,:), 'LineWidth', 1);
    end

    % Inicializar handles de animación
    h_path = gobjects(1, n_robots);
    h_body = gobjects(1, n_robots);
    h_wheel_left = gobjects(1, n_robots);
    h_wheel_right = gobjects(1, n_robots);
    for r = 1:n_robots
        h_path(r) = plot(NaN, NaN, ':', 'Color', robot_colors(r,:), 'LineWidth', 2.5);
        h_body(r) = fill(NaN, NaN, robot_colors(r,:));
        h_wheel_left(r) = fill(NaN, NaN, 'b');
        h_wheel_right(r) = fill(NaN, NaN, 'b');
    end

    % Simulación sincronizada
    finished = false(1, n_robots);
    while ~all(finished)
        for r = 1:n_robots
            if finished(r), continue; end
            path = states(r).path;
            idx = states(r).path_idx;
            if idx > size(path,1)
                finished(r) = true;
                continue;
            end
            x_target = path(idx,1);
            y_target = path(idx,2);
            x = states(r).x;
            y = states(r).y;
            theta = states(r).theta;

            dx = x_target - x;
            dy = y_target - y;
            d = sqrt(dx^2 + dy^2);
            theta_des = atan2(dy, dx);
            alpha = wrapToPi(theta_des - theta);
            [wi, wd] = controlador(alpha, epsilon_alpha, w_max, k_alpha, k_v, v_nom, c(1), L(1));
            v = c*(wd+wi)/2;
            omega = c*(wd-wi)/L;

            x = x + v*cos(theta)*dt;
            y = y + v*sin(theta)*dt;
            theta = wrapToPi(theta + omega*dt);

            logs(r).tray_x = [logs(r).tray_x, x];
            logs(r).tray_y = [logs(r).tray_y, y];
            logs(r).wi_log = [logs(r).wi_log, wi];
            logs(r).wd_log = [logs(r).wd_log, wd];
            logs(r).theta_log = [logs(r).theta_log, theta];

            states(r).x = x;
            states(r).y = y;
            states(r).theta = theta;

            if d < epsilon_d
                states(r).path_idx = idx + 1;
            end
        end

        % Actualizar animación
        for r = 1:n_robots
            if isempty(logs(r).tray_x), continue; end
            x = logs(r).tray_x(end);
            y = logs(r).tray_y(end);
            theta = logs(r).theta_log(end);
            wi = logs(r).wi_log(end);
            wd = logs(r).wd_log(end);

            [xb, yb] = rect_from_pose(x, y, theta, L_robot, W_robot);
            set(h_body(r), 'XData', xb, 'YData', yb);

            offset = W_robot / 2 + W_wheel / 2;
            [xwl, ywl] = rect_from_pose(x - offset*sin(theta), y + offset*cos(theta), theta, L_wheel, W_wheel);
            [xwr, ywr] = rect_from_pose(x + offset*sin(theta), y - offset*cos(theta), theta, L_wheel, W_wheel);

            color_l = [0 0 1] * (wi >= 0) + [1 0.5 0] * (wi < 0);
            color_r = [0 0 1] * (wd >= 0) + [1 0.5 0] * (wd < 0);

            set(h_wheel_left(r),  'XData', xwl, 'YData', ywl, 'FaceColor', color_l);
            set(h_wheel_right(r), 'XData', xwr, 'YData', ywr, 'FaceColor', color_r);

            set(h_path(r), 'XData', logs(r).tray_x, 'YData', logs(r).tray_y);
        end
        pause(0.01);
        drawnow;
    end
end

function [x_rect, y_rect] = rect_from_pose(x, y, theta, L, W)
    corners = [-L/2, -W/2;
                L/2, -W/2;
                L/2,  W/2;
               -L/2,  W/2]';
    R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
    rotated = R * corners;
    x_rect = rotated(1,:) + x;
    y_rect = rotated(2,:) + y;
end


