classdef Robot < handle
    properties
        real_path   % Trayectoria interpolada para seguimiento
        smoothedPath
        flag_matrix;
        rob_n;
        timeStep;
        cost_l;
        cost_w;
        oldTheta;
        orientation;
        tray_x;
        tray_y;
        wi_log;
        wd_log;
        theta_log;
        log_stop;
    end

    methods
        function obj = Robot(path,N_rob,cost_l,cost_w,theta0)
            obj.generatePath(path);
            obj.smoothedPath=0;
            obj.flag_matrix=zeros(N_rob);
            obj.cost_l=cost_l;
            obj.cost_w=cost_w;
            obj.oldTheta=0;
            obj.orientation=theta0;
            obj.log_stop=[];
            obj.smoothedPath = obj.real_path;
            %obj.rob_n=id; 
            
        end

        function generatePath(obj, path)
            %obj.real_path=path;
            densidad = 3;  % puntos por unidad de distancia
            totalPoints = 0;

            % Paso 1: Calcular el número total de puntos necesarios
            for i = 1:size(path,1)-1
                dist = norm(path(i+1,:) - path(i,:));
                totalPoints = totalPoints + max(round(densidad * dist), 2);
            end

            % Paso 2: Preasignar memoria
            tray = zeros(totalPoints, 2);
            idx = 1;

            % Paso 3: Interpolar cada segmento
            for i = 1:size(path,1)-1
                p1 = path(i,:);
                p2 = path(i+1,:);
                dist = norm(p2 - p1);
                n = max(round(densidad * dist), 2);

                x_interp = linspace(p1(1), p2(1), n)';
                y_interp = linspace(p1(2), p2(2), n)';

                tray(idx:idx+n-1, :) = [x_interp, y_interp];
                idx = idx + n - 1;  % avanzamos sin duplicar el punto final
            end

            % Si sobran filas por redondeos, las recortamos
            obj.real_path = tray(1:idx-1, :);

            % Asegurar que el último punto del path esté incluido
            obj.real_path = [obj.real_path; path(end,:)];
            
        end

        function update_costs(obj)
            obj.cost_l=0;
            obj.cost_w=0;
            stopCounter=0;
            oldTh=atan2(obj.smoothedPath(end,2)-obj.smoothedPath(1,2),obj.smoothedPath(end,1)-obj.smoothedPath(1,1));%orientacion inicial
            for i=1:(length(obj.smoothedPath)-1)
                d=norm(obj.smoothedPath(i+1,:)-obj.smoothedPath(i,:));
                if d==0
                    stopCounter=stopCounter+1;
                else
                    obj.cost_l=obj.cost_l+d;
                    newTheta=atan2(obj.smoothedPath(i+1,1)-obj.smoothedPath(i,1),obj.smoothedPath(i+1,2)-obj.smoothedPath(i,2));
                    dtheta=abs(angleDiff(newTheta,oldTh));
                    obj.cost_w=obj.cost_w+(dtheta/d);
                    oldTh=newTheta;
                end
            end
            obj.cost_w=obj.cost_w/(length(obj.smoothedPath)-1-stopCounter);

            
        end

        function updateSimRecord(obj,tray_x,tray_y,log_wi,log_wd,theta_log)
            obj.tray_x=tray_x;
            obj.tray_y=tray_y;
            obj.wi_log=log_wi;
            obj.wd_log=log_wd;
            obj.theta_log=theta_log;

        end

        function updatepath(obj,newPath,cost_l,cost_w)
            obj.generatePath(newPath);
            obj.cost_l=cost_l;
            obj.cost_w=cost_w;
            obj.smoothedPath = obj.real_path;

        end

    end
end
