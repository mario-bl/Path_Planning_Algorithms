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

        function smoothPathWithSpline(obj, degree)
            if nargin < 1
                degree = 3; % Por defecto, spline cúbica
            end
            X=obj.real_path(:,2);
            Y=obj.real_path(:,1);
            t=linspace(0,1,length(X));
            
            sx=spapi(degree,t,X);
            sy=spapi(degree,t,Y);
            
            t_fine=linspace(0,1,length(X)*20);
            X_smooth = fnval(sx, t_fine)';
            Y_smooth = fnval(sy, t_fine)';
            
            obj.smoothedPath=[Y_smooth,X_smooth];
          
        end
             
        function smoothPathWithBSpline(obj, degree, numCtrlPoints)
            if nargin < 2
                degree = 3; % Por defecto cúbica
            end
            if nargin < 3
                %numCtrlPoints = round(length(obj.real_path) / 2); % Por defecto, la mitad de los puntos originales
                numCtrlPoints = round(length(obj.real_path) / 10);
            end
        
            X = obj.real_path(:, 2);
            Y = obj.real_path(:, 1);
            t = linspace(0, 1, length(X));
        
            % Ajuste B-spline a los datos con 'numCtrlPoints' nodos de control
            sx = spap2(numCtrlPoints, degree, t, X);
            sy = spap2(numCtrlPoints, degree, t, Y);
        
            % Re-evaluamos en una malla fina para suavizar la trayectoria
            t_fine = linspace(0, 1, length(X) * 20);
            X_smooth = fnval(sx, t_fine)';
            Y_smooth = fnval(sy, t_fine)';
        
            obj.smoothedPath = [Y_smooth, X_smooth];
            obj.update_costs;
            
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



    end
end
