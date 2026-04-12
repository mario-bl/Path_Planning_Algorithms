function [F]=IAPF_MRS(rob,Robots,cts,n_obs_r,i_rob)
    d_max=7;
    xGoal=rob.goal(1,1);
    yGoal=rob.goal(1,2);
    F_att_x=cts.k_att*(xGoal-rob.x);
    F_att_y=cts.k_att*(yGoal-rob.y);
    %Inicializamos fuerzas repulsivas
    F_rep_x=0;
    F_rep_y=0;
    %Revisamos si algun obstaculo esta en radio de accion

    for i=1:length(rob.obs)
        %Distancia del robot a los obstaculos
        d_obs=sqrt((rob.obs(i).x-rob.x)^2+(rob.obs(i).y-rob.y)^2)-rob.obs(i).r; %Le restamos el radio del obstaculo

        if(d_obs<cts.d_inf) %Distancia al obstaculo con el engorde esta dentro de la influencia
            %Primer termino repulsivo
            F_rep_x=F_rep_x+rob.obs(i).k_rep*(1/d_obs-1/cts.d_inf)/(d_obs^3)*min((rob.x-xGoal)^2+(rob.y-yGoal)^2,d_max)*(rob.x-rob.obs(i).x); 
            F_rep_y=F_rep_y+rob.obs(i).k_rep*(1/d_obs-1/cts.d_inf)/(d_obs^3)*min((rob.x-xGoal)^2+(rob.y-yGoal)^2,d_max)*(rob.y-rob.obs(i).y); 
            %Segundo termino repulsivo falta un signo negativo
            F_rep_x=F_rep_x-rob.obs(i).k_rep*(1/d_obs-1/cts.d_inf)^2*(rob.x-xGoal);
            F_rep_y=F_rep_y-rob.obs(i).k_rep*(1/d_obs-1/cts.d_inf)^2*(rob.y-yGoal);
        end
    end
    %Revisamos si algun robot esta en radio de accion
    for j=1:length(Robots)
        d_ij=norm(Robots(j).path(end,:)-rob.path(end,:))-0.2;
        if(~isequal(Robots(j),rob))&&(d_ij<=cts.d_inf_robs) && (Robots(j).APF_flag) %%Condicion de que no sea el mismo robot, distancia menor o igual a la de influencia y el robot(j) no haya alcanzado el punto objetivo 
        
            F_rep_x=F_rep_x+cts.k_rep*(1/d_ij-1/cts.d_inf_robs)/(d_ij^3)*min((rob.x-xGoal)^2+(rob.y-yGoal)^2,3*d_max)*(rob.x-Robots(j).x); 
            F_rep_y=F_rep_y+cts.k_rep*(1/d_ij-1/cts.d_inf_robs)/(d_ij^3)*min((rob.x-xGoal)^2+(rob.y-yGoal)^2,3*d_max)*(rob.y-Robots(j).y);
            
            F_rep_x=F_rep_x-2*cts.k_rep*(1/d_ij-1/cts.d_inf_robs)^2*(rob.x-xGoal);
            F_rep_y=F_rep_y-2*cts.k_rep*(1/d_ij-1/cts.d_inf_robs)^2*(rob.y-yGoal);
            %Añadimos un objeto virtual en la direccion destino
            n=length(rob.obs); %Numero de obstaculos
            m=length(Robots(j).obs);
            if(checkParallelRobots(rob,Robots(j),cts.l))
                rob.flagParallel=true;
            end

            if n > n_obs_r  
                for k = (n_obs_r + 1):n
                    distancia = sqrt((rob.obs(k).x - rob.x)^2 + (rob.obs(k).y - rob.y)^2);
                    if distancia < cts.d_inf
                        rob.flagParallel= false; % No se permite la creación de más objetos virtuales
                        break; % Se sale del bucle en cuanto se encuentra un obstáculo cercano
                    end
                end
            end
            if(norm(rob.path(end,:)-rob.goal(:)')<3)
                rob.flagParallel=false;
            end

            if (rob.flagParallel)
                fprintf("El robot %d y %d estan llevando a cabo un movimiento en paralelo \n",i_rob,j)
                fprintf("Se añadira un objeto virtual a %d para evitar esta situacion\n", j)
                % fprintf("Casi se produce una closision entre robots en (%.2f,%.2f) \n",rob.x,rob.y);
                % fprintf("Se añadera un objeto virtual para evitar desviaciones por influencia del potencial\n");
                theta_1=atan2(yGoal-rob.y,xGoal-rob.x)+(rand-0.5)*60*pi/180;%angulo aleatorio +30 -30
                %theta_2=atan2(rob.goal(2)-rob.y,rob.goal(1)-rob.x)+(rand-0.5)*40*pi/180;%angulo aleatorio +10 -10
                rob.obs(n+1)=struct('x', rob.x+0.8*cts.d_inf*cos(theta_1), 'y', rob.y+0.8*cts.d_inf*sin(theta_1), 'r', 0.5,'colour','magenta','k_rep',cts.k_rep_v);%En verde para que se vea que es virtual
                Robots(j).obs(m+1)=struct('x', rob.x+0.8*cts.d_inf*cos(theta_1), 'y', rob.y+0.8*cts.d_inf*sin(theta_1), 'r', 0.5,'colour','magenta','k_rep',cts.k_rep_v);
                rob.flagParallel=false;
            end
        end
    end 
    
    Fx=F_att_x+F_rep_x;
    Fy=F_att_y+F_rep_y;
    %Output de la funcion
    F=[Fx,Fy];        

end
    

