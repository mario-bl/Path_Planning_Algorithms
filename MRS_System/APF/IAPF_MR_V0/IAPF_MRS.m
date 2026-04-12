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
        end
    end 
    
    Fx=F_att_x+F_rep_x;
    Fy=F_att_y+F_rep_y;
    %Output de la funcion
    F=[Fx,Fy];        

end
    

