function [wi,wd]=controlador(alpha,epsilon_alpha,omega_max,k_alpha,k_v,v_nom,c,L)
    %Dos opciones avance o rotacion, en base a la 
    if abs(alpha)>epsilon_alpha
        %Giro en torno a su eje
        v=0;
        omega=k_alpha*alpha;     
        omega = max(min(omega, omega_max), -omega_max);

    else
        v=k_v*v_nom;
        omega=0;
    
    end
    wi=1/c*(v-omega*L/2);
    wd=1/c*(v+omega*L/2);

end