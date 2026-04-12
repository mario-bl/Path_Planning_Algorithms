function [flag]=startPointCheck(obs,target)
flag=true;
    for i=1:length(obs)
        if(sqrt((target(1)-obs(i).x)^2+(target(2)-obs(i).y)^2)<=obs(i).r)
            fprintf("El punto final seleccionado (%d,%d) esta dentro de un objeto \n ",target(1),target(2))
            flag=false;
        end
        
    end


end