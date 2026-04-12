function [flag]=startsCheck(obs,targets)
flag=true;
    for i=1:length(obs)
        for k=1:size(targets,1)
            if(sqrt((targets(k,1)-obs(i).x)^2+(targets(k,2)-obs(i).y)^2)<=obs(i).r)
                fprintf("El punto inicial seleccionado (%d,%d) esta dentro de un objeto \n ",targets(k,1),targets(k,2))
                flag=false;
            end
        end
    end


end