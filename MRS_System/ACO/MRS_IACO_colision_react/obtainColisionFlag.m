function [colision] = obtainColisionFlag(dataRecalc, evaluatedPoint, time)
    colision = false;
    
    for k = 1: length(dataRecalc.t)
        if (dataRecalc.t(k)-1<=time)&&(dataRecalc.t(k)+1>=time) && dataRecalc.pos(k,1)==evaluatedPoint(1) && ...
            dataRecalc.pos(k,2)==evaluatedPoint(2)
            
            colision = true;
            return
        end
    end
end