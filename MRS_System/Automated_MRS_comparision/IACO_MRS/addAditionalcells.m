function dataRecalc = addAditionalcells(dataRecalc, path_robot_i)
endNode = path_robot_i(end,:);
lastUbication = dataRecalc.pos(end,:);
dataRecalc.t = [dataRecalc.t; dataRecalc.t(end) + 1;dataRecalc.t(end)+1;dataRecalc.t(end)+1];
%dataRecalc.pos = [dataRecalc.pos; lastUbication ;lastUbication + [1,0]; lastUbication + [0,1]];

v = lastUbication - endNode;
inv = [0 1;-1 0];
u = v * inv; %%Vector perpendicular
u = u/norm(u);
p1 = lastUbication + 0.5*u;
p2 = lastUbication - 0.5*u;

p1 = evaluatePoint(p1,endNode,lastUbication);
p2 = evaluatePoint(p2,endNode,lastUbication);
dataRecalc.pos = [dataRecalc.pos;lastUbication ;p1;p2];

end


function [point] = evaluatePoint(point,endNode,originalPoint)

    possiblePoints = [];
    p1 = [ceil(point(1)), round(point(2))];
    p2 = [round(point(1)), ceil(point(2))];
    p3 = [floor(point(1)), round(point(2))];
    p4 = [round(point(1)), floor(point(2))];
    availPts = [p1;p2;p3;p4];
    for k=1:4
        if norm(availPts(k,:)-originalPoint) ~= 0
            possiblePoints = [possiblePoints;availPts(k,:)];
        end

    end
    d = inf;
    %d = 0;
    for i = 1: size(possiblePoints,1)
        di=norm(possiblePoints(i,:)-endNode);
        if di<d %di>d
            d = di;
            point = possiblePoints(i,:);
        end
    end

end