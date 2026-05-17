function [startNodes,goalNodes, correctSelection] = generateNodesV2(trajectories, mapSize)
    Nrows = mapSize(1);
    Ncolumns = mapSize(2);

    nTrayec = size(trajectories,2);
    startNodes = zeros(nTrayec, 2);
    goalNodes = zeros(nTrayec, 2);
    correctSelection = false;
    for i = 1:nTrayec
        startPosition = [trajectories(i).x0, trajectories(i).y0];
        endPosition = [trajectories(i).xObj, trajectories(i).yObj];
        if startPosition(1) < -1 || startPosition(2) < -1 || ...
           endPosition(1)   < -1 || endPosition(2)   < -1

            fprintf("Negative coordinates are not supported\n")
            return 
        end
        if (startPosition(1) + 1 > Ncolumns) || (startPosition(2) + 1 > Nrows) ...
                ||(endPosition(1) + 1 > Ncolumns) || (endPosition(2) + 1 > Nrows) 
        
            fprintf("Points must be located inside the array size\n")
            return 
        end
        startNodes(i,:) = [startPosition(2) + 2, startPosition(1) + 2];
        goalNodes(i,:) = [endPosition(2) + 2, endPosition(1) + 2];
    end
    correctSelection = true;

end