function [startNode,goalNode, correctSelection] = generateNodes(startPosition,endPosition, Nrows, Ncolumns)
startNode = 0;
goalNode = 0;
correctSelection = false;

if startPosition(1) < 0 || startPosition(2) < 0 || ...
   endPosition(1)   < 0 || endPosition(2)   < 0
    
    fprintf("Negative coordinates are not supported\n")

    return 
end
if (startPosition(1) + 1 > Ncolumns) || (startPosition(2) + 1 > Nrows) ...
        ||(endPosition(1) + 1 > Ncolumns) || (endPosition(2) + 1 > Nrows) 

    fprintf("Points must be located inside the array size\n")
    return 
end
startNode = [startPosition(2) + 1, startPosition(1) + 1];
goalNode = [endPosition(2) + 1, endPosition(1) + 1];
correctSelection = true;

end