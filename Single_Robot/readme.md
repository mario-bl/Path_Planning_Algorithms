For the single robot path planning we have them divided in two folders :

    - ACO, relative to Ant Colony Optimization inside 3 different approaches are made:
        * ACO_Classic: Classic aproach to the ACO path planning problem --> run the file *ACO_SR.m*
        * OACO: Improved path planning with line of sight optimization to remove unnecessary points --> run the file *OACO_SR.m* 
        * IACO: Best ACO option that also includes a new heuristic function --> run the file *IACO_SR.m* 
    

    - APF, relative to Artificial Potential Field inside we have 2 different folders:
        * OAPF: Takes the classic aproach to this algorithm and includes modifications to reduce jitter efect and local minima potential problems
        * IAPF: Combines OACO modifications with a new restritcion for the repulsive potential of the obstacles
        
        Run the file *main_V0.m* inside each of the folders to run the specific algorithm  
