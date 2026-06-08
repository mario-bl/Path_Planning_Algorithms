Relative to multy robot path planning we have 3 folders, all of them are developed for 2 robots. 

  - ACO 
  - APF 
  - Automated_MRS_comparision


To run the simulations for the different APF strategis located inside MRS_System/APF it's only necesary to execute the *main_V0.m* file that is located inside each folder.

On the other hand for the differerent algorithms modifications located inside MRS_System/ACO to run each one of them  it's only necesary to execute *IACO_MRS_Simulation.m*


Automated_MRS_comparision folder is dedicated to run both most optimized algorithms of ACO and APF. 
To execute all the calculus you'll need to run *main.m* which is configured to make 5 simulations on 20 different scenarios.
If you wan't to shorten the number os differenmt stages (therefore reduce the time) you'll only need to modify the parameters:

  * diffStages = 20;
  * runs = 5;
