Relative to multy robot path planning we have 3 folders:
  --> ACO 
  --> APF 
  --> Automated_MRS_comparision
Due to the multi folders that ACO/APF have we explain on specific readmes how to use them. 
At this readme you'll find how to use/modify the Automated_MRS_comparision folder.

Automated_MRS_comparision folder is dedicated to run both most optimized algorithms of ACO and APF on a 2 robot system. 
To execute all the calculus you'll need to run main.m which is configured to make 5 simulations on 20 different scenarios.
If you wan't to shorten the number os differenmt stages (therefore reduce the time) you'll only need to modify the parameters:

  * diffStages = 20;
  * runs = 5;
