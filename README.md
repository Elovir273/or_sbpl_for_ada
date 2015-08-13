# or_sbpl_fo_ada

## Goal
This package is primary intended to be used to make an automatic mode switching with ADA.
It's an adaptation of or_sbpl for 7D ( 3 orientations, 3 rotations and the mode ). 

## Use
This package is using yaml-cpp and sbpl with multiple goals, called sbplmg or sbpl_multiple_goals :
it's a personnalized version of sbpl, which can do the search with multiple goals, giving an array of the goals id. 
You need to use sbpl_ams.py to call this package, as it creates a OpenRave Plugin. 
This file should be in prpy/src/prpy/planning or in or_sbpl_for_ada itself. 
Need yaml >= 0.5

You might need to do
mkdir build
cd build/
cmake ..
make
sudo make install to use sbplmg

## Info
sbpl_ams.py creates the planner and gives him the model of the robot and the goal ( or goals ) to achieve.
SBPLBasePlanner.cpp takes care of the initialisation, the differents calls to the sbpl planneur and sending back the results.
SBPLBasePlannerEnvironment.cpp takes care of the functions needed by the planner : getHeuristic, getSuccessors, etc.
It's the interface between world coordinate, actions and states ids ( which is the only thing the planner works with ).
When this file opens a new state, it gets a new id, which is id[precedent state] + 1, that the planner is gonna uses, and an index, which comes from the worlcoordinate and represent the grid coordinate. 

All the informations concerning the actions are within the yaml files in the yaml folder. actions2DOFs_s2.yaml means it's a file describing an action with 2 Degrees of Freedom with a step of 2 cm. It's loading different files depending on the distance to the closest goal to adapt the step size and so the speed of the search. The actions are described as only one point, while they could be an array of points ( see herb.py, on or_sbpl ). More points means more collision checking; we primary intend to be faster, not safer ( as it's the user giving all the movements input ). 
However, if you're using a robot who is not following the grid ( it's quite rare to do so ), you can create your own file (matlab might help ).

