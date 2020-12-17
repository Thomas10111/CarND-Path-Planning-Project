# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Behavior
The behavior of the car is based on cost. Each lane has a cost. The cost of a lane is based on the distance to the closest car in front
and the speed this car. Each action has a cost, if the action would lead to a crash or get very
close to another car the cost for this action is high. All the costs are added and finally the car chooses
the action with the lowest cost. To enable lane changes over several lanes the lane that is directly left or right of the current lane
gets the lowest value of all the other lanes that are further left or right. So, the car will step by step move to the lane
with the lowest cost.


### Path generation
The trajectory that is send to the simulator contains 50 points with x,y coordinates given in world coordinates.
For this project most of Aarons code presented in the Q&A was used for path planing. 
Five waypoints are used to generate a new continuous path. 
The five waypoints that are used are:
- the two last points of the current path
- waypoints at 30m, 60m, and 90m ahead (these waypoints need to be transformed into global x,y-coordinates)
   
These five points need to be transformed into the car coordinate system by using the current position and orientation of the car.
With a library from http://kluge.in-chemnitz.de/opensource/spline/ a spline through these five points is calculated. Now,
an x-coordinates can be passed into the calculated function and the function will output a y-coordinate in a way that all
the points lie on a smooth line.  

Points in the current trajectory that were reached already are removed from the trajectory. So, new points
from the newly created trajectory can be appended to the end of the trajectory. Therefore, the last point of the trajectory is
taken as reference for the x-coordinate. The distance the car can travel with the current speed in 0.02s is used as new x-coordinate.
To determine the y-coordinate of the new point the x-coordinate is feed into our spline function. This is repeated until the trajectory
contains 50 points again. Finally the 50 points long vector is send to the simulator. 




