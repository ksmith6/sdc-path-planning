# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
Kelly Smith - August 19, 2017
## Approach

### Trajectory Generation
**Rubric: The car stays in its lane, except for the time between changing lanes**
**Rubric: The car stays in its lane, except for the time between changing lanes**

For this project, I attempted to build the trajectory generation routine several times by implementing basic lane following.  However, I could not eliminate glitches in the resulting trajectory caused by the `getXY` routine.

As a result, I opted to build the trajectory generation routine demonstrated by Udacity in the project walkthrough video to get pass that obstacle.

At a high level, it works by generating a spline defined by 5 anchor points, of which the first two define the initial orientation of the spline to maintain position acceleration and jerk continuity.  The last three anchor points are distant waypoints located 30, 60, and 90 meters in the distance in the desired lane.  All anchor points are based in Frenet coordinates and converted into global $$xy$$ coordinates.

Then points are sampled from this spline to maintain a queue of 50 upcoming points. 

On cold starts (initialization), the algorithm will be required to generate all 50 points.  On warm starts (previous path points available), the queue is pre-populated with the previous points.  Then the spline is sampled to fill the queue to its capacity (50 points).

### Speed Tracking
**Rubric: The car drives according to the speed limit.**

**Rubric: Max Acceleration and Jerk are not Exceeded.**

The vehicle will be initialized with a speed of zero, and it will accelerate at a constant acceleration (0.5g) until it achieves the maximum speed, approximately 49.5 mph.  It will continue at the maximum speed until it is necessary to slow down (deceleration at 0.5g) to prevent a collision with a car ahead.  Deceleration is constrained to 0.5g to satisfy acceleration and jerk limits.

Once our car is within a preset distance of a leader car, our car will begin decelerating to match the speed of the leader vehicle in order to prevent a collison.

While tracking the leader car's speed and maintaining a safe separation distance, our car will begin seeking lane change opportunities if we are forced to drive more than 3mph slower than the maximum speed.  If no lane change opportunities are available in the adjacent lanes, then the vehicle will simply continue to track the leader speed and wait for a future lane change opportunity. 

### Lane Changes
**Rubric: The car is able to change lanes.**

**Rubric: Car does not have collisions.**

**Rubric: The car stays in its lane, except for the time between changing lanes.**
 
In my implementation, the vehicle will only change lane if it is feasible (don't go off road or veer into opposing lanes) and if the desired lane is clear of vehicles within some distance of our vehicle's current position.

In my implementation, I wrote a function called `safeToEnterLane` to analyze the traffic in the desired lane.  If no vehicles in the desired lane are within some distance of our car's current position `s`, then it is considered to be _safe_ to execute a lane change.  Otherwise, the lane change is considered to be too risky due to the close proximity to other vehicles.

If the lane change is both **feasible** and **safe** to execute, then the lane change will be commanded to the trajectory generator.  The vehicle will prefer passing on the left versus passing on the right if both options are available.  This is done to produce a consistent behavior that minimizing surprises to other drivers.  In America, passing on the left is the norm, whereas passing on the right is less common and generally only done when a vehicle is impeding traffic in the fast lane (left lane).

If there is no car impeding our progress (`obstacleDetected = false`), then our car will consider making a lane change to move over to the far right lane as a courtesy to other drivers behind our car who may which to pass.  This behavior is designed to prevent our car from lingering in the left lane or in the middle lane without good reason.  Many jurisdictions have traffic laws which require all vehicles to remain in the right lane (slow lane) unless your passing another vehicle.  This behavior requires at least 50 meters of clearance in the next-most right lane before it will shift lanes rightward.  As a result, it will only perform this behavior when there are few other cars around in the slower lanes.


