# Path Planning Project #

## Rubric Points ##

### Valid Trajectories ###

#### The car is able to drive at least 4.32 miles without incident.. ####

In the link https://youtu.be/36Z3fnfLm3A, I've upload a full lap around the track of my path planning project.

#### The car drives according to the speed limit. #### 

The code generates several trajectories, with varying distances, end lanes, and end speeds. I score each trajectory according to kinematic and lane parameters. The trajectories that infringe the speed limit are awarded a very high penalty, effectively making it inellegible.

#### Max Acceleration and Jerk are not Exceeded. ####

Same as before, the trajectories are evaluated for acceleration and jerk, and the ones that go over a given threshold are heavily penalized.

#### Car does not have collisions. ####

The tentative trajectories are analyzed for collisions. I simulate the others cars position during the time horizon of the trajectory, extrapolating their position using the sensed velocity. If a collision is detected, the trajectory is completely removed from the list of tentative trajectories.

#### The car stays in its lane, except for the time between changing lanes. ####

When generating the trajectories, we do it in the Frenet space. This naturally minimizes the time spent outside the lane. We also penalize trajectories according to amount of time spent outside the lane, further enforcing this policy.


#### The car is able to change lanes ####

Because I generate trajectories that have end points in all 3 lanes, and I give a higher reward for trajectories with higher speeds, the algorithm decides when it's best to change lane to maintain a high velocity, respecting the other constraints.

### Reflection ###

#### Generating Single Trajectory ####

In order to generate the paths, we use the following approach:

1) We use the previous existing trajectory. We determine a "stitch point" in this trajectory, the point where we are going to merge with the new trajectory. From this stitch point, we extract position, velocity and acceleration. Then, we generate a jerk minimizing trajectory with start configuration using the stitch point configuration as a start configuration, and an end configuration that we have determined. This is done in Frenet coordinates. We use the JMT function that was provided by Udacity.

2) We merge the old trajectory up to the stitch point with the new trajectory. We convert this merged trajectory to Cartesian coordinates. We then create a spline by fitting it through sampled points from this trajectory, in order to correct any jerk offset. This results in a XY trajectory that is compatible with the current trajectory but achieves a new goal.

By adjusting the stitch point, we adjust the tradeoff between responsiveness and stability of the path planning algorithm. The code for generating paths can be found on the [trajectory.h](src/trajectory.h) file.

#### Choosing the next trajectory ####

We use the previous method to generate several trajectories, with varying end configurations (end speed, end lane, end position). We then check these trajectories for collision, extrapolating the other cars position during the trajectory time horizon. This data is provided by the sensor fusion. In order to be extra cautious, we assume the cars will slow down and we increase the collision distance in time to account for the increased uncertainty of the cars position. This results in a conservative path planning. All trajectories that are determined to have a collision are eliminated from the list of tentative trajectories.

The remaining trajectories are then scored according to kinematic and lane following parameters. We reward achieving a higher velocity, while penalizing the acceleration and jerk, and angular velocity and acceleration. We also penalize trajectories that stay too much time outside the lanes and outside the street. This approach doesn't use a state machine at all. the algorithm just chooses the trajectory that will best accomplish its goals, which can be summarized by achieving a higher velocity, while staying inside the proposed constraints.



