# Particle Filter Implementation
As part of the second mini project for Introduction to Computational Robotics, we designed an architecture for a particle filter implementation. Given a map, our challenge was to localize the robot on this map by using lidar and odometry readings and provide a pose estimation that would be used in a path planning algorithm or similar process. Our current implementation is not quite functioning, but all of the separate pieces of the particle filter are in place and require further testing to identify where the algorithm is behving incorrectly.

## Code Architecture diagram [CHARLIE]

![Alt Text](https://github.com/ksoltan/robot_localization/blob/master/robot_localizer/videos/particlefilter_codearchitecture.png)

## How did we do? RESULT SECTION (Some animations or results)

## Motion Model
Each particle needs to respond to the robot's movement which is represented by a change in the base_link -> odom transform. The position and orientation odom change is used to propagate each particle within a hypothetical base_link frame centered at each particle's pose (illustrated below). This propagation step can also include noise to account for the discrepancies in the odometry readings.

![Alt Text](https://github.com/ksoltan/robot_localization/blob/master/robot_localizer/videos/particle_propagation.gif)

Above, the blue pose arrows represent the robot's odometry readings while the yellow poses are particles that follow the change in odom. As the robot turns, each particle performs the same rotation in its own frame of reference.

Particle propagation without any noise.
- ParticleFilter: Step by step animations: propagate multiple particles, update with weights (maybe make markers instead of posearray and show weight of particles with different sized arrows, but this should not be a priority), resample [KATYA].

## SensorModel
Once particles have been propogated forward, we need to find the error between the obstacles sensed by the robot and the obstacles near each particle in the map.

![Alt Text](https://github.com/ksoltan/robot_localization/blob/master/robot_localizer/videos/error_validation_fixed.png)

Above, you see an example of how this looks. The robot (unseen) has four detected obstacles, each one meter away from it at 0, 90, 180, and 270 degrees. The green dots are projections that represent where those obstacles should be in relation to a particle (red). However, you can see that the obstacles around the particle aren't located at those positions. 

Our SensorModel makes use of the MapModel's occupancy field to get the distance to each projection's nearest obstacle. The radius of that distance is shown in pale blue around the green dots. For example, you can see that the model knows the top projection is much farther from its nearest obstacle (has greater "error") than the projection to the right (which has less error).

While the image only shows four projections, SensorModel as it stands actually obtains the error for 7 different scans per particle. It then calculates the overall likelihood of that particle using the errors from each scan. The more scans with high errors, the less likely that particle is to be accurate.

## Design Choices
Our goal was to develop and test each discrete step of the particle filter separately. The approach was to devise a high-level architecture and fill in functionality based on unit tests. Additionally, we aimed to start with the most simplest implementation possible, introducing better probability functions and noise models, which would simply consistute a change in the class representing the impacted step.

- Unittesting! [KATYA]
  - not useful for everything. Maybe a better thing would be to get things working separately and then stick them together. Also, not super great for noise...
  - Initial code architecture to modularize things
- Odometry-based movement vs timing-based movement. -> wanted to do simplest thing, but really didn't focus correctly [CHARLIE]

## Challenges [KATYA]
- Transforms....what the heck is going on...but we figured it out! ish....map ->odom->base_link (how long it took us to ask that question...)
- figuring out depth of this project: conceptually, totally get it. Implementation-wise, what...?! A lot of unknown players like tf...

## What would we improve? [CHARLIE]
- Process: do actual simplest model rather than filling in complex code architecture.
- PF: actual likelihood functions. Mathematical improvements rather than in working.

## Interesting lessons? [CHARLIE UND KATYA]
- most direct thing you understand, may not be the best ultimately.
- unittesting was useful for some things, some bugs, but not as useful for the more basic ones because you were testing your concept of what was there and not what was there.
- NEVER USE DEGREES. ALWAYS U SE RADIANS. USE DEGREES ONLY IN PRINT STATEMENTS.
- same type of python...would be good. Initial workspace assumptions!
