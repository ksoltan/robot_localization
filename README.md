# Particle Filter Implementation
As part of the second mini project for Introduction to Computational Robotics, we designed an architecture for a particle filter implementation. Given a map, our challenge was to localize the robot on this map by using lidar and odometry readings and provide a pose estimation that would be used in a path planning algorithm or similar process. Our current implementation is not quite functioning, but all of the separate pieces of the particle filter are in place and require further testing to identify where the algorithm is behaving incorrectly.

## Code Architecture
Our approach to the particle filter consisted of one main ParticleFilter node which contained a MapModel, MotionModel, and SensorModel, and handled all subscription and publicationt to topics, as well as passing around a list of particle objects (ParticleDistribution). This architecture is depicted in the diagram below:

![Alt Text](https://github.com/ksoltan/robot_localization/blob/master/robot_localizer/videos/particlefilter_codearchitecture.png)

The MotionModel class handles the changing odometry, tracking when the robot has moved far enough to merit updating the filter, and propagating particles to reflect this change.

The SensorModel is responsible for calculating the likelihood that a scan came from a given particle.

The MapModel provides an interface to the given OccupancyField class which stores information about the map and how far obstacles are from any given point on the map. The MapModel projects the lidar data into each particle's frame and is used by SensorModel for likelihood calculations.

The ParticleDistribution class contains a list of particles as well as some helper functions that resample the particles based on their weights as well as normalizing the weights.

## Filtering Steps
The particle filter updates when the robot has moved or turned a certain threshold distance. First, each particle in the distribution is propagated to reflect the change in position. Next, each particle's weight is updated based on how likely the lidar readings are from its pose. These weights are then normalized and the entire distribution is resampled based on the new probability distribution. A weighted average of all of the particle's poses generates an estimated robot pose which is then used to update the odom -> map transform.

### Propagation (MotionModel)
Each particle needs to respond to the robot's movement which is represented by a change in the base_link -> odom transform. The odometry's position and orientation change is used to propagate each particle within a hypothetical base_link frame centered at each particle's pose, illustrated below. 

![Alt Text](https://github.com/ksoltan/robot_localization/blob/master/robot_localizer/videos/particle_propagation.gif)

Above, the blue pose arrows represent the robot's odometry readings while the yellow poses are particles that mimic the change in odom. As the robot turns, each particle performs the same rotation in its own frame of reference. This propagation step can also include noise to account for the discrepancies in the odometry readings.

### Likelihood Update (SensorModel)
Once particles have been propagated, the likelihood of each particle given new sensor readings is determined by finding the error between the obstacles sensed by the robot and the obstacles near each particle in the map. Below is an illustration of this process. 

![Alt Text](https://github.com/ksoltan/robot_localization/blob/master/robot_localizer/videos/error_validation_fixed.png)

The robot (unseen) has four detected obstacles, each one meter away at 0, 90, 180, and 270 degrees. The green dots are projections that represent where those obstacles should be in relation to one particle (red). The pale blue circles have a radius corresponding to the error between the projected reading and the nearest obstacle at that point (found using the OccupancyField). The likelihood is inversely proportional to the distance: the error at 0 and 180 degrees gives the particle a low propbability, while the relatively low errors at 90 and 270 degrees give it a likelihood closer to 1.

This process is repeated for a configurable number of angles of the lidar scan for each particle. The overall likelihood of each particle is calculated as the sum of the cubes of these errors. The cube is used to give low errors a higher weight in the total probability, rewarding more correct readings and not penalizing too steeply for high-error readings.

### Resampling (ParticleDistribution)
With new particle likelihoods, the particle distribution is updated by resampling particles using the new probability distribution.

## Design Choices and Challenges
### Unit Test Driven
Our goal was to develop and test each discrete step of the particle filter separately and to later integrate the pieces. Our approach was to devise a high-level modular architecture and fill in functionality based on unit tests. Additionally, we aimed to start with the most simplest implementation possible, introducing better probability functions and noise models, which would simply consistute a change in the class representing the impacted step.

While a unit testing framework helped catch some poor implementations of mostly mathematical functions, such as resampling and propagation, it was difficult to test the random probability updates as well as the intuitive "what should happen" at each step. Towards the end of the project, we began to implement parts of the filter in semi-isolation and use rviz more heavily to visually validate each function. This helped us identify some fundamental problems, like strange angles due to radian/degree conversions and understand how the transforms between coordinate system should behave.

### Odometry-based vs Time-based Movement
Our initial approach to the project was to teleoperate the robot and predict the robot's movement using a desired velocity and time-based model. We believed this would be the simplest method of implementation since we had worked extensively with cmd_vel before and we could work in a single frame (map). However, this actually introduced unneeded complexity into the system because we still needed to have an odom -> map transform. Additionally, the time-based model is not by any means a reliable prediction of how the robot has actually moved. While at first timing-based movement seemed straight-forward and approachable, it wasn't the right focus for the problem we were working on, and it was worth making the switch,.

## Future Improvements
There are two main improvements we would like to have in this project, besides finding the current bug in our implementation and seeing the particles converge on an accurate predicted pose, which center around our process and approach.

### MVP vs. Complex Code Architecture
Perhaps a better approach to the project would have been to start with implementing very rough filtering operations on one or two particles, seeing them through the entire algorithm, and then creating a better architecture that scales to more particles. The most challenging part of our approach was that although we had a fully laid-out architecture, we were not entirely sure of all of the components that would have to interact or what fundamental issues we might encounter across the full implementation.

### Conceptual vs. Implementational Focus
Because we spent so much time debugging and thinking through a complex interaction of components, we were not able to focus on the more interesting mathematical aspects of particle filters. It would have been much more interesting to dive into the weeds of probability algorithms and their computational strengths and limitations, which we may have been able to do with a less ambitious MVP. Our current propagation model does not include noise, and the likelihood updates are based on an average of probabilities rather than at least a Gaussian distribution.

## Interesting lessons
We learned several interesting lessons throughout this project:

- The most direct thing you understand may not be the best, ultimately.
  - Like written above, we initally went for timing-based movement because it seemed more conceptually accessible. It actually have complicated the project quite a bit.
- Ask for help early to avoid rabbit-hole timesinks
  - We have run into this problem in multiple projects, where lack of context for the problems we are working on leads us to spend a lot of time hacking away at something with a simple solution. A conversation with a professor early on about even the most general aspects of the project can save a lot more time than a trouble-shooting conversation later, once you are already at the bottom of the well.
- Unit testing is only as good as your overall understanding of the system.
  - Because we did not have a detailed understanding of each of the necessary elements, we did not design very good unit tests. One way we could have had a better picture of what we should be testing for is running the built-in particle filter and writing tests based on its output. This approach would have also created a built-in benchmark for our algorith.


Additionally, here is a small list of some good, basic implementation lessons for future projects:
1. NEVER USE DEGREES. It is more intuitive, sure, but you will absolutely lose track of your conversions. Use default radians and only print using degrees for readability.
2. Check that you have the same workspace as your partner. More specifically, the same version of python.


