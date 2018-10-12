# Particle Filter Implementation
As part of the second mini project for Introduction to Computational Robotics, we designed an architecture for a particle filter implementation. Given a map, our challenge was to localize the robot on this map by using lidar and odometry readings and provide a pose estimation that would be used in a path planning algorithm or similar process. Our current implementation is not quite functioning, but all of the separate pieces of the particle filter are in place and require further testing to identify where the algorithm is behaving incorrectly.

## Code Architecture
Our approach to the particle filter consisted of one main ParticleFilter node which contained a MapModel, MotionModel, and SensorModel, and handled all subscription and publicationt to topics, as well as passing around a list of particle objects (ParticleDistribution). This architecture is depicted in the diagram below:

![Alt Text](https://github.com/ksoltan/robot_localization/blob/master/robot_localizer/videos/particlefilter_codearchitecture.png)

The MotionModel class handles the changing odometry, tracking when the robot has moved far enough to merit updating the filter, and propagating particles to reflect this change.

The SensorModel is responsible for projecting the lidar data into each particle's frame and calculating the likelihood that the scan came from a given particle.

The MapModel provides an interface to the given OccupancyField class which stores information about the map and how far obstacles are from any given point on the map. The MapModel is used by SensorModel for likelihood calculations.

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

## Design Choices
Our goal was to develop and test each discrete step of the particle filter separately. The approach was to devise a high-level architecture and fill in functionality based on unit tests. Additionally, we aimed to start with the most simplest implementation possible, introducing better probability functions and noise models, which would simply consistute a change in the class representing the impacted step.

- Unittesting! [KATYA]
  - not useful for everything. Maybe a better thing would be to get things working separately and then stick them together. Also, not super great for noise...
  - Initial code architecture to modularize things
  
- Odometry-based movement vs timing-based movement. -> wanted to do simplest thing, but really didn't focus correctly [CHARLIE]

### Odometry-based vs time-based movement (@katya, check me on this, I'm a little rusty)
We initially set out on this project believing timing-based movement would be the simplest method of movement to implement in order to propagate particles. However, this actually introduced unneeded complexity into the system. By implementing timing-based movement, we would have created yet another error-prone coordinate frame, that would certainly have mapped inaccurately to the odometry and map frames. While at first timing-based movement seemed straight-forward and approachable, it wasn't the right focus for the problem we were working on, and it was worth making the switch.

## Challenges [KATYA]
- Transforms....what the heck is going on...but we figured it out! ish....map ->odom->base_link (how long it took us to ask that question...)
- figuring out depth of this project: conceptually, totally get it. Implementation-wise, what...?! A lot of unknown players like tf...

## What would we improve?

There are two main improvements we'd like to have in this project:

1. MVP vs. Complex Code Architecture
2. Conceptual focus rather than implementation struggle

### MVP vs. Complex Code Architecture
We approached our particle filter project with a fully laid out code architecture, with different functions elegantly laid out in their relevant classes, and a confident yet somewhat vague idea of how they would all interact.

In retrospect, this may not have been the best idea. We ended up filling in the classes and encountering problems in several files at once, many of which had to be tracked down a few levels to find some fundamental issue. The original architecture would have looked beautiful in a final product, but it was quite the headache as a first pass. If we were to build this again, we think it would have been better to put ideas together in one place, and separate functionality out into different classes once we had a better idea of how they would intuitively group, and what fundamental issues we might encounter across the full implementation.

### Conceptual focus rather than implementation struggle
This leads us to the next woe: Because we had to spend so much time debugging and thinking through implementation, we weren't able to focus on the more interesting mathematical aspects of particle filters. It would have been much more interesting to dive in the weeds of probability algorithms and their computational strengths and limitations, which we may have been able to do with a less ambitious MVP.

## Interesting lessons?
We learned several interesting lessons from this project:

- The most direct thing you understand may not be the best, ultimately.
  - Like written above, we initally went for timing-based movement because it seemed more conceptually accessible. It would actually have complicated the project quite a bit.
- Ask for help early to avoid rabbit-hole timesinks
  - I (Charlie) have run into this problem in multiple projects, where lack of context for the problem I'm working on leads me to spend a lot of time hacking away at something with a simple solution. A conversation with a professor early on about even the most general aspects of the project can save way more time than a trouble-shooting conversation later, once you're already at the bottom of the well.


- unittesting was useful for some things, some bugs, but not as useful for the more basic ones because you were testing your concept of what was there and not what was there.


Additionally, here's a small list of some good, basic implementation lessons for future projects:
1. NEVER USE DEGREES. It's more intuitive, sure, but you will absolutely lose track of your conversions. Use default radians.
2. Check that you have the same workspace as your partner. More specifically, the same version of python.


