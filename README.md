# What was our goal? (Requirements) [KATYA]
- Particle Filter
- this is specifically for estimating pose of robit given a map.

# Code Architecture diagram [CHARLIE]

# How did we do? RESULT SECTION (Some animations or results)
- SensorModel: project scan onto ONE particle, how do these match up with actual obstacles (errors) [CHARLIE]
- MotionModel: particle propagation, introduce some noise. [KATYA]

## Motion Model
![Alt Text](https://github.com/ksoltan/robot_localization/blob/master/robot_localizer/videos/particle_propagation.gif)


Particle propagation without any noise.
- ParticleFilter: Step by step animations: propagate multiple particles, update with weights (maybe make markers instead of posearray and show weight of particles with different sized arrows, but this should not be a priority), resample [KATYA].

# Design Decision
- Unittesting! [KATYA]
  - not useful for everything. Maybe a better thing would be to get things working separately and then stick them together. Also, not super great for noise...
  - Initial code architecture to modularize things
- Odometry-based movement vs timing-based movement. -> wanted to do simplest thing, but really didn't focus correctly [CHARLIE]

# Challenges [KATYA]
- Transforms....what the heck is going on...but we figured it out! ish....map ->odom->base_link (how long it took us to ask that question...)
- figuring out depth of this project: conceptually, totally get it. Implementation-wise, what...?! A lot of unknown players like tf...

# What would we improve? [CHARLIE]
- Process: do actual simplest model rather than filling in complex code architecture.
- PF: actual likelihood functions. Mathematical improvements rather than in working.

# Interesting lessons? [CHARLIE UND KATYA]
- most direct thing you understand, may not be the best ultimately.
- unittesting was useful for some things, some bugs, but not as useful for the more basic ones because you were testing your concept of what was there and not what was there.
- NEVER USE DEGREES. ALWAYS U SE RADIANS. USE DEGREES ONLY IN PRINT STATEMENTS.
- same type of python...would be good. Initial workspace assumptions!
