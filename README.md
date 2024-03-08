# Project LegionAir

## Introduction
This document summarizes the project proposal for the Swarm Robotics course Research project. 
The project aims to emulate a swarm formation of a roman legion where if a single comrade falls out of formation there is a readjustment of the 
whole formation such that the coverage can be maximized.

Swarming behavior using vision is quite common in the animal kingdom, birds flock together while migrating to locations and maintain formation at the same time just based on limited field of view vision sensors. Our motivation is to use a similar limited FOV swarm of drones to track a mother ship drone in the center at the same time maintaining the formation while the the mother ship guides the flock to the designated target. The second stage of the project involves emulating Roman legion tactics of replacing and maintaining coverage of troops. Roman legions functioned using continuous funneling of troops to the front lines so that exhausted troops can rest or recuperate while at the same time fallen soldiers can be replaced. 
Our objective would be to emulate the second formation where one of the drones in the flock looses track or is sent on a scouting mission ahead, 
while the slower moving mother ship and it's flock reorients itself to account for the missing drone.

* The below image shows the stages through which the swarm will execute a coverage compensation maneuver using just visual data of the tracked target
![](assets/repn1.png)

* The below image shows the change in communication graphs of the drones, here nodes means drones:
![](assets/GraphRepn.png)


* The next image shows the final formation:
![](assets/repn2.png)

## Sensors and Hardware

Each of the follower and a leader drone uses Visual Inertial Odometry taken out from the fisheye camera mounted on the drones for its local position estimate. The follower drones tra
