# CarND-Kidnapped-Vehicle-Project
## Project Description
A vehicle has been kidnapped and transported to a new location, but with a map of this location, a GPS estimate of its initial location, and sensor and control data, this project implements a 2 dimensional particle filter for tracking this vehicle.

## Project files
The project includes the following folder/files:
- illustrations - the folder with pictures for README.md.
- data – the folder with samples of:
    
    - control data ('control_data.txt'): vehicle speed (in meters per second), vehicle yaw rate (in radians per second);
    - true vehicle positions ('gt_data.txt'): x coordinate, y coordinate, yaw;
    - map data ('map_data.txt'): x coordinate, y coordinate, landmark id;
    - [observation data](https://github.com/udacity/CarND-Kidnapped-Vehicle-Project/tree/master/data/observation): x distance to the landmark in meters (right is positive) relative to vehicle, y distance to the landmark in meters (forward is positive) relative to vehicle.
    - output data ('output.txt') - example of calculated x coordinate, y coordinate and yaw by using particle filter. 
- src - the folder with c++ files with Particle Filter implementation.
- CMakeLists.txt - the file for building program.

## Results
The output results are introduced in 'output.txt' file. Additionaly.

Here is the picture showing map landmarks, true and predicted vehicle positions:

<img src="https://github.com/SergeiDm/CarND-Kidnapped-Vehicle-Project/blob/master/illustration/landmarks_and_positions.png" width="800" height="400"/>
