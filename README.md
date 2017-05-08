# CarND-Kidnapped-Vehicle-Project
## Project Description
A vehicle has been kidnapped and transported to a new location, but with a map of this location, a GPS estimate of its initial location, and sensor and control data, this project implements a 2 dimensional particle filter for tracking this vehicle.

## Project files
The project includes the following folder/files:
- illustrations - the folder with pictures for README.md.
- data – the folder with samples of:
    control data ('control_data.txt'), true positions measurement data from both LIDAR and RADAR.
- output_data - the folder with output files, produced by the project pipeline.
- src - the folder with c++ files with Unscented Kalman Filter algorithm.
- CMakeLists.txt - the file for building program.

## Compiling and running the project
The project can be compiled and run by using, for example, the following command:

`mkdir build && cd build`

`cmake .. && make`

`./UnscentedKF path/to/input.txt path/to/output.txt`

The project uses Eigen library, so for compilation Eigen library files must be put in 'src' folder.

## Results
The output results introduced in 'output_data' folder. Additionaly, here are RMSE values (calculated for 2D position and 2D velocity):

<img src="https://github.com/SergeiDm/CarND-Unscented-Kalman-Filter/blob/master/illustrations/RMSE.png" width="270" height="100"/>

It is possible to run the project in three different modes: considering laser only, with considering radar only, or with using both. There are 'use_laser_' and 'use_radar_' flags in 'ukf.cpp'. 

Here is comparison performance Laser, Radar and Sensor Fusion for [obj_pose-laser-radar-synthetic-input.txt](https://github.com/SergeiDm/CarND-Unscented-Kalman-Filter/blob/master/input_data/obj_pose-laser-radar-synthetic-input.txt):

<img src="https://github.com/SergeiDm/CarND-Unscented-Kalman-Filter/blob/master/illustrations/Sensor_fusion.png" width="270" height="100"/>

As shown in the picture above, Sensor fusion gives better results.

The model used in the project has hyperparameters which were adjusted:
- standard deviation longitudinal acceleration in m/s^2
- standard deviation yaw acceleration in rad/s^2

To control their adjustment 'Normalized Innovation Squared' check was used. NIS calculation is based on the difference between the predicted measurement and the actual measurement.

The following picture shows that NIS under chi-square 5% point most of the time:

<img src="https://github.com/SergeiDm/CarND-Unscented-Kalman-Filter/blob/master/illustrations/NIS.png" width="500" height="400"/>
