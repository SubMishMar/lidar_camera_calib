## PBPC-Cal [Extrinsic Calibration of Camera and Lidar]
3D-Lidar Camera Calibration using  edge Point to Backprojected Plane Constraint

## Related Papers
1. [Extrinsic Calibration of a 3D-LIDAR and a Camera](https://arxiv.org/abs/2003.01213). [IV-2020 Presentation](https://www.youtube.com/watch?v=GyoPNhvupfg&t=1s)
2. [Experimental Evaluation of 3D-LIDAR Camera Extrinsic Calibration](https://arxiv.org/abs/2007.01959). [IROS-2020 Presentation](https://www.youtube.com/watch?v=cnBgSQyAj5E)

## Disclaimer
This code is experimental. The backend is easy to grasp and one may read the include files in the backend folder to better understand the geometric constraints used for extrinsic calibration. The front-end might require special tweaking. One may use this code-base as a reference for implementing their own calibration algorithm. I, however, provide a working dataset for running the code. The instructions for running it is provided below.

## Working
A planar target's plane and edges (also called features) are detected across both the sensing modalities and geometric constraints are use to related these features using the unknown extrinsic calibration between both the sensors. The geometric constraint is squared and summed over several observations and an optimization problem is formed which on minimization yeilds the unknown extrinsic calibration parameters between the sensors. 

The code base has two different modules, viz. the front-end whose sub-modules detect the planar target's planes and edges in both the sensors and the backend which does optimization using these detected features.

## Sensors 
We have validated this code for the following sensor pairs:
1. Basler Camera (ace2) <-> Ouster-64 Channel Lidar
2. Basler Camera (ace2)<-> Velodyne-32 (VLP-32C) Channel Lidar
3. Point-grey Camera (CM3) <-> Velodyne-32 (VLP-32C) Channel Lidar
4. Nerian Karmin2 Stereo Camera <-> Ouster-64 Channel Lidar
5. Nerian Karmin2 Stereo Camera <-> Velodyne-32 (VLP-32C) Channel Lidar

The frontend can be finicky about the sensors used but the mathematics in the backend remains the same. One can use our backend with their own sensor specific frontend which publishes lines and planes, from both the sensing modalities, as our front-end does.

## Software requirements
Implemented and tested on Ubuntu 16.04 systems. ROS-kinetic has been used as the middleware. 

Other requirements are:
1. Point Cloud Library 1.7
2. OpenCV, ships with ROS
3. Ceres Library

## Running the Code
In different terminals roslaunch as below:
1. `roslaunch random_frame_generator random_frame_generator_basler_os.launch`
2. `roslaunch target_detector target_detector_node_os.launch`
3. `roslaunch target_line_detector target_line_detector_node_os.launch`
4. `roslaunch image_line_detection basler_image_line_detection.launch`
5. `roslaunch calibration_backend_optimization calibration_test_initialization_basler_os.launch`

2,3,4 can be combined as one launch file. 

Then press `[ENTER]` on the terminal where you have launched 1. 

The calibration routine should run and generate results. Read the launch files to detemine the location of data, target and sensor config files.

## Validating Calibration Result
One may validate the calibration result by using the estimated extrinsic calibration parameters to project lidar points onto the image plane. 
In different terminals roslaunch as below:
1. `roslaunch random_frame_generator random_frame_generator_basler_os.launch`
2. `roslaunch projection basler_projection_node_os.launch`

Then press `[ENTER]` on the terminal where you have launched 1. 

