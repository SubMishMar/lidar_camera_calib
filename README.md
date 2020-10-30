## PBPC-Cal
3D-Lidar Camera Calibration using  edge Point to Backprojected Plane Constraint

## Related Papers
1. [Extrinsic Calibration of a 3D-LIDAR and a Camera](https://arxiv.org/abs/2003.01213).
2. [Experimental Evaluation of 3D-LIDAR Camera Extrinsic Calibration](https://arxiv.org/abs/2007.01959).

## Disclaimer
This code is experimental. The backend is easy to understand and one can read the include files in the backend folder to better understand the geometric constraints employed.

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
