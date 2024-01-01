# 3D-Reconstruction-of-Architectural-Structures-from-Stereoscopic-Images
This repository focuses on creating a 3D model from two 2D images of a castle using advanced computer vision techniques.

## First image:

![image](data/0014_2.png)

## Second image:

![image](data/0017_2.png)


## Features

- **3D Transformation Estimation:** Computed the transformation between the two images.
- **Calibrated Projections:** Utilized known intrinsic camera parameters.
- **Essential Matrix Estimation:** Estimated the essential matrix using SIFT feature matches.
- **RANSAC Integration:** Employed RANSAC for a robust estimation of the transformation.

## SIFT FEATURES

![image](https://github.com/Saibernard/3D-Reconstruction-of-Architectural-Structures-from-Stereoscopic-Images/assets/112599512/ca846260-95bd-4f96-89b6-d158e77101b3)

![image](https://github.com/Saibernard/3D-Reconstruction-of-Architectural-Structures-from-Stereoscopic-Images/assets/112599512/0a22a9c2-9e38-4b0e-a6c9-520b002f66dc)


- **Epipolar Geometry Visualization:** Plotted epipolar lines to visualize the correspondence between the two images.

![image](https://github.com/Saibernard/3D-Reconstruction-of-Architectural-Structures-from-Stereoscopic-Images/assets/112599512/3a4cb404-454a-400e-85d2-a610f4339988)

- **Camera Pose Recovery:** Extracted the rotation and translation between the two camera positions.
- **3D Triangulation:** Triangulated the matched feature points to derive 3D points.
- **Reprojection Verification:** Confirmed the accuracy of the 3D reconstruction by reprojecting the 3D points onto the 2D images.

![image](https://github.com/Saibernard/3D-Reconstruction-of-Architectural-Structures-from-Stereoscopic-Images/assets/112599512/1f035632-80ef-409f-ae5d-67072a1d308c)

![image](https://github.com/Saibernard/3D-Reconstruction-of-Architectural-Structures-from-Stereoscopic-Images/assets/112599512/ed618710-d72c-4692-8eea-085449db9a21)
