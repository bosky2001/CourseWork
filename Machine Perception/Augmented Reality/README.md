# Augmented-Reality-Implementation-Camera-Pose-Recovery-Virtual-Object-Placement
This repository dives into augmented reality (AR) by overlaying virtual objects onto a real-world video. Given a video with AprilTags and their coordinates, your goal is two-fold: 1) Determine camera pose using PnP with coplanar assumptions and P3P with the Procrustes problem. 2) Place virtual objects seamlessly in the scene. 


**Introduction:**

This repository invites you to delve into the world of augmented reality (AR) by designing an application that projects virtual object models into a real-world video sequence as if they coexist. The crux of the project is to deduce the camera's position and orientation in relation to a known object, paving the way for the seamless integration of virtual elements.

**Given Resources:**

**Video Sequence:** The video provided has an AprilTag embedded in each frame. Renowned in robotics, AprilTags are pivotal in discerning the camera's pose.

**Tag Details:** To streamline your workflow, the pixel coordinates of the four corners of the AprilTag are furnished, along with the tag's dimensions.

**Core Tasks:**

**Camera Pose Recovery:**

**Perspective-N-Point (PnP):** Utilizing the coplanar assumption, solve the PnP problem. This involves determining the camera's position and orientation by recognizing n points in the image and their corresponding world coordinates.

**Perspective-three-point (P3P) & Procrustes:** Solve the P3P problem, followed by the Procrustes problem. The P3P extracts the camera's pose from three 2D-3D point correspondences, while the Procrustes aligns two sets of points optimally.


**Virtual Object Placement:** Post-retrieval of the 3D relationship between the camera and the world, you'll have the leverage to embed arbitrary objects into the scene. Define the pixel positions to precisely dictate where these virtual objects should manifest.

![vis](https://github.com/bosky2001/CourseWork/blob/main/Machine%20Perception/Augmented%20Reality/code/Vis.png)

![VR_res-min](https://github.com/bosky2001/CourseWork/blob/main/Machine%20Perception/Augmented%20Reality/code/VR_res.gif)



