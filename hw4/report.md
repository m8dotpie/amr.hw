# Assigment 4

Author: Igor Alentev

## Approach

To solve the given task we need to answer several questions.

1. How to convert laser data into 3D points in the world?
2. How to project 3D points of the world onto the camera plane

Since we are given information, that there is no transformation between camera and laser frames, we can skip
transformations **to** and **from** the world frame and operate directly in the camera frame. From the information
about the laser scanner we can conclude, that it scans the space in plane from $-\pi$ to $\pi$ with some angle step.

Having information about the laser we can convert depth information to the point on the plane as

$$[R \cdot \cos{\theta}, R\cdot\sin{\theta}, 0]$$

Where $R$ is the given depth information and $\theta$ is current angle.

Having the point described in the 3D coordinates of the camera frame all what is left only a projection onto the
camera plane. It is simple as well, since projection matrix is contained in the camera information published in ROS. Therefore we have everything needed for transformation.

## Fine tuning

The biggest issue and pain was the fact that camera and laser data have different order of axes. After some investigation I have found right order. Moreover, since the point lies within the $XY$ plane of the camera frame, 
we can specify $z$ coordinate as $0$. However, I have decided to use this variable to store information
about the depth.

I have decided to color points differently based on the distance to them. Depth information saved on the previous step can be used for this. I have normalised the distance to the segment $[0;255]$ (color limits) and set color vector as $[255 - d_{norm}; 0; d_{norm}]$. Also I have used some offset to shift colors to blue.

The need for the offset was due to the problem that laser range is much higher than the range of the camera. As a result, sometimes we can see that laser detects information at the distance, however camera does not see it. Depth information I have used to color points according to the distance. As a result if the point is further, it has a bit more cold color, while closer points closer to the red.