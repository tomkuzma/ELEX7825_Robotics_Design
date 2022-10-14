# ELEX7825_Robotics_Design
C++ labs and template for Robotic System Design Course

These labs are written in c++ and use homogeneous transformations to position figures in a 3D environment, then modelling a pinhole camera and using opencv's GUI features to control the camera position, the 3D objects and frames are projected onto a 2D focal point using matrix transformations.

# LAB 3
- Draw 5 cubic boxes with 5 cm sides to form a virtual robot as shown below. Also draw the world
coordinate system origin at the center of the lowest cube.
- Add a virtual camera object which determines the intrinsic and extrinsic camera matrixes for a
perspective pin-hole camera. For default values use an image size of 1000x600, or the size of your
webcam image, a central critical point, focal length of 3 mm, and position the camera at X=0 cm,
Y=0 cm, Z=50 cm. Orient the pitch/roll/yaw angles for the camera such that it is point at the robot.
- Link the trackbars X/Y/Z/R/P/Y/Focus to the virtual camera so changes in the trackbar values are
automatically reflected in the virtual image.

# LAB 4 
In this lab a webcam will be calibrated to determine the intrinsic parameters of the camera. The
calibrated camera as well as ArUco fiducial markers will then be used to determine the extrinsic
parameters of the camera and by inverse, the pose of the ArUco board. The virtual robot created in the
previous lab will then be transformed to the origin of the ArUco board coordinate system to create an
augmented reality interface.

https://user-images.githubusercontent.com/77029857/195733171-87891164-d6e4-4613-ae58-fe3d36e28225.mov

