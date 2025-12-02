# iPhone Teleop
#### Teleoporate a humanoid arm with just your iPhone

This project enables real-time teleoperation of a humanoid robotic arm using an iPhone. By leveraging Apple's ARKit for precise pose estimation, we bypass the need for expensive VR hardware, making intuitive robot control accessible to anyone with a smartphone.

<video width="320" height="240" controls>
  <source src="Demo/demo.gif" type="video/mp4">
</video>


## How It Works
1. The iOS app uses ARKit to track the device's position (x,y,z) and orientation (quaternion) in real-time
2. The 6D pose data is streamed via UDP sockets to the Python host machine running the visualizer
3. The 6D pose is transformed from into the G1 humanoid's coordinate frame
4. Pinocchio and CasADi solve the inverse kinematics problem, converting the pose target into joint angles
5. The resulting joint configurations are visualized in real-time using Meshcat.


## Acknowledgments
This project would not have been possible without the support of my mentor Soofiyan Atar and ARCLab for providing the resources and environment to explore this tecnhology.