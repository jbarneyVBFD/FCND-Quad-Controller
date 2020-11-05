# Write Up #

### Implement body rate control in C++ ###
- hello

### Implement roll pitch control in C++ ###
-

### Implement altitude contoller in C++ ###
-

### Implement lateral position control in C++ ###
-

### Implement yaw control in C++ ###
-

### Implement calculating the motor commands given commanded thrust and moments in C++ ###
- I first calculated the distance between a propeller and the x axis by dividing the length of the arm by the sqaure root of 2 and placing the result in the variable l.

```cpp
float l = L / sqrtf(2.f); 
```
- I then calculated the moments about the x, y, and z axes by dividing the roll and pitch moments by the d variable above, and dividing the yaw moment by the drag/thrust
ratio kappa. Then they were placed in Fx, Fy, and Fz variables.

```cpp
float Fx = momentCmd.x / l;
float Fy = momentCmd.y / l;
float Fz = momentCmd.z / kappa;
```

-  With the x, y and z axes rotating positive in the clockwise direction, and the motors on the arm with the negative slope spinning in the clockwise direction and thus 
producing negative moments about the z axis. The desired thrust for each of the motors was calculated by summing the collective thrust and the moments generated
together, then dividing by the four motors. The results were placed in the class member variable for graphing cmd.desuredThrustsN.

```cpp
cmd.desiredThrustsN[0] = (collThrustCmd + Mx + My - Mz) / 4.f; // front left
cmd.desiredThrustsN[1] = (collThrustCmd - Mx + My + Mz) / 4.f; // front right
cmd.desiredThrustsN[2] = (collThrustCmd + Mx - My + Mz) / 4.f; // rear left
cmd.desiredThrustsN[3] = (collThrustCmd - Mx - My - Mz) / 4.f; // rear right
```

