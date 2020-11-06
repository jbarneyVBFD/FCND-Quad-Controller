# Write Up #

### Implement body rate control in C++ ###
- I first created vectors of floats with the V3F class for the moments of inertia, and the pqr error. 

```cpp
V3F inertia(Ixx, Iyy, Izz);
V3F pqr_error = pqrCmd - pqr;
```
Then I multiplied the moments of inertia by the pqr error and the gain parameters in the kpPQR variable. I placed the result in the already created and returned
V3F momentCmd variable.

```cpp
momentCmd = pqr_error * kpPQR * inertia;
```

### Implement roll pitch control in C++ ###
- I first converted the collective thrust variable in Newtons to acceleration and placed it in the p control time constant variable named Trp. I solved for 
<a href="https://www.codecogs.com/eqnedit.php?latex=\dot{b}_{c}^{x}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\dot{b}_{c}^{x}" title="\dot{b}_{c}^{x}" /></a> 
by taking the difference of the actual and the given desired acceleration in the X coordinate then divided by Trp. <br />
  <a href="https://www.codecogs.com/eqnedit.php?latex=\dot{b}_{c}^{x}&space;=&space;\left&space;(b^{x}&space;-&space;b_{c}^{x}&space;\right&space;)&space;/&space;\tau&space;_{rp}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\dot{b}_{c}^{x}&space;=&space;\left&space;(b^{x}&space;-&space;b_{c}^{x}&space;\right&space;)&space;/&space;\tau&space;_{rp}" title="\dot{b}_{c}^{x} = \left (b^{x} - b_{c}^{x} \right ) / \tau _{rp}" /></a> <br />
I placed the result in the variable bXcT and constrained it to be within the min and max tilt angles. I repeated the process for the acceleration in the Y cooridinate with the equation <br /> <a href="https://www.codecogs.com/eqnedit.php?latex=\dot{b}_{c}^{y}&space;=&space;\left&space;(b^{y}&space;-&space;b_{c}^{y}&space;\right&space;)&space;/&space;\tau&space;_{rp}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\dot{b}_{c}^{y}&space;=&space;\left&space;(b^{y}&space;-&space;b_{c}^{y}&space;\right&space;)&space;/&space;\tau&space;_{rp}" title="\dot{b}_{c}^{y} = \left (b^{y} - b_{c}^{y} \right ) / \tau _{rp}" /></a> <br />
and placed the result in the bYcT variable.

```cpp
float Trp = collThrustCmd / mass;
float bXcT = CONSTRAIN(((R(0, 2) -accelCmd.x )/ Trp), -maxTiltAngle, maxTiltAngle);
float bYcT = CONSTRAIN(((R(0,2) -accelCmd.y )/ Trp), -maxTiltAngle, maxTiltAngle);
```

To obtain the desired pitch and roll rates I solved the following equation. <br />  
<a href="https://www.codecogs.com/eqnedit.php?latex=\begin{bmatrix}&space;p_{c}\\&space;q_{c}&space;\end{bmatrix}&space;=&space;\frac{\mathrm{1}&space;}{\mathrm{R}&space;_{33}}&space;\begin{bmatrix}&space;R_{21}&space;&&space;-R_{11}&space;\\&space;R_{22}&space;&&space;-R_{12}&space;\end{bmatrix}&space;\begin{bmatrix}&space;\dot{b}_{c}^{x}\\&space;\dot{b}_{c}^{y}&space;\end{bmatrix}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\begin{bmatrix}&space;p_{c}\\&space;q_{c}&space;\end{bmatrix}&space;=&space;\frac{\mathrm{1}&space;}{\mathrm{R}&space;_{33}}&space;\begin{bmatrix}&space;R_{21}&space;&&space;-R_{11}&space;\\&space;R_{22}&space;&&space;-R_{12}&space;\end{bmatrix}&space;\begin{bmatrix}&space;\dot{b}_{c}^{x}\\&space;\dot{b}_{c}^{y}&space;\end{bmatrix}" title="\begin{bmatrix} p_{c}\\ q_{c} \end{bmatrix} = \frac{\mathrm{1} }{\mathrm{R} _{33}} \begin{bmatrix} R_{21} & -R_{11} \\ R_{22} & -R_{12} \end{bmatrix} \begin{bmatrix} \dot{b}_{c}^{x}\\ \dot{b}_{c}^{y} \end{bmatrix}" /></a> <br />
I placed the results in the variables pc and qc then into the already created and returned variable pqrCmd. 
```cpp
float pc = (1 / R(2, 2)) * (-R(1, 0) * kpBank * (R(0, 2) - bXcT) + R(0, 0) * kpBank * (R(1, 2) - bYcT));
float qc = (1 / R(2, 2)) * (-R(1, 1) * kpBank * (R(0, 2) - bXcT) + R(0, 1) * kpBank * (R(1, 2) - bYcT));

pqrCmd.x = pc;
pqrCmd.y = qc;
```
I then wrapped it all in an if statement to ensure the collective thrust input is not negative. 

### Implement altitude contoller in C++ ###
- I first calculated the z error by finding the difference of the inputs of the desired vertical position and the actual vertical position. I placed this in the zE variable.
Then I incremented the integrated altitude error by the product of the z error and the time step. 
```cpp
float zE = posZCmd - posZ;
integratedAltitudeError += zE * dt;
```
- I then calculated the p-term by multiplying the positional gain parameter by the z error. Simarily, I calculated the i-term by multiplying the integrated positional gain parameter by the integrated z error. 
```cpp
float pTerm = kpPosZ * zE;
float iTerm = KiPosZ * integratedAltitudeError;
```
- Then calculated the z dot error with the difference of the commanded velocity and the actual velocity, constrained to be within the maximum ascent and descent rates. This led me to solve for the d-term by multiplying the velocity gain parameter by the z dot error.
```cpp
float zDotE = CONSTRAIN(velZCmd, -maxAscentRate, maxDescentRate) - velZ;
float dTerm = kpVelZ * zDotE;
```
- I then solved for <a href="https://www.codecogs.com/eqnedit.php?latex=\bar{u}_{1}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\bar{u}_{1}" title="\bar{u}_{1}" /></a> by summing the p, i and d-terms as well as the feed forward vertical acceleration. I then solved for u1 by dividing the difference of <a href="https://www.codecogs.com/eqnedit.php?latex=\bar{u}_{1}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\bar{u}_{1}" title="\bar{u}_{1}" /></a> and gravity by the adjustable knob <a href="https://www.codecogs.com/eqnedit.php?latex=b^{z}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?b^{z}" title="b^{z}" /></a>
. From there I converted the u1 to Newtons and accounted for z axis down is positive by multiplying it by the negative mass. I also constrained u1 by the max ascent and descent rates divided by the time step.
```cpp
float u1Bar = pTerm + iTerm + dTerm + accelZCmd;
float u1 = (u1Bar - CONST_GRAVITY) / R(2, 2);
thrust = - mass * CONSTRAIN(u1, - maxAscentRate / dt, maxDescentRate / dt);
```

### Implement lateral position control in C++ ###
- I first calculated the magnitude of the commanded velocity, then checked if it was greater than the max speed in the x and y direction. If found to be greater, I normalized the commanded velocity while multiplying by the max speed. I did the same thing for the commanded acceleration. 
```cpp
float velocityMag = sqrtf(powf(velCmd.x, 2) + powf(velCmd.y, 2));

if (velocityMag > maxSpeedXY){
    velCmd *= maxSpeedXY / velocityMag;
}

float accelMag = sqrtf(powf(accelCmd.x, 2) + powf(accelCmd.y, 2));

if (accelMag > maxAccelXY){
    accelCmd *= maxAccelXY / accelMag;
}
```
- I then summed the p-term, d-term and feed forward acceleration command and set <a href="https://www.codecogs.com/eqnedit.php?latex=\ddot{z}" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\ddot{z}" title="\ddot{z}" /></a> to equal 0.
```cpp
accelCmd = accelCmdFF + kpPosXY * (posCmd - pos) + kpVelXY * (velCmd - vel);
accelCmd.z = 0.f;
```

### Implement yaw control in C++ ###
- To keep the commanded yaw within 0 to 2pi (or 360 degrees) I first change the commanded yaw to be the remainder of it by 2 times pi. I then calculated the yaw error. If not within the range pi to negative pi, I incremented the error by 2 times pi back into the range. 
```cpp
yawCmd = fmodf(yawCmd, 2.0 * F_PI);

float yawE = yawCmd - yaw;

if (yawE > F_PI){
    yawE -= 2.0 * F_PI;
}
else if (yawE < -F_PI){
    yawE += 2.0 * F_PI;
}
```
- I then multiplied the yaw error by the yaw gain parameter and returned the result.
```cpp
yawRateCmd = kpYaw * yawE;
```

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
together, then dividing by the four motors. The results were placed in the class member variable for graphing, cmd.desuredThrustsN.

```cpp
cmd.desiredThrustsN[0] = (collThrustCmd + Mx + My - Mz) / 4.f; // front left
cmd.desiredThrustsN[1] = (collThrustCmd - Mx + My + Mz) / 4.f; // front right
cmd.desiredThrustsN[2] = (collThrustCmd + Mx - My + Mz) / 4.f; // rear left
cmd.desiredThrustsN[3] = (collThrustCmd - Mx - My - Mz) / 4.f; // rear right
```

