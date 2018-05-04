# FCND Controls Project


## C++ Controller

### 1. Body Rate control
I obtain the desired moment along the axis in the body frame using a P controller.
First I calculated the error in the body rate. Then I turned that into the desired acceleration buy multiplying with kpPQR. It is also multiplied by the MOI to turn it into a desired moment.

	  V3F I;
	  I.x = Ixx;
	  I.y = Iyy;
	  I.z = Izz;
	  V3F body_rate_err = pqrCmd - pqr;
	  momentCmd = body_rate_err * kpPQR * I;

### 2. Roll-pitch controller
The roll pitch controller is responsible for generating roll and pitch commands to match the desired lateral acceleration. 

To do this I have to get the acceleration command. This is c. After making sure they are constrained to be between the max tilt angles, I calculate the b_x and b_y terms. These are then converted to angular velocities in the body frame. Since the quadcopter shouldn't ever thrust downwards, if the desired thrust is downwards, we skip this step and don't accelerate at all.

	if (collThrustCmd > 0.f){
	  float c = collThrustCmd / mass;

	  float b_x_c_target = accelCmd.x / c;
	  b_x_c_target = -CONSTRAIN(b_x_c_target, -maxTiltAngle, maxTiltAngle);
	  float b_x_term = kpBank * (b_x_c_target - R(0, 2));

	  float b_y_c_target = accelCmd.y / c;
	  b_y_c_target = -CONSTRAIN(b_y_c_target, -maxTiltAngle, maxTiltAngle);
	  float b_y_term = kpBank * (b_y_c_target - R(1, 2));

	  pqrCmd.x = (R(1, 0) * b_x_term - R(0, 0) * b_y_term) / R(2, 2);
	  pqrCmd.y = (R(1, 1) * b_x_term - R(0, 1) * b_y_term) / R(2, 2);
	  }
	else {
	  pqrCmd.x = 0.0; pqrCmd.y = 0.0;
	}
  
### Altitude Control
To control the altitude, I used a full PID controller. I first obtained the errors for the altitude, vertical acceleration, and the running altitude error. Then I multiplied these by their accociated kp values to get the PID terms. These were added to the vertical acceleration command to get and updated command. After making sure to account for gravity and the current rotation of the copter, I constrained it to the max acceleration rates, and multiplied it by the mass to obtain our thrust. This value was made negative because our z-axis points downward. 

	  float z_err = posZCmd - posZ;
	  integratedAltitudeError += z_err * dt;
	  float vel_err = velZCmd - velZ;

	  float p = kpPosZ * z_err;
	  float i = KiPosZ * integratedAltitudeError;
	  float d = kpVelZ * vel_err;

	  float vert_acc_cmd = p + i + d + accelZCmd;

	  float b_z = R(2, 2);

	  float acc = (vert_acc_cmd - CONST_GRAVITY) / b_z;
	  thrust = - mass * CONSTRAIN(acc, - maxAscentRate/dt, maxAscentRate/dt);


## Lateral position control
Here I take the desired lateral position, velocity and acceleration, and calculate the desire horizontal acceleration. First i check if the magnitude of the velocity command is grater than the maximum velocity. If it is I multiply the maximum speed by a normalized vector of the velocity commands. This gives more reasonable acceleration commands that are between 0 and the maximum speed. Then I implement a PD controller using the desired and actual position and velocity. This is then normalized again to make sure it hasn't exeeded the desired maximum acceleration.

	  if (velCmd.mag() > maxSpeedXY) {
	      velCmd = velCmd.norm() * maxSpeedXY;
	  }

	  float pos_err_x =  posCmd.x - pos.x;
	  float pos_err_y = posCmd.y - pos.y;

	  float vel_err_x = velCmd.x - vel.x;
	  float vel_err_y = velCmd.y - vel.y;

	  float p_x = pos_err_x * kpPosXY;
	  float p_y = pos_err_y * kpPosXY;

	  float d_x = vel_err_x * kpVelXY;
	  float d_y = vel_err_y * kpVelXY;

	  accelCmd.x = p_x + d_x + accelCmdFF.x;
	  accelCmd.y = p_y + d_y + accelCmdFF.y;

	  if (accelCmd.mag() > maxAccelXY) {
	      accelCmd = accelCmd.norm() * maxAccelXY;
	  }

	  accelCmd.z = 0;

	
### Yaw Control
A basic P controller is used to control the yaw. But additional calculations were used to make sure that the controller always turned towards the desired direction in the way that required the least motion. 

	  float yaw_cmd_pi;
	  if (yawCmd > 0) {
		  yaw_cmd_pi = fmodf(yawCmd, 2 * F_PI);
	  }
	  else {
		  yaw_cmd_pi = -fmodf(-yawCmd, 2 * F_PI);
	  }

	  float yaw_err = yaw_cmd_pi - yaw;
	  if (yaw_err > F_PI) {
		  yaw_err -= (2.0 * F_PI);
	  }else if (yaw_err < -F_PI) {
		  yaw_err += 2 * F_PI;
	  }

	  yawRateCmd = kpYaw * yaw_err;
