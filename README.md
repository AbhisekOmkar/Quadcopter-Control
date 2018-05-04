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
