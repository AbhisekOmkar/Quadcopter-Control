# FCND Controls Project


## C++ Controller

### 1. Roll-pitch controller
`
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
  `
  
