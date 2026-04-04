#ifndef ROVER_4WD_EKF_H
#define ROVER_4WD_EKF_H

void Rover4wdEkf_Initialize(void);
float Rover4wdEfk_GetX(void);
float Rover4wdEfk_GetY(void);
float Rover4wdEfk_GetHeading(void);
void Rover4wdEkf_Predict(const float delta_t_imu, const float a_x_measured, const float a_y_measured, const float omega_z_measured);
void Rover4wdEkf_Update(const float delta_t_encoders, const float omega_z_measured, const float delta_s_l, const float delta_s_r);

#endif /* ROVER_4WD_EKF_H */