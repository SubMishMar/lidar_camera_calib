function [R_z] = getRotZ(yaw_angle)
  yaw_angle = yaw_angle*pi/180;
  R_z = [cos(yaw_angle), -sin(yaw_angle), 0;
         sin(yaw_angle),  cos(yaw_angle), 0;
                      0,               0, 1];
end