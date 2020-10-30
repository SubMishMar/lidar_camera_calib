function [R_x] = getRotX(roll_angle)
  roll_angle = roll_angle*pi/180;
  R_x = [1              0                 0;
         0 cos(roll_angle) -sin(roll_angle);
         0 sin(roll_angle)  cos(roll_angle)];
end
