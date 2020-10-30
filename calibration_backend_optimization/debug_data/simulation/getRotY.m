function [R_y] = getRotY(pitch_angle)
  pitch_angle = pitch_angle*pi/180;
  R_y = [cos(pitch_angle), 0, sin(pitch_angle);
                        0, 1,                0;
         -sin(pitch_angle), 0, cos(pitch_angle)];
end