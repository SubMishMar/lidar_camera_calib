orig = [0, 0, 0];
i = [orig; 0.1 0 0];
j = [orig; 0 0.1 0];
k = [orig; 0 0 0.1];

side_dim = 0.25;
plane_pts = [-side_dim,  side_dim, 0, 1;
              side_dim,  side_dim, 0, 1;
              side_dim, -side_dim, 0, 1;
             -side_dim, -side_dim, 0, 1;
             -side_dim,  side_dim, 0, 1;];
plane_pts = plane_pts';

roll = 5 * pi/180;
pitch = 10 * pi/180;
yaw = 7 * pi/180;

R_x = [1          0,          0; 
       0, cos(roll), -sin(roll);
       0, sin(roll),  cos(roll)];
      
R_y = [cos(pitch), 0, sin(pitch);
                0, 1, 0;
      -sin(pitch), 0, cos(pitch)];

R_z = [cos(yaw), -sin(yaw), 0;
       sin(yaw),  cos(yaw), 0;
              0,         0, 1];
              
R = [0 0 -1; -1 0 0; 0 1 0]*R_x*R_y*R_z;
T =[1.5, 0, 0]';
R_T = eye(4);
R_T(1:3,1:3) = R;
R_T(1:3, 4) = T;

plane_pts_transformed = R_T*plane_pts;
X = plane_pts_transformed(1,:);
Y = plane_pts_transformed(2,:);
Z = plane_pts_transformed(3,:);

obj_pt1 = plane_pts_transformed(1:3, 1);
obj_pt2 = plane_pts_transformed(1:3, 2);
obj_pt3 = plane_pts_transformed(1:3, 3);
obj_pt4 = plane_pts_transformed(1:3, 4);

figure(1)
plot3(X, Y, Z);
hold on;
plot3(i(:,1), i(:,2), i(:,3), 'Linewidth', 3);
hold on;
plot3(j(:,1), j(:,2), j(:,3), 'Linewidth', 3);
hold on;
plot3(k(:,1), k(:,2), k(:,3), 'Linewidth', 3);
hold off;
grid;
axis square

fx = 6.4372590342756985e+02;
fy = 6.4372590342756985e+02;
cx = 3.9534097290039062e+02;
cy = 3.0199901199340820e+02;
K = [fx,  0, cx;
      0, fy, cy;
      0,  0,  1];
C_T_L = [ 0.0256051, -0.999486, 0.0192813, 0.175155;
         -0.0361813, -0.0202016,  -0.999141,  -0.170539;
          0.999017,  0.0248855,   -0.03668 -0.0378068];
        
uv1 = K*C_T_L*plane_pts_transformed;
uv = uv1./uv1(3,:);
uv = uv';

figure(2)
plot(uv(:,1), uv(:,2));
grid;
axis square;

img_pt1 = uv(1,:)';
img_pt2 = uv(2,:)';
img_pt3 = uv(3,:)';
img_pt4 = uv(4,:)';

line_1 = cross(img_pt1, img_pt2);
line_2 = cross(img_pt2, img_pt3);
line_3 = cross(img_pt3, img_pt4);
line_4 = cross(img_pt4, img_pt1);

norm_1 = K'*line_1/norm(K'*line_1);
norm_2 = K'*line_2/norm(K'*line_2);
norm_3 = K'*line_3/norm(K'*line_3);
norm_4 = K'*line_4/norm(K'*line_4);

R = C_T_L(1:3, 1:3);
t = C_T_L(1:3, 4);

A1 = [norm_1(1)*(obj_pt1') norm_1(2)*(obj_pt1') norm_1(3)*(obj_pt1') norm_1'];
A2 = [norm_1(1)*(obj_pt2') norm_1(2)*(obj_pt2') norm_1(3)*(obj_pt2') norm_1'];

A3 = [norm_2(1)*(obj_pt2') norm_2(2)*(obj_pt2') norm_2(3)*(obj_pt2') norm_2'];
A4 = [norm_2(1)*(obj_pt3') norm_2(2)*(obj_pt3') norm_2(3)*(obj_pt3') norm_2'];

A5 = [norm_3(1)*(obj_pt3') norm_3(2)*(obj_pt3') norm_3(3)*(obj_pt3') norm_3'];
A6 = [norm_3(1)*(obj_pt4') norm_3(2)*(obj_pt4') norm_3(3)*(obj_pt4') norm_3'];

A7 = [norm_4(1)*(obj_pt4') norm_4(2)*(obj_pt4') norm_4(3)*(obj_pt4') norm_4'];
A8 = [norm_4(1)*(obj_pt1') norm_4(2)*(obj_pt1') norm_4(3)*(obj_pt1') norm_4'];

A = [A1; A2; A3; A4; A5; A6; A7; A8]

rank(A)