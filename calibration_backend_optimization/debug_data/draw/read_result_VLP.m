CL_T_L = textread('C_T_L_left_VLP.txt')';
CL_T_L = reshape(CL_T_L, 4, 3)';
CL_T_L = [CL_T_L; 0 0 0 1];

CR_T_L = textread('C_T_L_right_VLP.txt');
CR_T_L = reshape(CR_T_L, 4, 3)';
CR_T_L = [CR_T_L; 0 0 0 1];

CL_T_CR = CL_T_L * inv(CR_T_L);

CL_T_CR_true = [9.9999182809792209e-01, 3.8870119487459331e-03, 1.1112495156658535e-03, 0; 
               -3.8852110384028980e-03, 9.9999114322882299e-01, -1.6182086079222404e-03, 0;
               -1.1175296697777312e-03, 1.6138779451952689e-03, 9.9999807326085144e-01, 0; 
               0 0 0 1];
CL_T_CR_true(:, 4) = [-2.5049370201088572e-01; -2.6837913205551514e-04; -2.1145709898939401e-04; 1];

CL_T_CR_true = inv(CL_T_CR_true);
T_err = inv(CL_T_CR)*CL_T_CR_true

r11 = T_err(1, 1);
r12 = T_err(1, 2);
r13 = T_err(1, 3);
r21 = T_err(2, 1);
r22 = T_err(2, 2);
r23 = T_err(2, 3);
r31 = T_err(3, 1);
r32 = T_err(3, 2);
r33 = T_err(3, 3);

alpha = 180*atan2(r21, r11)/pi
beta = 180*atan2(r31, sqrt(r32*r32+r33*r33))/pi
gamma = 180*atan2(r32, r33)/pi