CL_T_L = textread('C_T_L_left.txt')';
CL_T_L = reshape(CL_T_L, 4, 3)';
CL_T_L = [CL_T_L; 0 0 0 1];

CR_T_L = textread('C_T_L_right.txt');
CR_T_L = reshape(CR_T_L, 4, 3)';
CR_T_L = [CR_T_L; 0 0 0 1];

CL_T_CR = CL_T_L * inv(CR_T_L)