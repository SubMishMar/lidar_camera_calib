%% nerian_left
CL_T_os = textread('C_T_L_left_os_GP.txt')';
CL_T_os = reshape(CL_T_os, 4, 3)';
CL_T_os = [CL_T_os; 0 0 0 1];

CL_T_vlp = textread('C_T_L_left_VLP_GP.txt')';
CL_T_vlp = reshape(CL_T_vlp, 4, 3)';
CL_T_vlp = [CL_T_vlp; 0 0 0 1];

vlp_T_os_nL_GP = inv(CL_T_vlp)*CL_T_os
fileID = fopen('vlp_T_os_nL_GP.txt','w');
fprintf(fileID,'%5d %5d %5d %5d\n', vlp_T_os_nL_GP');
fclose(fileID);

%% nerian_right
CR_T_os = textread('C_T_L_right_os_GP.txt')';
CR_T_os = reshape(CR_T_os, 4, 3)';
CR_T_os = [CR_T_os; 0 0 0 1];

CR_T_vlp = textread('C_T_L_right_VLP_GP.txt')';
CR_T_vlp = reshape(CR_T_vlp, 4, 3)';
CR_T_vlp = [CR_T_vlp; 0 0 0 1];

vlp_T_os_nR_GP = inv(CR_T_vlp)*CR_T_os
fileID = fopen('vlp_T_os_nR_GP.txt','w');
fprintf(fileID,'%5d %5d %5d %5d\n', vlp_T_os_nR_GP');
fclose(fileID);
%% basler
basler_T_os = textread('C_T_L_basler_os_GP.txt')';
basler_T_os = reshape(basler_T_os, 4, 3)';
basler_T_os = [basler_T_os; 0 0 0 1];

basler_T_vlp = textread('C_T_L_basler_VLP_GP.txt')';
basler_T_vlp = reshape(basler_T_vlp, 4, 3)';
basler_T_vlp = [basler_T_vlp; 0 0 0 1];

vlp_T_os_basler_GP = inv(basler_T_vlp)*basler_T_os
fileID = fopen('vlp_T_os_basler_GP.txt','w');
fprintf(fileID,'%5d %5d %5d %5d\n', vlp_T_os_basler_GP');
fclose(fileID);