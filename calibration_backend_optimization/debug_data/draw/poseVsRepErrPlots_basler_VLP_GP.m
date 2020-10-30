csv = load('PoseVsRepErr_basler_VLP_GP.csv');
avg_repErr = csv(:, 5);
 
mean_err = mean(avg_repErr) 
median_err = median(avg_repErr)
mode_err = mode(avg_repErr)
