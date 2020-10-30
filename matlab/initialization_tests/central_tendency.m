initializations1 = csvread('test1_planeOnly15views/initializations.csv');
results1 = csvread('test1_planeOnly15views/results.csv');

initializations2 = csvread('test2_lineOnly25views/initializations.csv');
results2 = csvread('test2_lineOnly25views/results.csv');

initializations3 = csvread('test3_planeLineJoint15_25views/initializations.csv');
results3 = csvread('test3_planeLineJoint15_25views/results.csv');

initializations4 = csvread('test4_planethenLineJoint15_25views/initializations.csv');
results4 = csvread('test4_planethenLineJoint15_25views/results.csv');

reproj_errs1 = results1(:, 7);
mean_reprojerr1 = mean(reproj_errs1(find(isnan(reproj_errs1) ~= 1)))
mode_reprojerr1 = mode(reproj_errs1(find(isnan(reproj_errs1) ~= 1)))
median_reprojerr1 = median(reproj_errs1(find(isnan(reproj_errs1) ~= 1)))
range_reprojerr1 = range(reproj_errs1(find(isnan(reproj_errs1) ~= 1)))
printf('\n');
reproj_errs2 = results2(:, 7);
mean_reprojerr2 = mean(reproj_errs2(find(isnan(reproj_errs2) ~= 1)))
mode_reprojerr2 = mode(reproj_errs2(find(isnan(reproj_errs2) ~= 1)))
median_reprojerr2 = median(reproj_errs2(find(isnan(reproj_errs2) ~= 1)))
range_reprojerr2 = range(reproj_errs2(find(isnan(reproj_errs2) ~= 1)))
printf('\n');
reproj_errs3 = results3(:, 7);
mean_reprojerr3 = mean(reproj_errs3(find(isnan(reproj_errs3) ~= 1)))
mode_reprojerr3 = mode(reproj_errs3(find(isnan(reproj_errs3) ~= 1)))
median_reprojerr3 = median(reproj_errs3(find(isnan(reproj_errs3) ~= 1)))
range_reprojerr3 = range(reproj_errs3(find(isnan(reproj_errs3) ~= 1)))
printf('\n');
reproj_errs4 = results4(:, 7);
mean_reprojerr4 = mean(reproj_errs4(find(isnan(reproj_errs4) ~= 1)))
mode_reprojerr4 = mode(reproj_errs4(find(isnan(reproj_errs4) ~= 1)))
median_reprojerr4 = median(reproj_errs4(find(isnan(reproj_errs4) ~= 1)))
range_reprojerr4 = range(reproj_errs4(find(isnan(reproj_errs4) ~= 1)))
printf('\n');
stat_data = [mean_reprojerr1, mode_reprojerr1, median_reprojerr1, range_reprojerr1;
             mean_reprojerr2, mode_reprojerr2, median_reprojerr2, range_reprojerr2;
             mean_reprojerr3, mode_reprojerr3, median_reprojerr3, range_reprojerr3;
             mean_reprojerr4, mode_reprojerr4, median_reprojerr4, range_reprojerr4];
x = [1, 2, 3, 4];      
b = bar(x, stat_data, 'LineWidth', 3);
ylabel('Reprojection Error', 'FontSize', 30, 'FontWeight', 'bold', 'Color','k');
legend('mean', 'median', 'mode', 'range');
grid;
