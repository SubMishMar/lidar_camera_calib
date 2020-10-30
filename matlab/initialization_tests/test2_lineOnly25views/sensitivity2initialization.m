initializations = csvread('initializations.csv');
results = csvread('results.csv');

rx_init = initializations(:, 1);
rx_res = results(:, 1);

ry_init = initializations(:, 2);
ry_res = results(:, 2);

rz_init = initializations(:, 3);
rz_res = results(:, 3);

tx_init = initializations(:, 4);
tx_res = results(:, 4);

ty_init = initializations(:, 5);
ty_res = results(:, 5);

tz_init = initializations(:, 6);
tz_res = results(:, 6);
reproj_errs = results(:, 7);

figure(1)
subplot(231)
plot(rx_init, '+', 'MarkerSize', 12, 'LineWidth', 3);
hold on;
plot(rx_res, '*', 'MarkerSize', 12, 'LineWidth', 3);
hold on;
plot(reproj_errs, 'd', 'MarkerSize', 12, 'LineWidth', 3);
hold off;
xlabel('experiment no', 'FontSize',12,'FontWeight','bold','Color','r');
ylabel('r_x', 'FontSize',20,'FontWeight','bold','Color','r');
grid;

subplot(232)
plot(ry_init, '+', 'MarkerSize',12, 'LineWidth', 3);
hold on;
plot(ry_res, '*', 'MarkerSize',12, 'LineWidth', 3);
hold on;
plot(reproj_errs, 'd', 'MarkerSize', 12, 'LineWidth', 3);
hold off;
xlabel('experiment no', 'FontSize',12,'FontWeight','bold','Color','r');
ylabel('r_y', 'FontSize',20,'FontWeight','bold','Color','r');
grid;

subplot(233)
plot(rz_init, '+', 'MarkerSize',12, 'LineWidth', 3);
hold on;
plot(rz_res, '*', 'MarkerSize',12, 'LineWidth', 3);
hold on;
plot(reproj_errs, 'd', 'MarkerSize', 12, 'LineWidth', 3);
hold off;
xlabel('experiment no', 'FontSize',12,'FontWeight','bold','Color','r');
ylabel('r_z', 'FontSize',20,'FontWeight','bold','Color','r');
grid;

subplot(234)
plot(tx_init, '+', 'MarkerSize',12, 'LineWidth', 3);
hold on;
plot(tx_res, '*', 'MarkerSize',12, 'LineWidth', 3);
hold on;
plot(reproj_errs, 'd', 'MarkerSize', 12, 'LineWidth', 3);
hold off;
xlabel('experiment no', 'FontSize',12,'FontWeight','bold','Color','r');
ylabel('t_x', 'FontSize',20,'FontWeight','bold','Color','r');
grid;


subplot(235)
plot(ty_init, '+', 'MarkerSize',12, 'LineWidth', 3);
hold on;
plot(ty_res, '*', 'MarkerSize',12, 'LineWidth', 3);
hold on;
plot(reproj_errs, 'd', 'MarkerSize', 12, 'LineWidth', 3);
hold off;
xlabel('experiment no', 'FontSize',12,'FontWeight','bold','Color','r');
ylabel('t_y', 'FontSize',20,'FontWeight','bold','Color','r');
grid;

subplot(236)
plot(tz_init, '+', 'MarkerSize',12, 'LineWidth', 3);
hold on;
plot(tz_res, '*', 'MarkerSize',12, 'LineWidth', 3);
hold on;
plot(reproj_errs, 'd', 'MarkerSize', 12, 'LineWidth', 3);
hold off;
xlabel('experiment no', 'FontSize',12,'FontWeight','bold','Color','r');
ylabel('t_z', 'FontSize',20,'FontWeight','bold','Color','r');
grid;

mean_reprojerr = mean(reproj_errs(find(isnan(reproj_errs) ~= 1)))
mode_reprojerr = mode(reproj_errs(find(isnan(reproj_errs) ~= 1)))
median_reprojerr = median(reproj_errs(find(isnan(reproj_errs) ~= 1)))
range_reprojerr = range(reproj_errs(find(isnan(reproj_errs) ~= 1)))