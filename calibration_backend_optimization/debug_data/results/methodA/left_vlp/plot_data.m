init_csv = csvread('random_init.txt');
initializations = init_csv(1:2:end,:);
results = init_csv(2:2:end,:);
rx_init = initializations(:,1);
ry_init = initializations(:,2);
rz_init = initializations(:,3);
x_init = initializations(:,4);
y_init = initializations(:,5);
z_init = initializations(:,6);
rx_res = results(:,1);
ry_res = results(:,2);
rz_res = results(:,3);
x_res = results(:,4);
y_res = results(:,5);
z_res = results(:,6);

subplot(231)
plot(rx_init, '.');
hold on;
plot(rx_res);
hold off;
xlabel('Trials')
ylabel('Roll[deg]')
legend('Initialization', 'Result')
grid;

subplot(232)
plot(ry_init, '.');
hold on;
plot(ry_res);
hold off;
xlabel('Trials')
ylabel('Pitch[deg]')
legend('Initialization', 'Result')
grid;

subplot(233)
plot(rz_init, '.');
hold on;
plot(rz_res);
hold off;
xlabel('Trials')
ylabel('Yaw[deg]')
legend('Initialization', 'Result')
grid;

subplot(234)
plot(x_init, '.');
hold on;
plot(x_res);
hold off;
xlabel('Trials')
ylabel('X[m]')
legend('Initialization', 'Result')
grid;


subplot(235)
plot(y_init, '.');
hold on;
plot(y_res);
hold off;
xlabel('Trials')
ylabel('Y[m]')
legend('Initialization', 'Result')
grid;

subplot(236)
plot(z_init, '.');
hold on;
plot(z_res);
hold off;
xlabel('Trials')
ylabel('Z[m]')
legend('Initialization', 'Result')
grid;

