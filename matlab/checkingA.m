A = [0.1; -0.5; 0.7];
t = randn(3, 1);

scale1 = randn;
scale2 = randn;
scale3 = randn;
scale4 = randn;
scale5 = randn;

X1 = A + scale1*t;
X2 = A + scale2*t;
X3 = A + scale3*t;
X4 = A + scale4*t;
X5 = A + scale5*t;

X = [X1'; X2'; X3'; X4'; X5'];

plot3(X(:,1), X(:,2), X(:,3), '*');
hold on;
plot3(X(:,1), X(:,2), X(:,3));
grid;
axis square;

n = randn(3, 1);
nx = n(1); ny = n(2); nz = n(3);

A = [nx*(X1') ny*(X1') nz*(X1') nx ny nz;
     nx*(X2') ny*(X2') nz*(X2') nx ny nz;
     nx*(X3') ny*(X3') nz*(X3') nx ny nz;
     nx*(X4') ny*(X4') nz*(X4') nx ny nz;
     nx*(X5') ny*(X5') nz*(X5') nx ny nz]
rank(A)