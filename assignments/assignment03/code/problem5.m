% Standard state-space matrices
A = [
 1,  4; 
-5, -10];
B = [0; 1];
C = [1, -4];
D = [0];

poles = [-1, -1];

% Need to use acker() because the poles are at the same location
% We'll use the fact that the controllability of the dual of the system
% is the same as the observability of the original system
L = acker(A' ,C',poles)';

dt = 0.001;
total_time = 10;
t = 0:dt:total_time;
num_its = length(t);

U = 1e-3; % input as a step function

X = [1; 1];       % initial states
X_hat = [0; 0];   % initial estimate of states
e0 = X - X_hat;   % initial error

% Setup vectors
X_vec = X;
X_hat_vec = X_hat;
e_vec = e0;

for i = 1:num_its-1;
  Y = C*X + D*U;
  Y_hat = C*X_hat + D*U;
  
  dX = A*X + B*U;
  X = X + dX*dt;
  
  dX_hat = A*X_hat + B*U + L*(Y - Y_hat);
  X_hat = X_hat + dX_hat*dt;
  
  e = X - X_hat;
  
  X_vec = [X_vec, X];
  X_hat_vec = [X_hat_vec, X_hat];
  e_vec = [e_vec, e];
end

figure;
plot(t, e_vec(1,:), 'LineWidth', 2);
hold on;
plot(t, e_vec(2,:), 'LineWidth', 2);
title('Error plot', 'fontsize', 20)
xlabel('time (s)', 'fontsize', 16)
ylabel('e(t)', 'fontsize', 16)
leg = legend('e_{1}(t)', 'e_{2}(t)')
set (leg, "fontsize", 16);

% References:
% [1]: http://www.eecs.tufts.edu/~khan/Courses/Spring2013/EE194/Lecs/Lec6and7.pdf
% [2]: http://ctms.engin.umich.edu/CTMS/index.php?example=Introduction&section=ControlStateSpace#23
% [3]: http://cse.lab.imtlucca.it/~bemporad/teaching/ac/pdf/06b-estimator.pdf