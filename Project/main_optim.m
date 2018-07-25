clc; clear; close all;

K = diag([10*[1 1 1], 10*[1 1 1]]);

% initialize
tf = 2; % time duration
Ts = 1e-3; % sampling time
n = 7; % number of joints
t = 0:Ts:tf; 
N = length(t);
x_i = [0.32 -0.55 0.31]'; % initial position
orient_i = [0 0 0 1]'; % inital orientation 
xd  = zeros(3,N);
q   = zeros(n,N);
% q(:,1) = getInitialJointConfiguration(xi, orient_i);
q(:,1) = [1.56489  1.38241  -2.07832  1.24803  -1.20479  1.97508  0.45348]';
dq  = zeros(n,N);
quat = zeros(4,N);
error_pos = zeros(3,N);
error_quat = zeros(3,N);
error = zeros(6,N);

for i=1:N
    % desired task trajectory
    xd(:,i) = [-0.5 -0.1 0.3]';
    quat_d(:,i) = [0 0 0 1]';
    
    % direct kinematics
    T = kuka_directkinematics(q(:,i));
    x(:,i) = T(1:3,4);
    quat(:,i) = Rot2Quat(T(1:3,1:3));
    
    % Jacobian (6x7)
    J = kuka_J(q(:,i));
    
    % Inverse kinematics algorithm
    error_pos(:,i) = xd(:,i) - x(:,i);
    error_quat(:,i) = QuatError(quat_d(:,i),quat(:,i));
    error(:,i) = [error_pos(:,i);error_quat(:,i)];

    dq(:,i) = pinv(J)*K*error(:,i);

    if i<N
        q(:,i+1) = q(:,i) + Ts*dq(:,i);
    end
end


figure(1)
plot(t,q)
xlabel('time (s)')
ylabel('joint position (rad)')
legend('q1','q2','q3','q4','q5','q6','q7')

figure(2)
plot(t,dq)
xlabel('time (s)')
ylabel('joint velocity')
legend('q1','q2','q3','q4','q5','q6','q7')

figure(3)
plot(t,error(1:3,:))
xlabel('time (s)')
ylabel('position error')
legend('x','y','z')

figure(4)
plot(t,error(4:6,:))
xlabel('time (s)')
ylabel('orientation error')
legend('a','b','c')