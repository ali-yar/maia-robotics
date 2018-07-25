clc; clear; close all;

isOptimized = true; % maximize manipulability

K = diag([50*[1 1 1], 1*[1 1 1]]);

% initialize
tf = 2; % time duration
Ts = 1e-3; % sampling time
n = 7; % number of joints
t = 0:Ts:1.5*tf; 
N = length(t);
x_i = [0.32 -0.55 0.31]'; % initial position
x_f = [-0.5 -0.1 0.3]'; % final position

xd  = zeros(3,N);
q   = zeros(n,N);

% orient_i = [0 0 0 1]'; % give some inital orientation 
% q(:,1) = getInitialJointConfiguration(xi, orient_i);

q(:,1) = [1.56489  1.38241  -2.07832  1.24803  -1.20479  1.97508  0.45348]';
dq  = zeros(n,N);
quat = zeros(4,N);
error_pos = zeros(3,N);
error_quat = zeros(3,N);
error = zeros(6,N);

dx_c = [1.5 1.6 1.8]'.*abs(x_f-x_i)/tf; % cruise

k_a = 100;
dq_a = zeros(n,1);

for i=1:N

    % get the intermediate desired position
    [xd(:,i),dx(:,i),ddx(:,i)] = trapezoidal(x_i,x_f,dx_c,tf,t(i));
    
    % direct kinematics
    T = kuka_directkinematics(q(:,i));
    
    % current position
    x(:,i) = T(1:3,4);
    
    % current orientation (in quaternions)
    quat(:,i) = Rot2Quat(T(1:3,1:3));
    
    % desired orientation (same as inital orientation)
    quat_d(:,i) = quat(:,i);
    
    % Jacobian (6x7)
    J = kuka_J(q(:,i)); 
    
    % Inverse kinematics algorithm
    
    % position error
    error_pos(:,i) = xd(:,i) - x(:,i); 
    
    % orientation error
    error_quat(:,i) = QuatError(quat_d(:,i),quat(:,i));
    
    % current error 
    error(:,i) = [error_pos(:,i);error_quat(:,i)];
    
    % compute joint displacement needed for reducing the error
    if isOptimized
      dq_a = get_dqa(q(:,i), k_a);
    end
    particular_sol = pinv(J) * K * error(:,i);
    homogenuous_sol = (eye(n) - pinv(J) * J) * dq_a;
    dq(:,i) = particular_sol + homogenuous_sol;

    if i<N
        % update joint configuration
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

figure(5)
plot(t,xd,'.')
title('Position')
ylabel('pos')
legend('x','y','z')
