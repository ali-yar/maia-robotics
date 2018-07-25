function q = getInitialJointConfiguration (x_d, quat_d)
%
% Get the Kuka LWR joint configuration given the position and orientation
%
% q = getInitialJointConfiguration(x)
%
% input:
%       x_d       dim 1x3     position vector
%       quat_d    dim 1x4     orientation vector (quaternions)
%
% output:
%       q       dim 4x4     joint vector

K = diag([50*[1 1 1], 20*[1 1 1]]);

% start with some random joint configuration
q = [7 25 -10 13 -15 11 20]'/180*pi;

for i=1:2000
  % obtain transformation matrix from base to e.e. by direct kinematics
  T = kuka_directkinematics(q);
  x = T(1:3,4); % current position
  quat = Rot2Quat(T(1:3,1:3)); % current orientation
  
  % get jacobian matrix (6x7)
  J = kuka_J(q);
  
  % Inverse kinematics algorithm
  error_pos = x_d - x;
  error_quat = QuatError(quat_d,quat);
  error = [error_pos; error_quat];
  
  % compute joint displacement
  dq = pinv(J) * K * error;

  q = q + 0.001 * dq;

end

end
