function dq_a = get_dqa(q, k)
%
% <description>
% 
% qa = get_qa(q, k)
%
% input:
%       q     dim nx1     joint configuration
%       k     dim 1x1     <description>
%
%   output:
%       qa    dim nx1     <description>
  
  function w = objective_func(q)
    J = kuka_J(q);
    w = sqrt(det(J*J'));
  end

n = size(q,1);

w_q = objective_func(q);

dw_q = zeros(1,n);

t = 0.001;

for i = 1:n
  q_t = q;
  q_t(i) = q_t(i) + t;
  w_q_t = objective_func(q_t);
  dw_q(i) = (w_q_t - w_q) / t;
end

dq_a = k * dw_q';

end
