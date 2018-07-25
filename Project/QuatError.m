function eo = QuatError(ed,e)
%
% error quaternion
%
% eo = QuatError(ed,e)
%
% input:
%       ed     dim 4x1     desired quaternion
%       e      dim 4x1     actual quaternion
%
% output:
%       eo      dim 4x1      error quaternion 
%
% G. Antonelli, Sistemi Robotici, fall 2012

eo = e(4)*ed(1:3) - ed(4)*e(1:3) - cross(ed(1:3),e(1:3));