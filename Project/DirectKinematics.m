function  T0 = DirectKinematics(DH)

% Matrici di trasformazione omogenee rispetto alla terna base
% T = Homogeneous_dh(DH_i)
%
% input:
%       DH     dim nx4     tabella Denavit-Hartenberg
%
% output:
%       T      dim 4x4xn   matrici di trasformazioni omogenee rispetto alla terna base   


n = size(DH,1);
T0 = zeros(4,4,n);

% calcolo matrice di trasformazione omogenea tra terne consecutive
for i=1:n
   
    T(:,:,i) = Homogeneous(DH(i,:));
    
end

%calcolo matrici di trasformazione omogenee rispetto a terna base. T0(:,:,n) contiene la matrice di trasformazione omogenea tra l'end-effector e la terna base

T0(:,:,1) = T(:,:,1);

for i=2:n

    T0(:,:,i) = T0(:,:,i-1) * T(:,:,i);
    
end


end
