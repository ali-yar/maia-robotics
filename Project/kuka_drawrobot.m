function kuka_drawrobot(q)
%
% Draw the Kuka LWR in 3D with intermediate frames
%
% function kuka_drawrobot(q)
%
% input:
%       q     dim 7x1     joint position according to the kuka convention
%
% WARNING: bad coding, to be rewritten calling kuka_directkinematics.m
%
% G. Antonelli, Sistemi Robotici, spring 2018

d = 0.03;

d0  = 0.3105;
d3  = 0.40; 
d5  = 0.39; 
d7e = 0.078; 

q1 = q(1);
q2 = q(2);
q3 = q(3);
q4 = q(4);
q5 = q(5);
q6 = q(6);
q7 = q(7);

s1 = sin(q1);
c1 = cos(q1);
s2 = sin(q2);
c2 = cos(q2);
s3 = sin(q3);
c3 = cos(q3);
s4 = sin(q4);
c4 = cos(q4);
s5 = sin(q5);
c5 = cos(q5);
s6 = sin(q6);
c6 = cos(q6);
s7 = sin(q7);
c7 = cos(q7);

T0 = zeros(4,4,9);
%%% the frame assignment is different with the assigned frame in kuka doccument
Tb0=[1 0 0 0;
     0 1 0 0;
     0 0 1 d0;
     0 0 0 1];       
A01=[c1 0  s1 0;
     s1 0 -c1 0;
     0  1  0  0;
     0  0  0  1];
A12=[c2 0 -s2 0;
     s2 0  c2 0;
     0 -1  0  0;
     0  0  0  1]; 
A23=[-c3 0 -s3 0;
     -s3 0  c3 0;
      0  1  0  d3;
      0  0  0  1];
A34=[-c4 0 -s4 0;
     -s4 0  c4 0;
      0  1  0  0;
      0  0  0  1]; 
A45=[c5 0  s5 0;
     s5 0 -c5 0;
     0  1  0  d5;
     0  0  0  1];
A56=[c6 0 -s6 0;
     s6 0  c6 0;
     0 -1  0  0;
     0  0  0  1];
A67=[c7 -s7 0 0;
     s7  c7 0 0;
     0   0  1 0;
     0   0  0 1];
T7e=[1 0 0 0;
     0 1 0 0 ;
     0 0 1 d7e;
     0 0 0 1];  
  
%T = Tb0*A01*A12*A23*A34*A45*A56*A67*T7e;
T0(:,:,1) = Tb0;
T0(:,:,2) = T0(:,:,1)*A01;
T0(:,:,3) = T0(:,:,2)*A12;
T0(:,:,4) = T0(:,:,3)*A23;
T0(:,:,5) = T0(:,:,4)*A34;
T0(:,:,6) = T0(:,:,5)*A45;
T0(:,:,7) = T0(:,:,6)*A56;
T0(:,:,8) = T0(:,:,7)*A67;
T0(:,:,9) = T0(:,:,8)*T7e;


hold on
view([2 -2 2])
T00 = [eye(3) [0 0 0]'; 0 0 0 1];
DrawFrame(T00,1);
for i=2:9
    DrawFrame(T0(:,:,i),0);
end

DrawLink(T00(1:3,4),T0(1:3,4,1),d)
for i=2:9
    DrawLink(T0(1:3,4,i-1),T0(1:3,4,i),d)
end
axis equal
grid
xlabel('x')
ylabel('y')
zlabel('z')

end

function DrawLink(pa,pb,d)
    %
    % DrawLink(pa,pb)
    %
    % input:
    %   pa   dim 3x1    "starting" point
    %   pb   dim 3x1    "ending" point
    %   d    dim 1x1    link diameter
    %
    % G. Antonelli, Simurv 4.0, 2013
    % http://www.eng.docente.unicas.it/gianluca_antonelli/simurv

    pa=CheckVector(pa);
    pb=CheckVector(pb);

    % generate points on the cylinder
    % aligned with x
    [z,y,x] = cylinder(d*ones(41,1),40);
    x = norm(pb-pa)*x;
    x2 = (pb-pa)/norm(pb-pa);
    if ((x2(1)==-1)||(x2(1)==1))
        z2 = [0 0 1]';
    else
        z2 = cross([1;0;0],x2); z2 = z2/norm(z2);
    end
    y2 = cross(x2,z2);

    % rotate and translate points
    R = [x2 y2 z2];
    for i=1:length(x)
       for j=1:length(x)
          % rotation
          rr=R*[x(i,j) y(i,j) z(i,j)]';
          x(i,j) = rr(1);
          y(i,j) = rr(2); 
          z(i,j) = rr(3);
          % translation
          x(i,j) = x(i,j) + pa(1);
          y(i,j) = y(i,j) + pa(2); 
          z(i,j) = z(i,j) + pa(3);
       end
    end


    h = surfl(x,y,z,[0 0 -5]);
    set(h,'facealpha',.5)
    set(h,'facecolor','interp');
    set(h,'edgecolor','none');
    colormap(bone)
end

function DrawFrame(T,mytext)
    %
    % DrawFrame(T,mytext)
    %
    % input:
    %   T   dim 4x4    homogeneous transf. matrix
    %   mytext          ==1 writes "x", "y", "z" close to the versor
    %
    % G. Antonelli, Sistemi Robotici, fall 2012

    scala = .1;     % scaling factor with respect to the versor

    d_ax = .05*scala; % axis diameter
    d_ar = .1*scala;  % axis diameter of the arrow head
    s_ar = .7;        % normalized position of the beginning of the head arrow
    d_txt = 1.1;      % text scaling factor

    T(1:3,1:3) = scala*T(1:3,1:3);

    % axis x - red
    arrow3d([T(1,4) T(1,4)+T(1,1)],[T(2,4) T(2,4)+T(2,1)],[T(3,4) T(3,4)+T(3,1)],s_ar,d_ax,d_ar,[1 0 0]);
    if mytext==1
        text(d_txt*(T(1,4)+T(1,1)),d_txt*T(2,4)+T(2,1),d_txt*T(3,4)+T(3,1),'x');
    end
    % axis y - green
    arrow3d([T(1,4) T(1,4)+T(1,2)],[T(2,4) T(2,4)+T(2,2)],[T(3,4) T(3,4)+T(3,2)],s_ar,d_ax,d_ar,[0 1 0]);
    if mytext==1
        text(d_txt*(T(1,4)+T(1,2)),d_txt*T(2,4)+T(2,2),d_txt*T(3,4)+T(3,2),'y');
    end
    % axis z - blue
    arrow3d([T(1,4) T(1,4)+T(1,3)],[T(2,4) T(2,4)+T(2,3)],[T(3,4) T(3,4)+T(3,3)],s_ar,d_ax,d_ar,[0 0 1]);
    if mytext==1
        text(d_txt*(T(1,4)+T(1,3)),d_txt*T(2,4)+T(2,3),d_txt*T(3,4)+T(3,3),'z');
    end

end

function [h]=arrow3d(x,y,z,head_frac,radii,radii2,colr)
    %
    % The function plotting 3-dimensional arrow
    %
    % h=arrow3d(x,y,z,head_frac,radii,radii2,colr)
    %
    % The inputs are:
    %       x,y,z =  vectors of the starting point and the ending point of the
    %           arrow, e.g.:  x=[x_start, x_end]; y=[y_start, y_end];z=[z_start,z_end];
    %       head_frac = fraction of the arrow length where the head should  start
    %       radii = radius of the arrow
    %       radii2 = radius of the arrow head (defult = radii*2)
    %       colr =   color of the arrow, can be string of the color name, or RGB vector  (default='blue')
    %
    % The output is the handle of the surfaceplot graphics object.
    % The settings of the plot can changed using: set(h, 'PropertyName', PropertyValue)
    %
    % example #1:
    %        arrow3d([0 0],[0 0],[0 6],.5,3,4,[1 0 .5]);
    % example #2:
    %        arrow3d([2 0],[5 0],[0 -6],.2,3,5,'r');
    % example #3:
    %        h = arrow3d([1 0],[0 1],[-2 3],.8,3);
    %        set(h,'facecolor',[1 0 0])
    % 
    % Written by Moshe Lindner , Bar-Ilan University, Israel.
    % July 2010 (C)

    if nargin==5
        radii2=radii*2;
        colr='blue';
    elseif nargin==6
        colr='blue';
    end
    if size(x,1)==2
        x=x';
        y=y';
        z=z';
    end

    x(3)=x(2);
    x(2)=x(1)+head_frac*(x(3)-x(1));
    y(3)=y(2);
    y(2)=y(1)+head_frac*(y(3)-y(1));
    z(3)=z(2);
    z(2)=z(1)+head_frac*(z(3)-z(1));
    r=[x(1:2)',y(1:2)',z(1:2)'];

    N=50;
    dr=diff(r);
    dr(end+1,:)=dr(end,:);
    origin_shift=(ones(size(r))*(1+max(abs(r(:))))+[dr(:,1) 2*dr(:,2) -dr(:,3)]);
    r=r+origin_shift;

    normdr=(sqrt((dr(:,1).^2)+(dr(:,2).^2)+(dr(:,3).^2)));
    normdr=[normdr,normdr,normdr];
    dr=dr./normdr;
    Pc=r;
    n1=cross(dr,Pc);
    normn1=(sqrt((n1(:,1).^2)+(n1(:,2).^2)+(n1(:,3).^2)));
    normn1=[normn1,normn1,normn1];
    n1=n1./normn1;
    P1=n1+Pc;

    X1=[];Y1=[];Z1=[];
    j=1;
    for theta=([0:N])*2*pi./(N);
        R1=Pc+radii*cos(theta).*(P1-Pc) + radii*sin(theta).*cross(dr,(P1-Pc)) -origin_shift;
        X1(2:3,j)=R1(:,1);
        Y1(2:3,j)=R1(:,2);
        Z1(2:3,j)=R1(:,3);
        j=j+1;
    end

    r=[x(2:3)',y(2:3)',z(2:3)'];

    dr=diff(r);
    dr(end+1,:)=dr(end,:);
    origin_shift=(ones(size(r))*(1+max(abs(r(:))))+[dr(:,1) 2*dr(:,2) -dr(:,3)]);
    r=r+origin_shift;

    normdr=(sqrt((dr(:,1).^2)+(dr(:,2).^2)+(dr(:,3).^2)));
    normdr=[normdr,normdr,normdr];
    dr=dr./normdr;
    Pc=r;
    n1=cross(dr,Pc);
    normn1=(sqrt((n1(:,1).^2)+(n1(:,2).^2)+(n1(:,3).^2)));
    normn1=[normn1,normn1,normn1];
    n1=n1./normn1;
    P1=n1+Pc;

    j=1;
    for theta=([0:N])*2*pi./(N);
        R1=Pc+radii2*cos(theta).*(P1-Pc) + radii2*sin(theta).*cross(dr,(P1-Pc)) -origin_shift;
        X1(4:5,j)=R1(:,1);
        Y1(4:5,j)=R1(:,2);
        Z1(4:5,j)=R1(:,3);
        j=j+1;
    end

    X1(1,:)=X1(1,:)*0 + x(1);
    Y1(1,:)=Y1(1,:)*0 + y(1);
    Z1(1,:)=Z1(1,:)*0 + z(1);
    X1(5,:)=X1(5,:)*0 + x(3);
    Y1(5,:)=Y1(5,:)*0 + y(3);
    Z1(5,:)=Z1(5,:)*0 + z(3);

    h=surf(X1,Y1,Z1,'facecolor',colr,'edgecolor','none');
    camlight
    lighting phong

end

function out=CheckVector(in)
    %
    % Vectors are columns
    %
    % G. Antonelli, Simurv 4.0, 2013
    % http://www.eng.docente.unicas.it/gianluca_antonelli/simurv

    if all(size(in)>1)
        error('Vector expected')
    elseif size(in,1)==1
        out = in';
    else
        out = in;
    end

end
