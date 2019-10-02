clear all; clc;
% Define Geometric parameters
l1 = 10;  l2=10; l3=10; l4 = 10; l5 =10; l6 = 10;

%D-H parameters for each joint

% Joints 1-2 O0-O1
d1=l1;
theta1 =  0;
a1 = 0;
alpha1 = -pi/2;

% line segment O1 - 02
d2=0; % can choose any convenient
theta2 = -pi/2;   
a2 = l2;
alpha2 = 0;

%Joints 2-3 O2-O3
d3=0; % can choose any convenient
theta3 = pi/2  ; 
a3 = 0;
alpha3 = pi/2;


%Joints 3-4 O3-O4
d4 = l3+l4; 
theta4 = 0;
a4 = 0;
alpha4 = -pi/2;

%Joints 4-5 04-05
d5 = 0; 
theta5 = 0;
a5 = 0;
alpha5 = pi/2;

%Joints 6-end effector O6-E
d6 = l5+l6; 
theta6 = 0;
a6 = 0;
alpha6 = 0;

%% Inverse Kinematics Part1. Inverse Position problem
x6 =15; y6 = 10; z6 = 30; roll = deg2rad(30); pitch = deg2rad(45); yaw = deg2rad(15);

o = [x6; y6; z6];
d6 = l5+l6;
R = rotx(roll)*roty(pitch)*rotz(yaw);

oc = o - d6*R*[0;0;1]

norm(oc)^2

x5 = oc(1);
y5 = oc(2);
z5 = oc(3);




t5 = sqrt(x5^2 + y5^2);

qi1 = atan2(y5, x5); %q1!=0 

p5 = sqrt( t5^2 + (z5-l1)^2);
p5^2
l34 = sqrt( l3^2 + l4^2);
D1 = l34^2 + l2^2 
D2 = 2*l34*l2

D3 =((-l34^2 + p5^2 - l2^2)/ (2*l34*l2)) % Cos theorem;
q3 = acos (D3);
psi2 = atan2(z5-l1, t5);
D4 = l34*sin(q3)/p5; %sin theorem;
psi1 = asin(D4) + pi ;
if q3>0
    m1 = 1;
else
    m1 = -1;
end
qi3 = m1*pi/2 - q3;

qi2 = m1*psi1+psi2;

qi4 = 0;

 Rob12 = SerialLink([theta1  d1    a1    alpha1; theta2  d2   a2    alpha2 ; theta3  d3    a3    alpha3; theta4  d4    a4   alpha4]);
 Rob12.name = 'IIwaPosition';
 figure (1);
 Rob12.plot ( [ qi1, qi2, qi3, qi4])


%p0 = [x4; y4; z4];
%p1 = rotz(qi1)*p0;
% Singular cases y3 = 0 and x3 = 0
% Disered values of
%q1 [ -pi ; pi]
%q2 [-phi2; pi/2 - phi2]
% Range of links 1-2: xmax = l2a; ymax = l2a; zmax = l1+l2;



%% Part 2 Inverse orientation problem
%  R = R03 * R36;
R03 = rotz(qi1)*roty(qi2)*roty(qi3);
R36t = R03'*R;
syms q4;
syms q5;
syms q6;
R36 = rotz(q4)*roty(q5)*rotz(q6);

q4 = atan2(R36t(2,3), R36t(1,3));
V1 = sqrt(R36t(1,3)^2+R36t(2,3)^2);
q5 = atan2(R36t(3,3), V1);
q6 = atan2(-R36t(3,1), R36t(3,2));

 Rob2 = SerialLink([theta1  d1    a1    alpha1; theta2  d2   a2    alpha2 ; theta3  d3    a3    alpha3; theta4  d4    a4   alpha4; theta5  d5    a5    alpha5; theta6  d6   a6    alpha6]);
 Rob2.name = 'IIwaPositionandOrientation';
 figure (2);
 Rob2.plot ( [ qi1, qi2, qi3, q4, q5, q6])
 Q = [ rad2deg(qi1),  rad2deg(qi2),  rad2deg(qi3),  rad2deg(q4),  rad2deg(q5),  rad2deg(q6)]























