clear; clc;
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
clc;
x6 =15; y6 = 10; z6 = 30; roll = deg2rad(30); pitch = deg2rad(45); yaw = deg2rad(15);

o = [x6; y6; z6];
d6 = l5+l6;
R = rotx(roll)*roty(pitch)*rotz(yaw);

oc = o - d6*R*[0;0;1]

norm(oc)^2
x5 = oc(1);
y5 = oc(2);
z5 = oc(3);

% Elbow up

t5 = sqrt(x5^2 + y5^2);

qi1 = atan2(y5, x5); %q1!=0 

p5 = sqrt( t5^2 + (z5-l1)^2);
p5^2
l34 = d4;
D1 = l34^2 + l2^2 
D2 = 2*l34*l2

D3 =(( p5^2 - l34^2  - l2^2)/ (2*l34*l2))  % Cos theorem;
q3 = atan2(sqrt(1-D3^2), D3) ;
 Q3 = rad2deg(q3)
phi2 = atan2(z5-l1, t5);
PHI2 = rad2deg(phi2);
D4 = l34*sin(q3)/p5;%sin theorem;
phi1 = atan2(D4, sqrt(1-D4^2));
PHI1 = rad2deg(phi1);
% Elbow up:
qi3up =  pi/2 - q3;
Qi3up = rad2deg(qi3up);
qi2up =(phi1-phi2);

Qi2up = rad2deg(qi2up)
% Elbow down:
qi3down = pi/2 + q3;

Qi3down = rad2deg(qi3down);

qi2down =(-phi1-phi2);

Qi2down = rad2deg(qi2down);



qi4 = 0;


 Robotup = SerialLink([theta1  d1    a1    alpha1 ; theta2  d2   a2    alpha2 ; theta3  d3    a3    alpha3; theta4  d4    a4   alpha4]);
 Robotup.name = 'IIwaPositionup';
 
 Robotdown = SerialLink([theta1  d1    a1    alpha1 ; theta2  d2   a2    alpha2 ; theta3  d3    a3    alpha3; theta4  d4    a4   alpha4]);
 Robotdown.name = 'IIwaPositiondown';

     
 figure (11)
 [rad2deg(qi1), rad2deg(qi2up), rad2deg(qi3up) , rad2deg(qi4)]
 Robotup.plot ( [ qi1, qi2up, qi3up , qi4]); 
 figure (12)
 [rad2deg(qi1), rad2deg(qi2down), rad2deg(qi3down) , rad2deg(qi4)]
 Robotdown.plot ( [ qi1, qi2down, qi3down , qi4]);


%% Part 2 Inverse orientation problem
%Elbowup
%  R = R03 * R36;
R03up = rotz(qi1)*roty(qi2up)*roty(qi3up);
R36trueup = R03up'*R;
syms q4up;
syms q5up;
syms q6up;
R36 = rotz(q4up)*roty(q5up)*rotz(q6up)

q4up = atan2(R36trueup(2,3), R36trueup(1,3));
V1 = sqrt(R36trueup(1,3)^2+R36trueup(2,3)^2);
q5up = atan2(V1, R36trueup(3,3));
q6up = atan2(R36trueup(3,2), -R36trueup(3,1));

 Robup = SerialLink([theta1  d1    a1    alpha1; theta2  d2   a2    alpha2 ; theta3  d3    a3    alpha3; theta4  d4    a4   alpha4; theta5  d5    a5    alpha5; theta6  d6   a6    alpha6]);
 Robup.name = 'IIwaup';
 figure (21);
 % Elbow up Plot
 figure(21);
 Robup.plot ( [ qi1, qi2up, qi3up, q4up, q5up, q6up])
 Qup = [ rad2deg(qi1),  rad2deg(qi2up),  rad2deg(qi3up),  rad2deg(q4up),  rad2deg(q5up),  rad2deg(q6up)]
 
 %Elbowdown
%  R = R03 * R36;
R03down = rotz(qi1)*roty(qi2down)*roty(qi3down);
R36truedown = R03down'*R;
syms q4down;
syms q5down;
syms q6down;
R36 = rotz(q4down)*roty(q5down)*rotz(q6down)

q4down = atan2(R36truedown(2,3), R36truedown(1,3));
V2 = sqrt(R36truedown(1,3)^2+R36truedown(2,3)^2);
q5down = atan2(V2, R36truedown(3,3));
q6down = atan2(R36truedown(3,2), -R36truedown(3,1));

Robdown = SerialLink([theta1  d1    a1    alpha1; theta2  d2   a2    alpha2 ; theta3  d3    a3    alpha3; theta4  d4    a4   alpha4; theta5  d5    a5    alpha5; theta6  d6   a6    alpha6]);
Robdown.name = 'IIwadown';
% Elbow down Plot
 
 figure (22);
 Robdown.plot ( [ qi1, qi2down, qi3down, q4down, q5down, q6down])
 Q = [ rad2deg(qi1),  rad2deg(qi2down),  rad2deg(qi3down),  rad2deg(q4down),  rad2deg(q5down),  rad2deg(q6down)]
























