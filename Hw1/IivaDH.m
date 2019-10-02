clear; clc;
% Kuka Iiwa (6 Dofs)
%Given:
syms l1;        
syms l2;
syms l3;
syms l4;
syms l5;
syms l6;

syms q1;
syms q2;
syms q3;
syms q4;
syms q5;
syms q6;

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


% DH-Parameters Using Peter Corke Robotics toolbox


% Create Link using this code
% L   = Link ( [ Th  d    a   alph])

 L(1)= Link ( [theta1  d1    a1    alpha1]); % Link 1
  
 L(2)= Link ( [theta2  d2    a2   alpha2]); % Link 2.2
  
 L(3)= Link ( [theta3  d3    a3    alpha3]); % Link 3
  
 L(4)= Link ( [theta4  d4    a4    alpha4]); % Link 4
   
 L(5)= Link ( [theta5  d5    a5    alpha5]); % Link 5
   
 L(6)= Link ( [theta6  d6    a6    alpha6]); % Link 6
 
 % Robot plot
  
 q1 = deg2rad(0) ; q2 = deg2rad(30) ; q3 = deg2rad(30); % q3 is always equal to pi/2 /(it's not a joint)
 q4 = deg2rad(0); q5 = deg2rad(0); q6 =deg2rad(0); 

  Rob = SerialLink(L);
  Rob.name = 'IIwa';
  Rob.plot ( [ q1, q2, q3, q4, q5, q6])
  %% Testing with Inverse Kinematics;
   figure(3)
   Rob.plot ( [  deg2rad(138.5606),  deg2rad(269.9425),  deg2rad(138.2241), deg2rad(-142.1620),   deg2rad(54.8055),  deg2rad(-94.802)])
   Rob.fkine( [deg2rad(138.5606) deg2rad(269.9425) deg2rad(138.2241) deg2rad(-142.1620)  deg2rad(54.8055) deg2rad(-94.802)])
 

%% Algebraic approach of Direct Kinematics

q1 = deg2rad(0) ; q2 = deg2rad(0) ; q3 = deg2rad(0); % q3 is always equal to pi/2 /(it's not a joint)
q4 = deg2rad(0); q5 = deg2rad(0); q6 = deg2rad(0); 
T1 = trotz(q1)*transl(0, 0, l1)*troty(q2)*transl(l2, 0, 0)*troty(q2)*trotz(q3)*transl(0, 0, l3+l4)*troty(q4)*transl(0, 0, l5+l6);
% Another way to 








