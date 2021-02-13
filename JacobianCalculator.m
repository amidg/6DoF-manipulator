clc;
clear all;

%variable declaration
syms d1; syms theta2; syms theta3; syms theta4; syms theta5; syms theta6;
variables = [d1; theta2; theta3; theta4; theta5; theta6];

%forward kinematics
     T_01 = [1  0   0   0;
             0  1   0   0;
             0  0   1   d1;
             0  0   0   1];
         
     T_12 = [cos(theta2)    -sin(theta2)    0   400*cos(theta2);
             sin(theta2)    cos(theta2)     0   400*sin(theta2);
             0                  0           1   40;
             0                  0           0   1];
         
     T_23 = [cos(theta3)    -sin(theta3)    0   400*cos(theta3);
             sin(theta3)    cos(theta3)     0   400*sin(theta3);
             0                  0           1   40;
             0                  0           0   1];
         
     T_34 = [cos(theta4)    0      sin(theta4)    450*cos(theta4);
             sin(theta4)    0      -cos(theta4)   450*sin(theta4);
             0              1          0          65;
             0              0          0          1];
         
     T_45 = [cos(theta5)    0      -sin(theta5)   0;
             sin(theta5)    0      cos(theta5)    0;
             0             -1          0          0;
             0              0          0          1];
         
     T_56 = [cos(theta6)    0      -sin(theta6)   0;
             sin(theta6)    0      cos(theta6)    0;
             0             -1          0          0;
             0              0          0          1];
         
     T_6ee = [1    0      0   100;
              0    1      0    0;
              0    0      1    0;
              0    0      0    1];
     
     T_02 = T_01*T_12;
     T_03 = T_02*T_23;
     T_04 = T_03*T_34;
     T_05 = T_04*T_45;
     T_06 = T_05*T_56;
     T_0ee = T_06* T_6ee; %Homogeneous Tranforms
     T_0w = T_03;
     
%PROJECT 4, PART 1: JACOBIAN J_ref-to-wrist:
%1. Extract Z-variable / 3-rd column of the T matrix
Z01 = [T_01(1,3); T_01(2,3); T_01(3,3)]; Z01 = [0; 0; 0]; %prism joint
Z02 = [T_02(1,3); T_02(2,3); T_02(3,3)];
Z03 = [T_03(1,3); T_03(2,3); T_03(3,3)]; 
Z04 = [T_04(1,3); T_04(2,3); T_04(3,3)];
Z05 = [T_05(1,3); T_05(2,3); T_05(3,3)];
Z06 = [T_06(1,3); T_06(2,3); T_06(3,3)];
Z0 = [Z01, Z02, Z03, Z04, Z05, Z06];

%2. Extract partial derivative of the last column, Position Vector:
PosVectorRaw = [ T_0w(1,4); T_0w(2,4); T_0w(3,4) ];
PosVectorDerivative = sym(zeros(3,6));

%calculate jacobian derivatives
for i = 1:1:length(PosVectorDerivative(1,:))
    PosVectorDerivative(1,i) = diff(PosVectorRaw(1),variables(i)); 
    PosVectorDerivative(2,i) = diff(PosVectorRaw(2),variables(i)); 
    PosVectorDerivative(3,i) = diff(PosVectorRaw(3),variables(i)); 
end

%build jacobian matrix:
J_0w = sym(zeros(6,6));
for i = 1:1:length(PosVectorDerivative(1,:))
    %fill the derivatives:
    J_0w((1:3),i) = PosVectorDerivative(:,i);
    
    %fill the Z-vector:
    J_0w((4:6),i) = Z0(:,i);
end

%3. determine singularity:
A3x3 = J_0w((1:3), (1:3));
C3x3 = J_0w((4:6), (4:6));

detA = det(A3x3);
detC = det(C3x3); %solution is on paper: S3 = 0, theta3 = 90deg

%4. Determine Tv_ee->w problem:
Tv = sym(zeros(6,6));
Tv(1:3,1:3) = [1 0 0; 0 1 0; 0 0 1];
Tv(4:6,4:6) = [1 0 0; 0 1 0; 0 0 1];

%PROJECT 4, PART 2: LINKS MODELLING
%NOTE: Inertia Tensors are in kg * m^2
%Link #1
Ixx = 0.32; 	Ixy = 0.00; 	Ixz = 0.00;
Iyx = 0.00;     Iyy = 0.15; 	Iyz = 0.09;
Izx = 0.00; 	Izy = 0.09;     Izz = 0.21;
TensorLink1 = 1000 * [Ixx, -Ixy, -Ixz; -Iyx, Iyy, -Iyz; -Izx, -Izy, Izz];
CoM_Vector1 = 1000 * [0; 0.1; 0.3]; %x, y, z in meters

%Link #2:
Ixx = 0.11; 	Ixy = 0.00; 	Ixz = 0.00;
Iyx = 0.00; 	Iyy = 0.01; 	Iyz = 0.00;
Izx = 0.00; 	Izy = 0.00; 	Izz = 0.13;
TensorLink2 = 1000 * [Ixx, -Ixy, -Ixz; -Iyx, Iyy, -Iyz; -Izx, -Izy, Izz];
CoM_Vector2 = 1000 * [0; 0.2; 0.05]; %x, y, z in meters

%Link #3:
Ixx = 0.08; 	Ixy = 0.00; 	Ixz = 0.00;
Iyx = 0.00; 	Iyy = 0.01; 	Iyz = 0.01;
Izx = 0.00; 	Izy = 0.01; 	Izz = 0.08;
TensorLink3 = 1000 * [Ixx, -Ixy, -Ixz; -Iyx, Iyy, -Iyz; -Izx, -Izy, Izz];
CoM_Vector3 = 1000 * [0; 0.25; 0.08]; %x, y, z in meters

%PROJECT 4, PART 3: DYNAMICS MODELLING
%NOTE: It is important to define position vector from the Rotation point to
%the Center of Mass of the every piece -> Optimized parts from Part II and
%III are used in the previous section of the Part IV.

%NOTE: WE DO FORMULATIONS FOR ONLY FIRST THREE JOINTS THAT DETERMINE WRIST
%POSITION (PAGE 13 OF THE MANUAL)

%NEUTON-EULER FORMULATION:
