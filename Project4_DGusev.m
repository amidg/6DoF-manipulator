%INDIVIDUAL PROJECT FOR MSE429 | FALL 2020
%AUTHOR: DMITRII GUSEV, 301297008

%1. INITIALIZATION (NO CHANGES ARE NECESSARY except for section 1.3)
%clear variables and close figures
clc; 
close all;

%1.1. Define size of figure and create figure handle (DO NOT MODIFY)
set(0,'Units','pixels');
dim = get(0,'ScreenSize'); 
fig_handle = figure('doublebuffer','on','Position',[0,35,dim(3),dim(4)-100],...
    'Name','3D Object','NumberTitle','off');
set(gcf,'color', [1 1 1]) %Background Colour

%1.2 Define the light in the figure (CHANGE POSITION VECTOR IF FIGURE IS TOO BRIGHT/DARK)
set(fig_handle,'Renderer','zbuffer','doublebuffer','off')
light('color',[.5,.5,.5],'position',[0,1,3],'Style','infinite')
lighting gouraud
daspect([1 1 1]);

%1.3 Axes (TO MODIFY Make sure your axes fit within the region) 
%minX, maxX, minY, maxY, minZ, maxZ
axis([-1500 1500 -1500 1500 0 1500]);  %To be changed to include workspace
view(40,30);                           %To be changed to view from other angle
zoom(0.8);                               %To be changed to zoom in/out 
axis on;


%2. LOAD PART FILES       %Load your manipultor links and external objects
%Load STL files of thes chess pieces and the manipulator links

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%PRESET THE DISPLACEMENTS FOR THE COMPONENTS
Object1X = 0;      Object1Y = -70;              Object1Z = 200;
Object2X = 0;      Object2Y = 400+Object1Y;     Object2Z = 600+20;
Object3X = 0;      Object3Y = Object2Y + 400;   Object3Z = 600+20+20+20+20;

%end effector values are the displacements from the Link #3
Object4X = 0;      Object4Y = 400 - 20;         Object4Z = 125; %axis 4
Object5X = 0;      Object5Y = 400 + 30 + 25;    Object5Z = 125 - 5; %axis 5
Object6X = 0;      Object6Y = 400 + 30;         Object6Z = 125; %axis6

%LOAD ALL THE INDIVIDUAL PARTS OF THE MANIPULATOR
%main arm
load 'Arm1.mat' %This is the post where the prismatic joint slides (aligned with z axis)
setappdata(0,'object_data',object);
object = getappdata(0,'object_data');
obj{1}=object;
obj{1}.V=obj{1}.V+repmat([Object1X Object1Y Object1Z],[length(obj{1}.V(:,1)) 1]); %adjust offset by -70mm along Y

load 'Arm2.mat' %This is the link between revolute joints (aligned with x axis)
setappdata(0,'object_data',object);
object = getappdata(0,'object_data');
obj{2}=object;
obj{2}.V=obj{2}.V+repmat([Object2X Object2Y Object2Z],[length(obj{2}.V(:,1)) 1]); %adjust offset to match the joint of the Arm 1

load 'Arm3.mat' %This is link after second revolute joint that includes end effector (aligned with x axis)
setappdata(0,'object_data',object);
object = getappdata(0,'object_data');
obj{3}=object;
obj{3}.V=obj{3}.V+repmat([Object3X Object3Y Object3Z],[length(obj{3}.V(:,1)) 1]); %adjust offset to match the joint of the Arm 2

load 'Prismatic.mat' %This is the read link that represents the prismatic joint (aligned with z axis)
setappdata(0,'object_data',object);
object = getappdata(0,'object_data');
obj{4}=object;
obj{4}.V=obj{4}.V+repmat([0 0 10],[length(obj{4}.V(:,1)) 1]); %adjust offset by 10mm along Z

load 'Base.mat' %This is the square base of the manipulator
setappdata(0,'object_data',object);
object = getappdata(0,'object_data');
obj{5}=object;

%end effector
load 'EndEffectorAxis4.mat' %End Effector, Axis 4
setappdata(0,'object_data',object);
object = getappdata(0,'object_data');
obj{6}=object;
obj{6}.V=obj{6}.V+repmat([Object4X, Object3Y + Object4Y, Object3Z + Object4Z],[length(obj{6}.V(:,1)) 1]); %adjust offset to match the joint of the end effector

load 'EndEffectorAxis5.mat' %End Effector, Axis 5
setappdata(0,'object_data',object);
object = getappdata(0,'object_data');
obj{7}=object;
obj{7}.V=obj{7}.V+repmat([Object5X, Object3Y + Object5Y, Object3Z + Object5Z],[length(obj{7}.V(:,1)) 1]); %adjust offset to match the joint of the end effector

load 'EndEffectorAxis6.mat' %End Effector, Axis 6
setappdata(0,'object_data',object);
object = getappdata(0,'object_data');
obj{8}=object;
obj{8}.V=obj{8}.V+repmat([Object6X, Object3Y + Object6Y, Object3Z + Object6Z],[length(obj{8}.V(:,1)) 1]); %adjust offset to match the joint of the end effector

% Print all the parts of the manipulator. Note all moving parts are
% located at the global reference frame
for i=1:8
    q(i) = patch('faces', obj{i}.F, 'vertices', obj{i}.V);
    set(q(i),'EdgeColor','none');
end

%%COLORS OF THE MANIPULATOR
%main arm
set(q(1),'FaceColor', [1,0.242,0.293]);
set(q(2),'FaceColor', [.4,0.6,0.6]);
set(q(3),'FaceColor', [.6,0.6,0.4]);
set(q(4),'FaceColor', [.9,0.9,0.9]);
set(q(5),'FaceColor', [.9,0.9,0.9]);

%end effector
set(q(6),'FaceColor', [.4,0.6,0.6]);
set(q(7),'FaceColor', [.6,0.6,0.4]);
set(q(8),'FaceColor', [1,0.242,0.293]);

%Rename of all the vertices. This step is redundant obj{}.V will not longer be used.  
%main arm
V1 = obj{1}.V'; %link 1 is Arm 1, obj{1}
V2 = obj{2}.V'; %link 2 is Arm 2, obj{2}
V3 = obj{3}.V'; %link 3 is Arm 3, obj{3}

%end effector
V4 = obj{6}.V'; %End Effector, Rotation Axis 4
V5 = obj{7}.V'; %End Effector, Rotation Axis 5 (contains gripper)
V6 = obj{8}.V'; %End Effector, Rotation Axis 6

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%ANIMATION FUNCTION

RGB=256;  %Resolution
fm = getframe; [img,map] = rgb2ind(fm.cdata,RGB,'nodither');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%PATH GENERATION

%3.2 PATH FOR THE END EFFECTOR:
%POS:  P1  P2  P3  P4   P5   P6   P7   P8   P9   P10  P11  P12  P13  P14  P15
P_ee=[200  200 200  250  250  250  300  300  300  350  350  350  400  450  450; 
      200  200 250  250  250  300  300  300  350  350  350  350  400  400  450;
      80   100 100  120  120  100  100  150  150  200  200  250  250  300  300;
      5    5   5    5    5    5    5    5    5    5    5    5    5    5    5;
      10   10  10   10   10   10   10   10   10   10   10   10   10   10   10;
      7    7   7    7    7    7    7    7    7    7    7    7    7    7    7 ];
%  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%INVERSE KINEMATICS OF THE SCARA MANIPULATOR
%4.1 Inverse Kinematics
  %Define the link dimensions (DH parameters)
    L2 = 400; 
    L3 = 400;
    dee=100; %z axis points upwards, gripper points downwards
    
    joints = zeros(6, 15); %create empty matrix for the path generation
    
    for i = 1:1:length(P_ee) %format is row, column
    
        %retrieve values from the position vectors
        alpha = P_ee(4, i);
        beta = P_ee(5, i);
        gamma = P_ee(6, i);
        
        PeeX = P_ee(1, i);
        PeeY = P_ee(2, i);
        PeeZ = P_ee(3, i);
    
        %this section creates Nxyz, Oxyz, Axyz for the inverse kinematics
        Nx = cosd(alpha)*cosd(beta);
        Ny = sind(alpha)*cosd(beta);
        Nz = -sind(beta);
        Ox = cosd(alpha)*sind(beta)*sind(gamma) - sind(alpha)*cosd(gamma);
        Oy = sind(alpha)*sind(beta)*sind(gamma) + cosd(alpha)*cosd(gamma);
        Oz = cosd(beta)*sind(gamma);
        Ax = cosd(alpha)*sind(beta)*cosd(gamma) + sind(alpha)*sind(gamma);
        Ay = sind(alpha)*sind(beta)*cosd(gamma) - cosd(alpha)*sind(gamma);
        Az = cosd(beta)*cosd(gamma);
        
        %compare to Project Part 2:
        %P_ee(1) is replaced with PeeX
        %P_ee(2) is replaced with PeeY
        %P_ee(3) is replaced with PeeZ
        FullMatrix=[Nx  Ox  Ax  PeeX;
                    Ny  Oy  Ay  PeeY;
                    Nz  Oz  Az  PeeZ]; 

        %middle variables from inverse kinematic solution:
        c3 = (FullMatrix(1,4).^2 + FullMatrix(2,4).^2-320000)/320000;

        %Inverse Kinematics
        %In this section the inverse kinematics of the manipulator is solved.
        %For this particular example solve for each position of the
        %end-effector the three joint displacements

        d1 = FullMatrix(3,4)-80;
        theta3 = atan2(sqrt(1-c3.^2), c3);

        c2 = (FullMatrix(1,4)/400)*((1+c3)/(c3.^2 + 2*c3+sin(theta3).^2+1)) - (FullMatrix(2,4)/400)*(sin(theta3)/(c3.^2 + 2*c3+sin(theta3).^2+1));
        s2 = (FullMatrix(1,4)/400)*(sin(theta3)/(c3.^2 + 2*c3+sin(theta3).^2+1)) + (FullMatrix(2,4)/400)*((1+c3)/(c3.^2 + 2*c3+sin(theta3).^2+1));
        theta2 = atan2(s2, c2);

        %END-EFFECTOR CALCULATIONS:    
        theta6 = atan2(-FullMatrix(3,3), FullMatrix(3,1)); %1 solution is picked

        theta5 = atan2(FullMatrix(3,1)*cos(theta6), -FullMatrix(3,3)*sin(theta6));

        theta4 = atan2(FullMatrix(2,2)/sin(theta5), FullMatrix(1, 2)/sin(theta5));

        %JOINT FORMAT FOR THE SINGLE POSITION FOR PART 2
        %joints = [d1;  theta2; theta3; theta4; theta5; theta6];
        
        %JOINTS MATRIX FILLED FOR THE PART 3
        joints(1, i) = d1; %prismatic joint
        joints(2, i) = theta2;
        joints(3, i) = theta3;
        joints(4, i) = theta4;
        joints(5, i) = theta5;
        joints(6, i) = theta6;
    
    end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%TRANJECTORY GENERATION
%5.1 Trajectory Generation
tf=1;  %Duration of each Segment (if segments have different durations enter it in a vector form)
dt=0.5;  %time steps

%PART 2 -> SINGLE POSE
%D = [joints(1); joints(2); joints(3); joints(4); joints(5); joints(6)];

%PART 3 -> TRAJECTORY
n = length(joints(1,:)); %number of columns of the trajectory generated matrix
stepsize = 0.05;
duration = zeros(n-1);
for i = 1:1:n-1
    duration(i) = 0.5;
end

[D, VelocityForJoints, A, time] = via_points_match_VA(joints, duration, stepsize, 'prescribed', [0,0]);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%ANIMATION:
 mov(1:length(D(1,:))) = struct('cdata', [],'colormap', []);
    [a,b]=size(img); gifim=zeros(a,b,1,n-1,'uint8');
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%FORWARD KINEMATICS
%INPUTS
% Length of links (CHANGE, YOUR DESIGN)
L1 = 400;
L2 = 400;
L3 = 400;

% Link Parameters of DH table (CONSTANT PARAMETERS)
alpha0 = 0;         a0 = 0;                     theta1 = 0; %(d1 is variable)
alpha1 = 0;         a1 = L1;    d2 = 40;        %(theta2 is variable)
alpha2 = 0;         a2 = L2;    d3 = 40;        %(theta3 is variable)
alpha3 = pi/2;      a3 = 0;     d4 = L3;        %(theta4 is variable)
alpha4 = -pi/2;     a4 = 0;     d5 = 0;         %(theta5 is variable)
alpha5 = -pi/2;     a5 = 0;     d6 = 0;         %(theta6 is variable)       
alpha6 = 0;         a6 = 100;   dee = 0;        thetaee = 0;

%this matrix is designed for the further comet3 plotting
cometMatrix = zeros(3, length(D(1,:))); %x; y; z; with length of D

%length of D is ~154
for i=1:length(D(1,:))
     
     %DH parameters calculated from FWD kinematics code, DH above is for reference   
     %d1 is D(1,:)
     %theta2..6 is D(2..6, :)
     d1 = D(1,i);
     theta2 = D(2,i);
     theta3 = D(3,i);
     theta4 = D(4,i);
     theta5 = D(5,i);
     theta6 = D(6,i);
     
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
     
    %Forward Kinematics
            T_02 = T_01*T_12;
            T_03 = T_02*T_23;
            T_04 = T_03*T_34;
            T_05 = T_04*T_45;
            T_06 = T_05*T_56;
            T_0ee = T_06* T_6ee; %Homogeneous Tranforms
    
            %Position and rotation matrices of frame {3} (where link 3 is
            %located) and the end-effector 
            R_01 = T_01(1:3, 1:3);
            R_02 = T_02(1:3, 1:3);
            R_12 = T_12(1:3, 1:3);
            R_23 = T_23(1:3, 1:3);
            R_03 = T_03(1:3,1:3); 
            R_34 = R_23; %T_34(1:3, 1:3);
            R_45 = R_23; %T_45(1:3, 1:3);
            R_56 = R_23; %T_56(1:3, 1:3);
            
            %way to extract position vector as column 4, elements 1 to 3
            P_01 = T_01(1:3,4);
            P_12 = T_12(1:3,4);
            P_02 = T_02(1:3,4);
            P_23 = T_23(1:3,4);
            P_03 = T_03(1:3,4); 
            
            %END EFFECTOR POSITION EXTRACTION
            R_0ee = T_0ee(1:3,1:3);
            P_0ee = T_0ee(1:3,4);
            
     %Move Links of Manipulator            
            %End-effector moves accordingly with T_03, other links will
            %move based on other T_0i. The following lines find the new
            %orientation and position of the vertices of the end-effector.
            
            %note: every position must be adjusted after the update, do
            %same operation as in the load part         
            %FIND NEW ORIENTATION OF LINK 1 (USE V{1})
            newV{1} = R_01*V1; 
            newV{1} = newV{1} + repmat(P_01,[1 length(newV{1}(1,:))]); %Find new position of link 1
            
            %NOTE!!!!!!!
            %REPMAT FUNCTIONS BELOW DO NOT USE P_0i MATRICES BECAUSE MY
            %R_01 IS IDENTITY MATRIX AND IT MESSES UP ALL THE CALCULATIONS
            %THAT RESULT WITH ROTATION OF ALL COMPONENTS AROUND THE GLOBAL
            %ORIGIN AND NOT PART'S ORIGIN!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            
            %FIND NEW ORIENTATION OF LINK 2 (USE V{2})
            newV{2} = R_12*V2; %rotate object on the %P_12 is used for repmat
            newV{2} = newV{2} + repmat([Object2Y*sin(theta2); Object2Y - Object2Y*cos(theta2); P_02(3)-40],[1 length(newV{2}(1,:))]); %Find new position of link 2
            
            %FIND NEW ORIENTATION OF LINK 3 (USE V{3})
            newV{3} = R_23*V3; %Find new orientation of link 3
            newV{3} = newV{3} + repmat([Object3Y*sin(theta3)-L1*sin(theta2); Object2Y+L1*cos(theta2)-Object3Y*cos(theta3); P_02(3)-40],[1 length(newV{3}(1,:))]); %Find new position of link 3
            
            %FIND NEW ORIENTATION OF END EFFECTOR AXIS 4 (USE V{4})
            newV{4} = R_34*V4; 
            newV{4} = newV{4} + repmat([Object3Y*sin(theta3)-L1*sin(theta2); Object2Y+L1*cos(theta2)-Object3Y*cos(theta3); P_02(3)-40],[1 length(newV{4}(1,:))]); 
            
            %FIND NEW ORIENTATION OF END EFFECTOR AXIS 5 (USE V{5})
            %THIS PART CONTAINS ACTUAL GRIPPER
            newV{5} = R_45*V5; 
            newV{5} = newV{5} + repmat([Object3Y*sin(theta3)-L1*sin(theta2); Object2Y+L1*cos(theta2)-Object3Y*cos(theta3); P_02(3)-40],[1 length(newV{5}(1,:))]); 
            
            %FIND NEW ORIENTATION OF END EFFECTOR AXIS 6 (USE V{6})
            newV{6} = R_56*V6; 
            newV{6} = newV{6} + repmat([Object3Y*sin(theta3)-L1*sin(theta2); Object2Y+L1*cos(theta2)-Object3Y*cos(theta3); P_02(3)-40],[1 length(newV{6}(1,:))]); 
            
            %THIS IS HERE JUST TO MAKE CODE WORK -> REDRAW OBJ(4) AND OBJ(5) AS BASE AND LINEAR ACTUATOR
            newV{7} = obj{4}.V'; %q4
            newV{8} = obj{5}.V'; %q5
            
            %USE FOR LOOP TO SET NEW POSITIONS AND DRAW
            for ii=1:1:8
                if (ii >= 1) && (ii <= 3)
                    set(q(ii),'Vertices',newV{ii}(1:3,:)'); %MAIN ARM
                elseif ii == 4 || ii == 5
                    set(q(ii),'Vertices',newV{ii+3}(1:3,:)'); %BASE AND ACTUATOR
                else 
                    set(q(ii),'Vertices',newV{ii-2}(1:3,:)'); %END EFFECTOR
                end
            end
                
     %ANIMATION, saves frames (DO NOT MODIFY)       
     drawnow;  %Draw objects to their new poisitons
     im= frame2im(getframe);
     gifim(:,:,:,i) = rgb2ind(im, map);
     mov(i)=getframe(gcf);
     
     %comet array for the end-effector position
     cometMatrix(1,i) = L2*sin(theta2) + (L3 + 30 + 25 + 100)*sin(theta2 + theta3); %X
     cometMatrix(2,i) = L1 + L2*cos(theta2) + (L3 + 30 + 25 + 100)*cos(theta2 + theta3); %y
     cometMatrix(3,i) = P_02(3)-40; %z
end

%ANIMATION, creates animated gif (DO NOT MODIFY)
imwrite(gifim,map,'MSE429_DGusev_Project.gif','DelayTime',0)%,'LoopCount',inf)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%PART 4 --> JACOBIAN / DYNAMICS AND SIMULATION OF THE MANIPULATOR:
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
     
     R_01 = T_01(1:3, 1:3);
     R_12 = T_12(1:3, 1:3);
     R_23 = T_23(1:3, 1:3);
     
%PROJECT 4, PART 1: JACOBIAN J_ref-to-wrist:
%1. Extract Z-variable / 3-rd column of the T matrix
Z01 = [0; 0; 0]; %prism joint
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
numberOfJoints = 6; %this number here is for the debugging purposes -> I can specify number of joints to model with respect to the manipulator... 
%I usually do first 3 joints then scale solution down to 6 joints when all
%steps are finalized

%Link #1
Ixx = 0.32; 	Ixy = 0.00; 	Ixz = 0.00;
Iyx = 0.00;     Iyy = 0.15; 	Iyz = 0.09;
Izx = 0.00; 	Izy = 0.09;     Izz = 0.21;
TensorLink1 = 1000 * [Ixx, -Ixy, -Ixz; -Iyx, Iyy, -Iyz; -Izx, -Izy, Izz];
CoM_Vector1 = 1000 * [0; 0.1; 0.3]; %x, y, z in MM

%Link #2:
Ixx = 0.11; 	Ixy = 0.00; 	Ixz = 0.00;
Iyx = 0.00; 	Iyy = 0.01; 	Iyz = 0.00;
Izx = 0.00; 	Izy = 0.00; 	Izz = 0.13;
TensorLink2 = 1000 * [Ixx, -Ixy, -Ixz; -Iyx, Iyy, -Iyz; -Izx, -Izy, Izz];
CoM_Vector2 = 1000 * [0; 0.2; 0.05]; %x, y, z in MM

%Link #3:
Ixx = 0.08; 	Ixy = 0.00; 	Ixz = 0.00;
Iyx = 0.00; 	Iyy = 0.01; 	Iyz = 0.01;
Izx = 0.00; 	Izy = 0.01; 	Izz = 0.08;
TensorLink3 = 1000 * [Ixx, -Ixy, -Ixz; -Iyx, Iyy, -Iyz; -Izx, -Izy, Izz];
CoM_Vector3 = 1000 * [0; 0.25; 0.08]; %x, y, z in MM

%Link #4:
Ixx = 18020.40; Ixy = 0.00; 	Ixz = 0.00;
Iyx = 0.00;     Iyy = 18147.68;	Iyz = 0.00;
Izx = 0.00; 	Izy = 0.00; 	Izz = 28413.89;
TensorLink4 = [Ixx, -Ixy, -Ixz; -Iyx, Iyy, -Iyz; -Izx, -Izy, Izz];
CoM_Vector4 = [0; 30.55; 0]; %x, y, z

%Link #5:
Ixx = 85.74;    Ixy = 0.00; 	Ixz = 285.62;
Iyx = 0.00; 	Iyy = 3951.79;	Iyz = 0.00;
Izx = 285.62;	Izy = 0.00; 	Izz = 3905.76;
TensorLink5 = [Ixx, -Ixy, -Ixz; -Iyx, Iyy, -Iyz; -Izx, -Izy, Izz];
CoM_Vector5 = [0; 51.25; -4.99]; %x, y, z

%Link #6:
Ixx = 13294.05; 	Ixy = 0.00;         Ixz = 0.00;
Iyx = 0.00;         Iyy = 17262.79; 	Iyz = 0.00;
Izx = 0.00;         Izy = 0.00;     	Izz = 26973.09;
TensorLink6 = [Ixx, -Ixy, -Ixz; -Iyx, Iyy, -Iyz; -Izx, -Izy, Izz];
CoM_Vector6 = 1000 * [0; 0.02; 0]; %x, y, z

%Center of Mass vectors
CoM_Vector = zeros(3,1,numberOfJoints);
CoM_Vector(:,:,1) = CoM_Vector1;
CoM_Vector(:,:,2) = CoM_Vector2;
CoM_Vector(:,:,3) = CoM_Vector3;
CoM_Vector(:,:,4) = CoM_Vector4;
CoM_Vector(:,:,5) = CoM_Vector5;
CoM_Vector(:,:,6) = CoM_Vector6;

%Inertia Tensors
TensorLink = zeros(3,3,numberOfJoints);
TensorLink(:,:,1) = TensorLink1;
TensorLink(:,:,2) = TensorLink2;
TensorLink(:,:,3) = TensorLink3;
TensorLink(:,:,4) = TensorLink4;
TensorLink(:,:,5) = TensorLink5;
TensorLink(:,:,6) = TensorLink6;

%PROJECT 4, PART 3: DYNAMICS MODELLING
%NOTE: It is important to define position vector from the Rotation point to
%the Center of Mass of the every piece -> Optimized parts from Part II and
%III are used in the previous section of the Part IV.

%NEWTON-EULER RECURISVE FORMULATION FOR THE WHOLE 6-DoF system
%variable declaration:
angVelocity = zeros(3,1,numberOfJoints);
angAccel = zeros(3,1,numberOfJoints);
linAccel = zeros(3,1,numberOfJoints);
linAccerCoM = zeros(3,1,numberOfJoints);
Forces = zeros(3,1,numberOfJoints);
Moments = zeros(3,1,numberOfJoints);

torquesAndForces = zeros(numberOfJoints,length(D)); %~154 values for specified number of joints

linkMass = [3456, 3514, 3394, 558, 324, 108]; %retrived from the report #1, gramms

force = zeros(3,1,numberOfJoints);
n = zeros(3,1,numberOfJoints);

%do the numeric formulation for the whole trajectory
for i=1:length(D(1,:))
     
     %DH parameters calculated from FWD kinematics code, DH above is for reference   
     %d1 is D(1,:)
     %theta2..6 is D(2..6, :)
     d1 = D(1,i);
     theta2 = D(2,i);
     theta3 = D(3,i);
     theta4 = D(4,i);
     theta5 = D(5,i);
     theta6 = D(6,i);
     
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
     
    %Forward Kinematics
    T_02 = T_01*T_12;
    T_03 = T_02*T_23;
    T_04 = T_03*T_34;
    T_05 = T_04*T_45;
    T_06 = T_05*T_56;
    T_0ee = T_06* T_6ee; %Homogeneous Tranforms
    
    %Position and rotation matrices of frame {3} (where link 3 is
    %located) and the end-effector 
    R_xy = zeros(3,3,6);
    R_01 = T_01(1:3, 1:3);  R_xy(:,:,1) = R_01;
    R_12 = T_12(1:3, 1:3);  R_xy(:,:,2) = R_12;
    R_23 = T_23(1:3, 1:3);  R_xy(:,:,3) = R_23;
    R_34 = R_23;            R_xy(:,:,4) = R_34;
    R_45 = R_23;            R_xy(:,:,5) = R_45;
    R_56 = R_23;            R_xy(:,:,6) = R_56; 
            
    %way to extract position vector as column 4, elements 1 to 3
    P_xy = zeros(3,1,6);
    P_01 = T_01(1:3,4);     P_xy(:,:,1) = P_01;
    P_12 = T_12(1:3,4);     P_xy(:,:,2) = P_12;
    P_23 = T_23(1:3,4);     P_xy(:,:,3) = P_23;
    P_34 = T_34(1:3,4);     P_xy(:,:,4) = P_34;
    P_45 = T_45(1:3,4);     P_xy(:,:,5) = P_45;
    P_56 = T_56(1:3,4);     P_xy(:,:,6) = P_56;
            
    %outward computation:
    g = 9800000; %mm/s^2
    linAccel(:,:,1) = [0;0;g];
    for out_i = 1:1:(numberOfJoints - 1) 
        %angular velocity:
        %NOTE: prismatic joint is the first one so I don't care
        theta_dot_iplus1 = [0; 0; D(out_i + 1, i)];
        angVelocity(:,:,out_i+1) = (inv(R_xy(:,:,out_i)))*angVelocity(:,:,out_i) + (theta_dot_iplus1); %last matrix is multiplied by 0,0,1
    
        %angular acceleration:
        theta_ddot_iplus1 = [0; 0; A(out_i + 1, i)];
        angAccel(:,:,out_i+1) = (inv(R_xy(:,:,out_i)))*angAccel(:,:,out_i) + cross((inv(R_xy(:,:,out_i)))*angVelocity(:,:,out_i), (theta_dot_iplus1)) + theta_ddot_iplus1;
        
        %linear acceleration:
        linAccel(:,:,out_i+1) = (inv(R_xy(:,:,out_i)))*( cross(angAccel(:,:,out_i),P_xy(:,:,out_i)) + cross(angVelocity(:,:,out_i), cross(angVelocity(:,:,out_i),P_xy(:,:,out_i))) + linAccel(:,:,out_i) );
        
        %linear acceleration at the Center of Mass:
        linAccerCoM(:,:,out_i+1) = cross(angAccel(:,:,out_i+1), CoM_Vector(:,:,out_i+1)) + cross(angVelocity(:,:,out_i+1), cross(angVelocity(:,:,out_i+1),P_xy(:,:,out_i+1))) + linAccel(:,:,out_i+1);
        
        %force
        Forces(:,:,out_i) = linkMass(out_i) * linAccerCoM(:,:,out_i);
        
        %moments:
        Moments(:,:,out_i) = TensorLink(:,:,out_i)*angAccel(:,:,out_i) + cross( angVelocity(:,:,out_i), TensorLink(:,:,out_i)*angVelocity(:,:,out_i) );
    end
    
    force(:,:,numberOfJoints) = [0; 0; 0]; %NOTE: due to the nature of my system, I don't apply any significant force at the end of the end effector because system does not carry any
    n(:,:,numberOfJoints) = [0; 0; 0]; %weight heavier than 14-22AWG wire end, thus I consider it to be zero: %3f_3 = 3n_3 = 0
    
    %INWARD
    for in_i = (numberOfJoints-1):-1:1 
        %forces
        force(:,:,in_i) = R_xy(:,:,in_i+1)*force(:,:,in_i+1) + Forces(:,:,in_i);
        
        %moments
        n(:,:,in_i) = Moments(:,:,in_i) + R_xy(:,:,in_i+1)*n(:,:,in_i+1) + cross( CoM_Vector(:,:,in_i),Forces(:,:,in_i) ) + cross( P_xy(:,:,in_i+1),(R_xy(:,:,in_i+1)*force(:,:,in_i+1)));
        
        %final torque calculation
        torquesAndForces(in_i,i) = ( transpose(n(:,:,in_i)) ) * [0;0;1]; 
        torquesAndForces(in_i,i) = torquesAndForces(in_i,i) / 10e8; %convert everything back to kg/m/N system
    end
end

%PLOTS:
figure(1);
plot(time(1,:), torquesAndForces(1,:));
title('Force required by the prismatic joint [N]');

figure(2);
plot(time(1,:), torquesAndForces(2,:));
title('Torque required by the rotation joint #2 [N-m]');

figure(3);
plot(time(1,:), torquesAndForces(3,:));
title('Torque required by the rotation joint #3 [N-m]');

figure(4);
plot(time(1,:), torquesAndForces(4,:));
title('Torque required by the rotation joint #4 [N-m]');

figure(5);
plot(time(1,:), torquesAndForces(5,:));
title('Torque required by the rotation joint #5 [N-m]');

