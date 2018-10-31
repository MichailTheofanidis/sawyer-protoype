%% Trying to mach the DH and non-DH model of the Sawyer Models

clear all
clc

%% angle values
theta0=deg2rad(0);
theta1=deg2rad(0)+deg2rad(90);
theta2=deg2rad(0);
theta3=deg2rad(0);
theta4=deg2rad(-10);
theta5=deg2rad(0);
theta6=deg2rad(0)+deg2rad(170);

%% robot parameters
l_0=0.079;

l_1=0.237;
a_1=0.081;
d_1=0.049;

l_2=0.142;
d_2=-0.14;

l_3=0.259;
d_3=-0.0419;

l_4=-0.1264;
d_4=-0.12249;

l_5=0.274;
d_5=0.031;

l_6=0.105;
d_6=-0.109;

l_e=0.0695;

%% frames of the Non-DH model
T(:,:,1)=[cos(theta0) -sin(theta0) 0 0;
    sin(theta0) cos(theta0) 0 0;
    0 0 1 l_0;
    0 0 0 1;];

T(:,:,2)=[cos(theta1) -sin(theta1) 0 a_1;
    0 0 1 d_1;
    -sin(theta1) -cos(theta1) 0 l_1;
    0 0 0 1;];

T(:,:,3)=[cos(theta2) -sin(theta2) 0 0;
    0 0 -1 d_2;
    sin(theta2) cos(theta2) 0 l_2;
    0 0 0 1;];

T(:,:,4)=[cos(theta3) -sin(theta3) 0 0;
    0 0 1 d_3;
    -sin(theta3) -cos(theta3) 0 l_3;
    0 0 0 1;];

T(:,:,5)=[cos(theta4) -sin(theta4) 0 0;
    0 0 -1 d_4;
    sin(theta4) cos(theta4) 0 l_4;
    0 0 0 1;];

T(:,:,6)=[cos(theta5) -sin(theta5) 0 0;
    0 0 1 d_5;
    -sin(theta5) -cos(theta5) 0 l_5;
    0 0 0 1;];

T(:,:,7)=[cos(theta6) -sin(theta6) 0 0;
    0 0 -1 d_6;
    sin(theta6) cos(theta6) 0 l_6;
    0 0 0 1;];

T(:,:,8)=[0 -1 0 0;
    1 0 0 0;
    0 0 1 0.0245;
    0 0 0 1;];

T(:,:,9)=[1 0 0 0;
    0 1 0 0;
    0 0 1 0.0450;
    0 0 0 1;];

%% number of transforms
N=size(T);

%% forward pass of the tree
T_f=T(:,:,1);

for i=2:(N(3))
    
    T_f(:,:,i)=T_f(:,:,i-1)*T(:,:,i);
    
end

%% Sawyer DH Table, or at least what I think it is

DH_Table=[abs(a_1) -pi/2 0 theta0;
    0 pi/2 abs(d_1)+abs(l_2) theta1;
    0 -pi/2 abs(d_2)+abs(l_3) theta2;
    0 pi/2 -(abs(d_3)+abs(l_4)) theta3;
    0 -pi/2 abs(d_4)+abs(l_5) theta4;
    0 pi/2 abs(d_5)+abs(l_6) theta5;
    0 0 abs(d_6)+abs(l_e) theta6+deg2rad(90);];

%% Get Size of the joint values
len=size(DH_Table);

%% Initialize the transformations
T_DH=zeros(4,4,len(1));

%% Create the Homogeneous matrixes
for i=1:(len(1))
    
    if i==(len(1))
        
        % This is for the end effector
        T_DH(:,:,i)=DH_matrix(DH_Table(i,1),DH_Table(i,2),DH_Table(i,3),DH_Table(i,4));
        
    else
        
        % This is for the res of the body
        T_DH(:,:,i)=DH_matrix(DH_Table(i,1),DH_Table(i,2),DH_Table(i,3),DH_Table(i,4));
        
    end
    
end

%% number of transforms
N_DH=size(T_DH);

%% forward pass of the tree
T_f_DH=T_DH(:,:,1);

for i=2:(N_DH(3))
    
    T_f_DH(:,:,i)=T_f_DH(:,:,i-1)*T_DH(:,:,i);
    
end

%% add the missing length from the bottom
T_f_DH(3,4,:)=T_f_DH(3,4,:)+l_0+l_1;

disp('DH Tf:')
disp(T_f_DH(:,:,7))

disp('Regular Tf:')
disp(T_f(:,:,9))

q=[theta0 theta1-deg2rad(90) theta2 theta3 theta4 theta5 theta6-deg2rad(170)];

% [A,~,~] = getSawyerFK_R(q)
% 
% [B,~,~] = getSawyerFK_DH(q)



