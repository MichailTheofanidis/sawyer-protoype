%% Analysis of the Sawyer

clear
clc

syms theta1 theta2 theta3 theta4 theta5 theta6 theta7 l_0 l_1 d_1 l_2 d_2 l_3 d_3 l_4 d_4 l_5 d_5 l_6 d_6 l_e a_1
 
theta1;
theta2;
theta4;

theta3;
theta5;
theta6;
theta7;

%% robot parameters
l_0=0.079;

l_1=0.237;
a_1=0.081;
d_1=0.049;

l_2=0.142;
d_2=0.14;

l_3=0.259;
d_3=0.0419;

l_4=0.1264;
d_4=0.12249;

l_5=0.274;
d_5=0.031;

l_6=0.105;
d_6=0.109;

l_e=0.0695;

%% Sawyer DH Table
DH_Table=[a_1 -pi/2 0 theta1;
    0 pi/2 d_1+l_2 theta2;
    0 -pi/2 d_2+l_3 theta3;
    0 pi/2 -(d_3+l_4) theta4;
    0 -pi/2 d_4+l_5 theta5;
    0 pi/2 d_5+l_6 theta6;
    0 0 d_6+l_e theta7;];

%% Initialize the transformations
T=sym(zeros(4,4,7));

%% Create the Homogeneous matrixes
for i=1:7
    
    if i==7
        
        % This is for the end effector
        T(:,:,i)=DH_matrix(DH_Table(i,1),DH_Table(i,2),DH_Table(i,3),DH_Table(i,4));
        
    else
        
        % This is for the res of the body
        T(:,:,i)=DH_matrix(DH_Table(i,1),DH_Table(i,2),DH_Table(i,3),DH_Table(i,4));
        
    end
    
end

%% number of transforms
N=size(T);

%% forward pass of the tree
T_f=T(:,:,1);

for i=2:(N(3))
    
    T_f(:,:,i)=T_f(:,:,i-1)*T(:,:,i);
    
end

%% simplify the calculations

x=simplify(T_f(1,4,7));
y=simplify(T_f(2,4,7));
z=simplify(T_f(3,4,7));



