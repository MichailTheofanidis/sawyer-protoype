%% Kinematic Analysis of the Sawyer Robot

clear
clc

%% angle values
theta0=deg2rad(0);
theta1=deg2rad(0);%+deg2rad(90);
theta2=deg2rad(0);
theta3=deg2rad(0);
theta4=deg2rad(0);
theta5=deg2rad(0);
theta6=deg2rad(0);%+deg2rad(170);

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

%% frames
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
          0 0 1 -0.005;
          0 0 0 1;];
 
      
T(:,:,10)=[1 0 0 0;
           0 1 0 0;
           0 0 1 0.05;
           0 0 0 1;];
      
%% number of transforms
 N=size(T);
      
      
%% forward pass of the tree
T_f=T(:,:,1);

for i=2:(N(3))

      T_f(:,:,i)=T_f(:,:,i-1)*T(:,:,i);

end

%% testing my forward kinematics

c0=cos(theta0);
c1=cos(theta1);
c2=cos(theta2);
c3=cos(theta3);
c4=cos(theta4);
c5=cos(theta5);
c6=cos(theta6);

s0=sin(theta0);
s1=sin(theta1);
s2=sin(theta2);
s3=sin(theta3);
s4=sin(theta4);
s5=sin(theta5);
s6=sin(theta6);

%% Simplified Version

% A=c0*c4*cos(theta1+theta3)-s0*s4;
% B=s0*c4*cos(theta1+theta3)+c0*s4;
% C=-sin(theta1+theta3)*c4;
% 
% D=-c0*s4*cos(theta1+theta3)-s0*c4;
% E=-s0*s4*cos(theta1+theta3)+c0*c4;
% F=sin(theta1+theta3)*s4;
% 
% G=c0*sin(theta1+theta3);
% I=s0*sin(theta1+theta3);
% J=cos(theta1+theta3);
% 
% T_5_simple=[A D G;
%             B E I;
%             C F J;];
% 
% T_5=[((c0*c1*c2-s0*s2)*c3-(c0*s1)*s3)*c4+(-c0*c1*s2-s0*c2)*s4 -((c0*c1*c2-s0*s2)*c3-(c0*s1)*s3)*s4+(-c0*c1*s2-s0*c2)*c4 (c0*c1*c2-s0*s2)*s3+(c0*s1)*c3 d_4*(-(c0*c1*c2-s0*s2)*s3-(c0*s1)*c3)+l_4*(-c0*c1*s2-s0*c2)+d_3*(-c0*c1*s2-s0*c2)+l_3*(c0*s1)+d_2*(-c0*s1)+l_2*(-s0)+a_1*c0-d_1*s0;
%      ((s0*c1*c2+c0*s2)*c3-(s0*s1)*s3)*c4+(-s0*c1*s2+c0*c2)*s4 -((s0*c1*c2+c0*s2)*c3-(s0*s1)*s3)*s4+(-s0*c1*s2+c0*c2)*c4 (s0*c1*c2+c0*s2)*s3+(s0*s1)*c3 d_4*(-(s0*c1*c2+c0*s2)*s3-(s0*s1)*c3)+l_4*(-s0*c1*s2+c0*c2)+d_3*(-s0*c1*s2+c0*c2)+l_3*(s0*s1)+d_2*(-s0*s1)+l_2*c0+a_1*s0+d_1*c0;
%      (-s1*c2*c3-c1*s3)*c4+(s1*s2)*s4 -(-s1*c2*c3-c1*s3)*s4+(s1*s2)*c4 -s1*c2*s3+c1*c3 d_4*(s1*c2*s3-c1*c3)+l_4*(s1*s2)+d_3*(s1*s2)+l_3*c1+d_2*(-c1)+l_1+l_0;
%       0 0 0 1;];
% 
% r11=(A*c5-G*s5)*(-s6)+D*c6;
% r21=(B*c5-I*s5)*(-s6)+E*c6;
% r31=(C*c5-J*s5)*(-s6)+F*c6;
%  
% r12=-(A*c5-G*s5)*(c6)-D*s6;
% r22=-(B*c5-I*s5)*(c6)-E*s6;
% r32=-(C*c5-J*s5)*(c6)-F*s6;
%  
% r13=A*s5+G*c5;
% r23=B*s5+I*c5;
% r33=C*s5+J*c5;
% 
%  T_10=[r11 r12 r13;
%        r21 r22 r23;
%        r31 r32 r33;
%        0 0 0;];

T_2=[c0*c1 -c0*s1 -s0 a_1*c0-d_1*s0;
     s0*c1 -s0*s1 c0 a_1*s0+d_1*c0;
     -s1 -c1 0 l_1+l_0;
     0 0 0 1;];

T_3=[c0*c1*c2-s0*s2 -c0*c1*s2-s0*c2 c0*s1 d_2*(-c0*s1)+l_2*(-s0)+a_1*c0-d_1*s0;
     s0*c1*c2+c0*s2 -s0*c1*s2+c0*c2 s0*s1 d_2*(-s0*s1)+l_2*c0+a_1*s0+d_1*c0;
     -s1*c2 s1*s2 c1 d_2*(-c1)+l_1+l_0;
      0 0 0 1;];

T_4=[(c0*c1*c2-s0*s2)*c3-(c0*s1)*s3 -(c0*c1*c2-s0*s2)*s3-(c0*s1)*c3 -c0*c1*s2-s0*c2 d_3*(-c0*c1*s2-s0*c2)+l_3*(c0*s1)+d_2*(-c0*s1)+l_2*(-s0)+a_1*c0-d_1*s0;
     (s0*c1*c2+c0*s2)*c3-(s0*s1)*s3 -(s0*c1*c2+c0*s2)*s3-(s0*s1)*c3 -s0*c1*s2+c0*c2 d_3*(-s0*c1*s2+c0*c2)+l_3*(s0*s1)+d_2*(-s0*s1)+l_2*c0+a_1*s0+d_1*c0;
     -s1*c2*c3-c1*s3 s1*c2*s3-c1*c3 s1*s2 d_3*(s1*s2)+l_3*c1+d_2*(-c1)+l_1+l_0;
      0 0 0 1;];

T_5=[((c0*c1*c2-s0*s2)*c3-(c0*s1)*s3)*c4+(-c0*c1*s2-s0*c2)*s4 -((c0*c1*c2-s0*s2)*c3-(c0*s1)*s3)*s4+(-c0*c1*s2-s0*c2)*c4 (c0*c1*c2-s0*s2)*s3+(c0*s1)*c3 d_4*(-(c0*c1*c2-s0*s2)*s3-(c0*s1)*c3)+l_4*(-c0*c1*s2-s0*c2)+d_3*(-c0*c1*s2-s0*c2)+l_3*(c0*s1)+d_2*(-c0*s1)+l_2*(-s0)+a_1*c0-d_1*s0;
     ((s0*c1*c2+c0*s2)*c3-(s0*s1)*s3)*c4+(-s0*c1*s2+c0*c2)*s4 -((s0*c1*c2+c0*s2)*c3-(s0*s1)*s3)*s4+(-s0*c1*s2+c0*c2)*c4 (s0*c1*c2+c0*s2)*s3+(s0*s1)*c3 d_4*(-(s0*c1*c2+c0*s2)*s3-(s0*s1)*c3)+l_4*(-s0*c1*s2+c0*c2)+d_3*(-s0*c1*s2+c0*c2)+l_3*(s0*s1)+d_2*(-s0*s1)+l_2*c0+a_1*s0+d_1*c0;
     (-s1*c2*c3-c1*s3)*c4+(s1*s2)*s4 -(-s1*c2*c3-c1*s3)*s4+(s1*s2)*c4 -s1*c2*s3+c1*c3 d_4*(s1*c2*s3-c1*c3)+l_4*(s1*s2)+d_3*(s1*s2)+l_3*c1+d_2*(-c1)+l_1+l_0;
      0 0 0 1;]

A=T_5(1,1);
B=T_5(2,1);
C=T_5(3,1);
D=T_5(1,2);
E=T_5(2,2);
F=T_5(3,2);
G=T_5(1,3);
I=T_5(2,3);
J=T_5(3,3);

T_6=[A*c5-G*s5 -A*s5-G*c5 D d_5*D+l_5*G+T_5(1,4);
     B*c5-I*s5 -B*s5-I*c5 E d_5*E+l_5*I+T_5(2,4);
     C*c5-J*s5 -C*s5-J*c5 F d_5*F+l_5*J+T_5(3,4);
     0 0 0 1;];

T_7=[(A*c5-G*s5)*c6+D*s6 (A*c5-G*s5)*(-s6)+D*c6 A*s5+G*c5 d_6*(-A*s5-G*c5)+l_6*D+d_5*D+l_5*G+T_5(1,4);
     (B*c5-I*s5)*c6+E*s6 (B*c5-I*s5)*(-s6)+E*c6 B*s5+I*c5 d_6*(-B*s5-I*c5)+l_6*E+d_5*E+l_5*I+T_5(2,4);
     (C*c5-J*s5)*c6+F*s6 (C*c5-J*s5)*(-s6)+F*c6 C*s5+J*c5 d_6*(-C*s5-J*c5)+l_6*F+d_5*F+l_5*J+T_5(3,4);
     0 0 0 1;];


T_8=[(A*c5-G*s5)*(-s6)+D*c6 -(A*c5-G*s5)*(c6)-D*s6 A*s5+G*c5 (A*s5+G*c5)*0.0245+d_6*(-A*s5-G*c5)+l_6*D+d_5*D+l_5*G+T_5(1,4);
     (B*c5-I*s5)*(-s6)+E*c6 -(B*c5-I*s5)*(c6)-E*s6 B*s5+I*c5 (B*s5+I*c5)*0.0245+d_6*(-B*s5-I*c5)+l_6*E+d_5*E+l_5*I+T_5(2,4);
     (C*c5-J*s5)*(-s6)+F*c6 -(C*c5-J*s5)*(c6)-F*s6 C*s5+J*c5 (C*s5+J*c5)*0.0245+d_6*(-C*s5-J*c5)+l_6*F+d_5*F+l_5*J+T_5(3,4);
     0 0 0 1;];
 
 r11=(A*c5-G*s5)*(-s6)+D*c6;
 r21=(B*c5-I*s5)*(-s6)+E*c6;
 r31=(C*c5-J*s5)*(-s6)+F*c6;
 
 r12=-(A*c5-G*s5)*(c6)-D*s6;
 r22=-(B*c5-I*s5)*(c6)-E*s6;
 r32=-(C*c5-J*s5)*(c6)-F*s6;
 
 r13=A*s5+G*c5;
 r23=B*s5+I*c5;
 r33=C*s5+J*c5;
 
 T_10=[r11 r12 r13 r13*0.0695+T_7(1,4);
       r21 r22 r23 r23*0.0695+T_7(2,4);
       r31 r32 r33 r33*0.0695+T_7(3,4);
       0 0 0 1;]
   
 disp(T_f(:,:,10))

 
 