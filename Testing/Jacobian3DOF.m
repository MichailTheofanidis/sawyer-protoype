function [J] = Jacobian3DOF(q1,q2,q4)

% robot parameters
a_1=0.0814619;
d_1=0.0499419;

l_2=0.142537;
d_2=0.140042;

l_3=0.259989;
d_3=0.0419592;

l_4=0.126442;
d_4=0.1224936;

l_5=0.274653;
d_5=0.031188;

l_6=0.105515;
d_6=0.109824;

%l_e=0.0695;
l_e=0.0245;

% simplify the parameters of the robot
L1=d_3+l_4-d_1-l_2-d_5-l_6;
L2=d_4+l_5+d_6+l_e;
L3=d_2+l_3;

%joint offsets
q2=q2+deg2rad(90);

%3-DOF Jacobian
J=[cos(q1)*L1-sin(q1)*a_1-sin(q1)*sin(q2+q4)*L2-sin(q1)*sin(q2)*L3 cos(q1)*cos(q2)*L3+cos(q1)*cos(q2+q4)*L2 cos(q1)*cos(q2+q4)*L2;
   sin(q1)*L1+cos(q1)*a_1+cos(q1)*sin(q2+q4)*L2+cos(q1)*sin(q2)*L3 sin(q1)*cos(q2)*L3+sin(q1)*cos(q2+q4)*L2 sin(q1)*cos(q2+q4)*L2;
   0 -sin(q2)*L3-sin(q2+q4)*L2 -sin(q2+q4)*L2;
   0 -sin(q1) 0;
   cos(q1) 0 1;
   1 0 0;];
 
end

