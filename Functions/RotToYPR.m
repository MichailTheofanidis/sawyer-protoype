%% Conversion from Rotation Matrix fo Yaw, Pitch, Roll
%% byMichail Theofanidis


function [alpha,beta,gamma] = RotToYPR(T)

Rot= T(1:3,1:3);

r11 = Rot(1,1);
r12 = Rot(1,2);
r13 = Rot(1,3);
r21 = Rot(2,1);
r22 = Rot(2,2);
r23 = Rot(2,3);
r31 = Rot(3,1);
r32 = Rot(3,2);
r33 = Rot(3,3);

b=atan2(-r31,sqrt((r11^2)+(r21^2)));

a=atan2(r21/cos(b),(r11)/(cos(b)));

g=atan2(r32/cos(b),r33/cos(b));

alpha=a*180/pi;

beta=b*180/pi;

gamma=g*180/pi;


end

