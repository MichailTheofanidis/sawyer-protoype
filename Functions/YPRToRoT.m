%% Conversion from Rotation Matrix fo Yaw, Pitch, Roll
%% byMichail Theofanidis


function [Rot_ang] = YPRToRoT(a,b,g)


%%  Convert to Rad
a_r=a*pi/180;
b_r=b*pi/180;
g_r=g*pi/180;

%% Trigonometric abbreviations

ca=cos(a_r);
sa=sin(a_r);
cb=cos(b_r);
sb=sin(b_r);
cg=cos(g_r);
sg=sin(g_r);

%%Rotation matrix elements

r11 = ca*cb;
r12 = ca*sb*sg-sa*cg;
r13 = ca*sb*cg+sa*sg;
r21 = sa*cb;
r22 = sa*sb*sg+ca*cg;
r23 = sa*sb*cg-ca*sg;
r31 = -sb;
r32 = cb*sg;
r33 = cb*cg;

%%Roll Pitch Yaw Rotation Matrix
Rot_ang=[r11 r12 r13;r21 r22 r23;r31 r32 r33]

end
