%% Sawyer Robot Inverse Kinematics with DH parameters with 3 DOF
%% by Michail Theofanidis

% rostopic pub -r 10 /robot/limb/right/joint_comma intera_core_msgs/JointCommand '{mode: 1, names: ['right_j0'], position: [1.5]}'

function [finalsolutions] = getSawyerIK_3DOF(xe,ye,ze)

%% robot parameters
l_0=0.0794732;

l_1=0.237;
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

l_e=0.0695;
%l_e=0.0245;

%% parameters for the IK solution
L1=d_3+l_4-d_1-l_2-d_5-l_6;
L2=d_4+l_5+d_6+l_e;
L3=d_2+l_3;

%% Remove the extra length
ze=ze-l_0-l_1;

%% Solution for theta4

c4(1)=((sqrt(xe^2+ye^2-L1^2)-a_1)^2+ze^2-L2^2-L3^2)/(2*L2*L3);
c4(2)=((-sqrt(xe^2+ye^2-L1^2)-a_1)^2+ze^2-L2^2-L3^2)/(2*L2*L3);

c4(1)=Check(c4(1));
c4(2)=Check(c4(2));

s4(1)=sqrt(1-c4(1)^2);
s4(2)=sqrt(1-c4(2)^2);

Mytheta4(1)=atan2(s4(1),c4(1));
Mytheta4(2)=-atan2(s4(1),c4(1));
Mytheta4(3)=atan2(s4(2),c4(2));
Mytheta4(4)=-atan2(s4(2),c4(2));

%% Solution for theta2

YP(1)=sqrt(xe^2+ye^2-L1^2)-a_1;
YP(2)=-sqrt(xe^2+ye^2-L1^2)-a_1;

XP=ze;

k1(1)=L3+L2*c4(1);
k1(2)=L3+L2*c4(2);

k2(1)=L2*s4(1);
k2(2)=L2*s4(2);

% Solution for 1rst theta4 and 1rst X
Mytheta2(1)=atan2(YP(1),XP)+atan2(k2(1),k1(1));
Mytheta2(1)=CheckRange(Mytheta2(1));

Mytheta2(2)=atan2(YP(1),XP)-atan2(k2(1),k1(1));
Mytheta2(2)=CheckRange(Mytheta2(2));

% Solution for 2nd theta4 and 1rst X
Mytheta2(3)=atan2(YP(1),XP)+atan2(k2(2),k1(2));
Mytheta2(3)=CheckRange(Mytheta2(3));

Mytheta2(4)=atan2(YP(1),XP)-atan2(k2(2),k1(2));
Mytheta2(4)=CheckRange(Mytheta2(4));

% Solution for 1rst theta4 and 2nd X
Mytheta2(5)=atan2(YP(2),XP)+atan2(k2(1),k1(1));
Mytheta2(5)=CheckRange(Mytheta2(5));

Mytheta2(6)=atan2(YP(2),XP)-atan2(k2(1),k1(1));
Mytheta2(6)=CheckRange(Mytheta2(6));

% Solution for 2nd theta4 and 2nd X
Mytheta2(7)=atan2(YP(2),XP)+atan2(k2(2),k1(2));
Mytheta2(7)=CheckRange(Mytheta2(7));

Mytheta2(8)=atan2(YP(2),XP)-atan2(k2(2),k1(2));
Mytheta2(8)=CheckRange(Mytheta2(8));

%Final solution
Mytheta2(9)=-Mytheta2(1);

Mytheta2(10)=-Mytheta2(8);

%% Solution for theta1

%number of previous solutions
N_4=size(Mytheta4);
N_2=size(Mytheta2);

%initialize the array that stores the A parameters
A_size=N_4(2)*N_2(2);
A=zeros(1,A_size);

%compute all the different possible A parameters
n=0;
for a=1:N_4(2)
    for b=1:N_2(2)
        
        n=n+1;
        A(n)=a_1+(cos(Mytheta2(b))*sin(Mytheta4(a)) + cos(Mytheta4(a))*sin(Mytheta2(b)))*L2+sin(Mytheta2(b))*L3;
        
    end
end

XP1=A.*ye+xe*L1;
YP1=A.*xe-ye*L1;

Mytheta1plus=zeros(1,A_size);
for m=1:A_size
    Mytheta1plus(m)=atan2(XP1(m),YP1(m));
end

%Add also the negative solutions
Mytheta1=[Mytheta1plus -Mytheta1plus];

%% List of possible solutions
theta1solsN=size(Mytheta1);
theta2solsN=size(Mytheta2);
theta4solsN=size(Mytheta4);

solutions=zeros(theta1solsN(2)*theta2solsN(2)*theta4solsN(2),7);

count=0;

for i=1:theta1solsN(2)

    for j=1:theta2solsN(2)
        
        for z=1:theta4solsN(2)
            count=count+1;
            solutions(count,:)=[Mytheta1(i) Mytheta2(j) 0 Mytheta4(z) 0 0 0;];
        end
        
    end
    
end

%% Find unique solutions
unique_solutions=unique(solutions,'rows');

%% Find the best solutions
limit=size(unique_solutions);

%Vector of estimations
v=zeros(limit(1),3);

for i=1:limit(1)
    [T,~,~]=getSawyerFK_DH(unique_solutions(i,:),0);
    [x,y,z]=MyTransl(T);
    
    %% subtract the missing length from the bottom
    z=z-l_0-l_1;
    
    v(i,:)=[x y z];
    
end

%accepted solutions
number=0;
accepted=zeros(1,1);

for i=1:limit(1)
    if Close(v(i,1),xe) && Close(v(i,2),ye) && Close(v(i,3),ze)
        number=number+1;
        accepted(number)=i;
        
    end
end

%% Find the final solutions
finalsolutions=unique_solutions(accepted(:),:);

%% Sort the solution from the positive to the negatives
finalsolutions=sortrows(finalsolutions);

%% Aplly the joint off-set
finalsolutions(:,2)=finalsolutions(:,2)-deg2rad(90);
finalsolutions(:,7)=finalsolutions(:,7)-deg2rad(170)-deg2rad(90);

end

