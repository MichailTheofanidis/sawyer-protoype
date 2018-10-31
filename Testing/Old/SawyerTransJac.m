%% Sawyer Robot Jacobian calculation
%% by Michail Theofanidis

clear
clc

%% flag to stop the IK solver
flag=0;

%% angle values
theta0=deg2rad(0);
theta1=deg2rad(0)+deg2rad(90);
theta2=deg2rad(0);
theta3=deg2rad(0);
theta4=deg2rad(0);
theta5=deg2rad(0);
theta6=deg2rad(0)+deg2rad(170);

%% Target Position
Xd=0.8;Yd=0.6;Zd=0.3;

%% Initialize Velocities
deltatheta0=0;deltatheta1=0;deltatheta2=0;deltatheta3=0;deltatheta4=0;deltatheta5=0;deltatheta6=0;

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


while(flag==0)

    %% Update Velocities
    theta0=theta0+deltatheta0;
    theta1=theta1+deltatheta1;
    theta2=theta2+deltatheta2;
    theta3=theta3+deltatheta3;
    theta4=theta4+deltatheta4;
    theta5=theta5+deltatheta5;
    theta6=theta6+deltatheta6;
    
    q=[theta0 theta1-deg2rad(90) theta2 theta3 theta4 theta5 theta6-deg2rad(170)];

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

    %% Plot the Sawyer Robot
    SawyerFK(q)

    %% Calculating the jacobian

    O7=T_f(1:3,4,7);

    Z0=[0;0;1];O=[0;0;0];
    Jv1=cross(Z0,(O7-O));

    Z1=T(1:3,3,1);O1=T_f(1:3,4,1);
    Jv2=cross(Z1,(O7-O1));

    Z2=T(1:3,3,2);O2=T_f(1:3,4,2);
    Jv3=cross(Z2,(O7-O2));

    Z3=T(1:3,3,3);O3=T_f(1:3,4,3);
    Jv4=cross(Z3,(O7-O3));

    Z4=T(1:3,3,4);O4=T_f(1:3,4,4);
    Jv5=cross(Z4,(O7-O4));

    Z5=T(1:3,3,5);O5=T_f(1:3,4,5);
    Jv6=cross(Z5,(O7-O5));

    Z6=T(1:3,3,6);O6=T_f(1:3,4,6);
    Jv7=cross(Z6,(O7-O6));

    Jv=[Jv1 Jv2 Jv3 Jv4 Jv5 Jv6 Jv7];

    disp('Jv:'); disp(Jv);

    %% Displacement
        Xinit=T_f(1,4,10);
        Yinit=T_f(2,4,10);
        Zinit=T_f(3,4,10);
        Xend=Xd;Yend=Yd;Zend=Zd;
        Xspeed=(Xend-Xinit);
        Yspeed=(Yend-Yinit);
        Zspeed=(Zend-Zinit);

        %% Pseudoinverse Jacobian
        thetadot=pinv(Jv)*[Xspeed;Yspeed;Zspeed];

        dis_error=sqrt(Xend^2+Yend^2+Zend^2)- sqrt(Xinit^2+Yinit^2+Zinit^2);


        if abs(dis_error)<=0.001
            flag=1;
        end

        theta1dot=thetadot(1,1);
        theta2dot=thetadot(2,1);
        theta3dot=thetadot(3,1);
        theta4dot=thetadot(4,1);
        theta5dot=thetadot(5,1);
        theta6dot=thetadot(6,1);
        theta7dot=thetadot(7,1);
        
        deltatheta1=theta1dot;
        deltatheta2=theta2dot;
        deltatheta3=theta3dot;
        deltatheta4=theta4dot;
        deltatheta5=theta5dot;
        deltatheta6=theta6dot;
        deltatheta7=theta7dot;
        

        pause(1);
        
        disp('-----------------------')

end




















