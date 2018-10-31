%% Script to control the sawyer robot

%Define the position
x=0.5;
y=-0.5;
z=0;

%Solve the IK and get the joint angles
[finalsolutions] = getSawyerIK_3DOF(x,y,z);

%Show a list of the solutions
disp(rad2deg(finalsolutions));

%Askthe user for a choice
prompt = 'Enter the solution you would like and press Enter ';
choice = input(prompt)

%Show the final Transformation matrix
[Te,T_f,T]=getSawyerFK_DH(finalsolutions(choice,:),1);
disp(Te);

%Plot the robot
plotSawyer(T);

%Pause to check
pause()

%diplay the joint angles
disp(rad2deg(finalsolutions(choice,:)))

%Generate message
msg= JointMessage(finalsolutions(choice,:),1);

%Send the message to the Robot
for i=1:100
    send(jointPub,msg)
    pause(0.1)
end
