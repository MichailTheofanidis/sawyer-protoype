
%% Subscribe
sub1 = rossubscriber('/robot/joint_states');
sub2 = rossubscriber('/forward_model_prediction');

while(1)

    %% Receive Data
    data=receive(sub1);
    prediction=receive(sub2);

    %% Assign the Joints
    q=[data.Position(2) data.Position(3) data.Position(4) data.Position(5) data.Position(6) data.Position(7) data.Position(8)];


    %% Display the Joints
    disp('---------------------------------')
    disp(q)
    disp([prediction.X prediction.Y prediction.Z])

    %% Display the Robot
    figure(1)
    plot3(prediction.X, prediction.Y, prediction.Z,'r*')
    drawnow
    [TeR,TrR,TR]=getSawyerFK_R((q));
    disp(TeR)
    plotSawyer(TR)

    [alpha,beta,gamma] = RotToYPR(TeR);

    %disp(alpha)
    %disp(beta)
    %disp(gamma)

end
