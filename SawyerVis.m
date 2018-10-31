%% Subscribe
sub = rossubscriber('/robot/joint_states');

while(1)
    
    %% Receive Data
    data=receive(sub);

    %% Assign the Joints
    q=radtodeg([data.Position(2) data.Position(3) data.Position(4) data.Position(5) data.Position(6) data.Position(7) data.Position(8)]);
    
    %% Display the Joints
    disp('---------------------------------')
    disp(q)
    
    %% Display the Robot
    figure(1)
    [TeR,TrR,TR]=getSawyerFK_R(degtorad(q));
    disp(TeR)
    plotSawyer(TR)
    
%     figure(1)
%     [TeDH,TrDH,T_DH]=getSawyerFK_DH(degtorad(q),1);
%     disp(TeDH)
%     plotSawyer(T_DH)
    
   
end