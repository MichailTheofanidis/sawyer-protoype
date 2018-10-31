%% Function to create your own custom message
%% by Michail Theofanidis

function [message] = JointMessage(q,mode)

size_q=size(q);

message = rosmessage('intera_core_msgs/JointCommand');
message.Mode=mode;
message.Header.Stamp.Sec=0;
message.Header.Stamp.Nsec=0;

string = java_array('java.lang.String', size_q(2));

for i=1:size_q(2)

    jnt_string=['right_j' num2str(i-1)];
    
    string(i) = java.lang.String(jnt_string);

end    
    
message.Names=cell(string);
message.Position=q;

end

