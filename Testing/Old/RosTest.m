%% Trying to publish something...

message = rosmessage('intera_core_msgs/JointCommand');
message.Mode=1;

string = java_array('java.lang.String', 7);
string(1) = java.lang.String('right_j0');
string(2) = java.lang.String('right_j1');
string(3) = java.lang.String('right_j2');
string(4) = java.lang.String('right_j3');
string(5) = java.lang.String('right_j4');
string(6) = java.lang.String('right_j5');
string(7) = java.lang.String('right_j6');

message.Names=cell(string);
message.Position=[0 0 0 0 0 0 0];
message.Header.Stamp.Sec=0;
message.Header.Stamp.Nsec=0;

for i=1:100
    send(jointPub,message)
    pause(0.1)
end