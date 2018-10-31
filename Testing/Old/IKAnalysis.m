%% Analyzing the IK data

clc
clear all

load('data.mat');

N=size(solutions);

counter=1;

for i=1:N(3)
    
    solutions(:,:,i);
    
    theta4=round(solutions(1,3,i),0);
    theta2=round(solutions(1,2,i),0);
    theta1=round(solutions(1,1,i),0)
    
    theta2sols=round(solutions(3,:,i),0);
    theta4sols=round(solutions(2,:,i),0);
    theta1sols=round(solutions(4,:,i),0)
    
    a=find(theta2sols==theta2);
    b=find(theta4sols==theta4);
    c=find(theta1sols==theta1);
    
%     value=size(c)
%     
%     if value(2)>4
%        pause() 
%     end
    
    if(isempty(a) || isempty(b) || isempty(c))
        
        check(counter)=i
        
        counter=counter+1;
        
    end
    
    %pause()
    
end