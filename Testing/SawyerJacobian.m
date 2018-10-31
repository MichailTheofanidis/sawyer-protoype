%% Script to calculate the Jacobian of the Sawyer Robot
%% by Michail Theofanidis

clear all
clc

%% Load the transformation matrices and the transformation tree 

load('RobotData2.mat')

%% Compare the data

% estimated velocity
V=zeros(length(x),6);

% estimated position
X=zeros(length(x),1);
Y=zeros(length(y),1);
Z=zeros(length(z),1);

dX=zeros(length(x),1);

for i=1:1:length(x)
        
    q=[q1(i) q2(i) 0 q4(i) 0 0 0];
    
    [Te,~,T]=getSawyerFK_DH(q,1);
    
    X(i)=Te(1,4);
    Y(i)=Te(2,4);
    Z(i)=Te(3,4);
    
    dq=[dq1(i) dq2(i) dq4(i)]';
    
    V(i,:)=[Jacobian3DOF(q1(i),q2(i),q4(i))*dq]';
       
end

%% Plot the comparison

DX=diff(x)./0.0099;
DY=diff(y)./0.0099;
DZ=diff(z)./0.0099;

figure(1)
plot([1:length(V(:,1))],V(:,1),'r',[1:length(DX)],DX,'b')

figure(2)
plot([1:length(V(:,2))],V(:,2),'r',[1:length(DY)],DY,'b')

figure(3)
plot([1:length(V(:,3))],V(:,3),'r',[1:length(DZ)],DZ,'b')

% figure(4)
% plot([1:length(V(:,4))],V(:,4),'r')
% 
% figure(5)
% plot([1:length(V(:,5))],V(:,5),'r')
% 
% figure(6)
% plot([1:length(V(:,6))],V(:,6),'r')







