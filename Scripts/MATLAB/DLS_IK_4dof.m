%R-2R robotic arm Sim
%Write better optimization parameter for main function

%DLS convergence varies on a lot of factors like initial pose
%Interations allowed and damping constant can vary convergence greatly!

%Written By Akshat Pandey

clear 
clc

syms x y z l1 l2 jacobian thetap1 thetap2 thetayaw thetabuffer ;%Symbolic Toolbox Variables

l1=4;
l2=3;

f1=subs(x-(l1*cos(thetap1)*cos(thetayaw)+l2*cos(thetap2)*cos(thetayaw))); %Function formulation
f2=subs(y-(l1*cos(thetap1)*sin(thetayaw)+l2*cos(thetap2)*sin(thetayaw)));
f3=subs(z-(l1*sin(thetap1)+l2*sin(thetap2)));

jacobian=([diff(f1,thetap1),diff(f1,thetap2),diff(f1,thetayaw);diff(f2,thetap1),diff(f2,thetap2),diff(f2,thetayaw);diff(f3,thetap1),diff(f3,thetap2),diff(f3,thetayaw)]);%Jacobian

x=1; %X-end effector
y=1; %Y-end effector
z=5; %Z-end effector

thetap1=0.2; %Initial Pose
thetap2=0;
thetayaw=0;
thetabuffer=[thetap1;thetap2;thetayaw];

D=145; %Damping  %85 %92 %145
%v = VideoWriter('nrit.avi');
%open(v);

ix=l1*cos(thetap1)*cos(thetayaw)+l2*cos(thetap2)*cos(thetayaw);
iy=l1*cos(thetap1)*sin(thetayaw)+l2*cos(thetap2)*sin(thetayaw);
iz=l1*sin(thetap1)+l2*sin(thetap2);

for i=0:80  
count=1;

fxyz=[x,y,z];


ixyz=[ix,iy,iz];
error= fxyz-ixyz;

%Main Logic
    thetabuffer=double((subs(thetabuffer)))-((double((subs(transpose(jacobian)))))*((double(subs(jacobian)))*(double(subs(transpose(jacobian))))+D^2*eye(3))*error');

    theta1=thetabuffer(1); %buffer matrix to another variable for convenience
    theta2=thetabuffer(2);
    theta3=thetabuffer(3);
    
    theta11=atan2(sin(theta1),cos(theta1)); %mapping angles -pi to pi, final angle-> might lead to convergence issues
    theta22=atan2(sin(theta2),cos(theta2));
    theta33=atan2(sin(theta3),cos(theta3));
    
    %end-effector position Update
    ix=l1*cos(theta11)*cos(theta33)+l2*cos(theta22)*cos(theta33);
    iy=l1*cos(theta11)*sin(theta33)+l2*cos(theta22)*sin(theta33);
    iz=l1*sin(theta11)+l2*sin(theta22);

    thetabufferupdate=[theta11;theta22;theta33]; %organizing angles in matrix
    disp(thetabufferupdate');
    
    if(norm(error)<0.1)
        break
    end
end