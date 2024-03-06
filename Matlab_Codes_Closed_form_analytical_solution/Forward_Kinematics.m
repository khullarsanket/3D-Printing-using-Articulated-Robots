%% The forward Kinematics for the 6R robotic arm is solved using the Product of exponentials (POE) approach.
%% ForwardsKinematics are used to output the end effectors location.
function [G]= Forward_Kinematics(l1,l2,l3,l4,l5,l6,theta1,theta2,theta3,theta4,theta5,theta6)

%Initialization
% M is the postion and orientation of the end effector frame wehn all joint
% angles are set ot zero (the "home" or "zero" position of the robot).
M=[1,0,0,l2;
    0,1,0,0;
    0,0,1,l1+l3+l4+l5+l6;
    0,0,0,1];

% Each revolute joint axis is considered to be a zero-pitch screw axis. If
% theta 1 and theta 2 are held at their zero position then the screw axis
% corresponding to rotating about joint 1 can be expressed in the {0} frame
% as S1 = [w1;
%          v1];
% w = angular velocity of the joint axis (1 rad/s)
% v = linear velocity of the point at the origin of {0}
% Thus the forward kinematics can be expressed as a product of matrix
% exponentials, each corresponding to a screwq motion.
S1=[0,-1,0,0;
    1,0,0,0;
    0,0,0,0;
    0,0,0,0];

S2=[0,0,1,-l1;
    0,0,0,0;
    -1,0,0,0;
    0,0,0,0];

S3=[0,0,1,-l1;
    0,0,0,0;
    -1,0,0,l2;
    0,0,0,0];

S4=[0,-1,0,0;
    1,0,0,-l2;
    0,0,0,0;
    0,0,0,0];

S5=[0,0,1,-(l1+l3+l4);
    0,0,0,0;
    -1,0,0,l2;
    0,0,0,0];

S6=[0,-1,0,0;
    1,0,0,-l2;
    0,0,0,0;
    0,0,0,0];

T=expm(S1*theta1)*expm(S2*theta2)*expm(S3*theta3)*expm(S4*theta4)*expm(S5*theta5)*expm(S6*theta6)*M;

G=[T(1,4);T(2,4);T(3,4)]; % The position vector giving the end effector position corresponding to the given joint angles.
end