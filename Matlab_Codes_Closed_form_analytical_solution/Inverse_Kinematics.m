%% Inverse Kinematics is solved using the Analytical method (geometry) and Product of Exponentials (POE) formula
function [theta] =  Inverse_Kinematics(l1,l2,l3,l4,l5,l6,x,y,z,r,p,yaw, configuration)
%% Description of the robots joint axes 

%{
1. The two shoulder axis intersect orthogonally at a common point, with joint axis 1 aligned in the z0 direction nad joint axis 2 aligned in the y0 direction.
2. Joint axis 3 (elbow joint) lies in the x0-y0 plane and is aligned parallel with joint axis 2
3. Joint axes 4,5 and 6 (the wrist joints) intersect orthogonally at a common point (the wrist center) to form an orthogonal wrist with the joint axes alignes in the z0, y0 and z0 direction.
%}

%% Methodology used to solve the inverse kinemtatics problem

%{
1. The components of the wrist center x are denoted by x = (x(1),x(2),x(3)). Projecting x on the x0-y0 plane the value of theta1 can be calculated.
2. Determining angles theta2 and theta3 for the robotic arm now reduces to the inverse kinematics problem for a planar two-link chain with length l2 and l3+l4.
3. Theta2 and theta3 are obtained by using the cosine rules and adjusting the angles based on the defined frames for the robotic arm.
4. The solution for theta 3 correspond to the well-known elbow-up and elbow-down configurations for the two-link planar arm.
5. The inverse orientation problem is solved for finding (theta4,theta5,theta6) given the end effector (Tool) orientation.
6. Having found (theta1,theta2,theta3) the forward kinematics can be manipulated into the from expm(S4,theta4)*expm(S5,theta5)*expm(S6,theta6) = expm(-S3*theta3)*expm(-S2*theta2)*expm(-S1*theta1)*X*(inverse(M)), where X is the Transforamtion matrix describing the orientation and position of the end effector and M is hte orientation and position of the end-effector/tool at the home configuration.
7. In the above statement the right hand side is now known, and the joint axes for the S4, S5 nad S6 are aligned with z0, y0 and z0 respectively.
8. Denoting the Orientation part of the tranforamtion matrix obtained on the right hand side of the equation by R, the wrist joint angles (theta4, theta5, theta6) can be determined as the solution to Rot(z0,theta4)*Rot(y0,theta5)*Rot(z0,theta6) = R, which corresponds to the ZYZ euler angles.
%}

%% Initializatin of end effector frame and screw axis
M=[1,0,0,l2;
   0,1,0,0;
   0,0,1,l1+l3+l4+l5+l6;
   0,0,0,1];

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

R_x = [1, 0, 0;
       0, cos(r), -sin(r) ;
       0, sin(r), cos(r)] ;

R_y = [cos(p), 0, sin(p);
        0, 1, 0;
       -sin(p), 0,cos(p)];

R_z = [cos(yaw), -sin(yaw), 0;
       sin(yaw), cos(yaw), 0;
       0, 0, 1];

R = R_x*R_y*R_z;

T = [R(1,1),R(1,2),R(1,3),x;
     R(2,1),R(2,2),R(2,3),y;
     R(3,1),R(3,2),R(3,3),z;
     0,0,0,1];

p = [0;
     0;
     -l5-l6;
     1];

x_w = T*p;

theta_1 = atan2(x_w(2),x_w(1)) ;

D = (x_w(1)^2 + x_w(2)^2 + (x_w(3)-l1)^2 - l2^2 - (l3 + l4)^2)/(2*l2*(l3 +l4)); 


theta_3_elbow_down = atan2(sqrt(1-(D^2)),D)
theta_3_elbow_up = atan2(-sqrt(1-(D^2)),D) 


theta_2_elbow_down = atan2(x_w(3)-l1,sqrt(x_w(1)^2 + x_w(2)^2)) - atan2((l3+l4)*sin(theta_3_elbow_down),l2 + (l3+l4)*cos(theta_3_elbow_down)) ;
theta_2_elbow_up = atan2(x_w(3)-l1,sqrt(x_w(1)^2 + x_w(2)^2)) - atan2((l3+l4)*sin(theta_3_elbow_up),l2 + (l3+l4)*cos(theta_3_elbow_up)) ;


if configuration == 0

T3 = expm(-S3*(pi/2 - theta_3_elbow_down));

T2 = expm(-S2*(2*pi -theta_2_elbow_down));
T1 = expm(-S1*theta_1);



T_wrist = T3*T2*T1*T/M;

theta_4 = atan2(T_wrist(2,3),T_wrist(1,3)) ;
theta_5 = atan2(sqrt(T_wrist(3,1)^2 + T_wrist(3,2)^2),T_wrist(3,3)) ;
theta_6 = atan2(T_wrist(3,2),-T_wrist(3,1)) ;



theta = [theta_1;
         2*pi - theta_2_elbow_down;
         pi/2 - theta_3_elbow_down;
         theta_4;
         theta_5;
         theta_6];
theta = theta*180/pi ;
else
T3 = expm(-S3*(theta_3_elbow_up - 3*pi/2));
T2 = expm(-S2*(2*pi - theta_2_elbow_up));
T1 = expm(-S1*theta_1);



T_wrist = T3*T2*T1*T/M;

theta_4 = atan2(T_wrist(2,3),T_wrist(1,3)) ;
theta_5 = atan2(sqrt(T_wrist(3,1)^2 + T_wrist(3,2)^2),T_wrist(3,3)) ;
theta_6 = atan2(T_wrist(3,2),-T_wrist(3,1)) ;


theta = [theta_1;
         2*pi - theta_2_elbow_up;
         theta_3_elbow_up - 3*pi/2;
         theta_4;
         theta_5;
         theta_6];
theta = theta*180/pi ;

end

end
