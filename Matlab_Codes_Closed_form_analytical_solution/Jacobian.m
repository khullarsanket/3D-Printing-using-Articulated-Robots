function [J] = Jacobian(l1,l2,l3,l4,l5,l6,theta1,theta2,theta3,theta4,theta5,~)
        
w_s1 = [0;
        0;
        1];

v1 = [0;
      0;
      0];

q_0 = [0;
       0;
       l1];  
      
w_s2_0 = [0;
          1;
          0];       

w_s2 = [sin(theta1);
        -cos(theta1);
         0];


q2_0 = [0;
        0;
        l1];

v2 = -cross(w_s2,q2_0) ;
 
w_s3_0 = [0;
          1;
          0];

w_s3 = rotation_z(theta1)*rotation_y(theta2)*w_s3_0;



q3_0 = [l2;
        0;
        0];

q3 = q_0 + rotation_z(theta1)*rotation_y(theta2)*q3_0 ;

v3 = -cross(w_s3,q3) ;

w_s4_0 = [0;
          0;
          1];

w_s4 = rotation_z(theta1)*rotation_y(theta2)*rotation_y(theta3)*w_s4_0 ;


q4_0 = [0;
        0;
        l3+l4];

q4 = q3 + rotation_z(theta1)*rotation_y(theta2)*rotation_y(theta3)*q4_0 ;

v4 = -cross(w_s4,q4);

w_s5_0 = [0;
          1;
          0];       


w_s5 = rotation_z(theta1)*rotation_y(theta2)*rotation_y(theta3)*rotation_z(theta4)*w_s5_0 ;   

v5 = -cross(w_s5,q4) ; %q4=q5

w_s6_0 = [0;
          0;
          1];                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   

w_s6 = rotation_z(theta1)*rotation_y(theta2)*rotation_y(theta3)*rotation_z(theta4)*rotation_y(theta5)*w_s6_0 ;  

q6_0 = [0;
        0;
        l5+l6];

q6 = q4 + rotation_z(theta1)*rotation_y(theta2)*rotation_y(theta3)*rotation_z(theta4)*rotation_y(theta5)*q6_0 ;  

v6 = -cross(w_s6,q6) ; 

J = [w_s1(1),w_s2(1),w_s3(1),w_s4(1),w_s5(1),w_s6(1);
     w_s1(2),w_s2(2),w_s3(2),w_s4(2),w_s5(2),w_s6(2);
     w_s1(3),w_s2(3),w_s3(3),w_s4(3),w_s5(3),w_s6(3);
     v1(1),v2(1),v3(1),v4(1),v5(1),v6(1);
     v1(2),v2(2),v3(2),v4(2),v5(2),v6(2);
     v1(3),v2(3),v3(3),v4(3),v5(3),v6(3)];
            
end





function [R_x] =  rotation_x(theta)

R_x = [1,0,0;
       0,cos(theta),-sin(theta);
       0,sin(theta),cos(theta)];

end

function [R_y] =  rotation_y(theta)

R_y = [cos(theta),0,sin(theta);
       0,1,0;
       -sin(theta),0,cos(theta)];

end

function [R_z] =  rotation_z(theta)

R_z = [cos(theta),-sin(theta),0;
       sin(theta),cos(theta),0;
       0,0,1];

end