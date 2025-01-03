clear all
clc
A1 = [0.5756 -0.2398 -0.7817 177.8;
       0.7738 -0.1494  0.6156 308;
      -0.2644 -0.9593  0.0996 -140.1;
            0       0       0   1   ];
A2 = [0.1736, -0.0000,  -0.9848,   0.0000;
      0.8529,  0.5000,   0.1504,   325.2;
      0.4924, -0.8660,   0.0868,  -158;
           0,       0,       0,       1 ];
S1 = inverse_kinematics(A1);
disp("Result of A1:")
disp(S1);
check_joint_limits(S1);
S2 = inverse_kinematics(A2);
disp("Result of A2:");
disp(S2);
check_joint_limits(S2);
function [Solutions] = inverse_kinematics(Cartesian_point)
    nx = Cartesian_point(1,1);
    ny = Cartesian_point(2,1);
    nz = Cartesian_point(3,1);
    ox = Cartesian_point(1,2);
    oy = Cartesian_point(2,2);
    oz = Cartesian_point(3,2);
    ax = Cartesian_point(1,3);
    ay = Cartesian_point(2,3);
    az = Cartesian_point(3,3);
    px = Cartesian_point(1,4)*0.001;
    py = Cartesian_point(2,4)*0.001;
    pz = Cartesian_point(3,4)*0.001;
    a1 = 0.12;
    a2 = 0.25;
    a3 = 0.26;
    phi = atan2(ay,ax);
    thetaa = atan2((cos(phi)*ax + sin(phi)*ay),az);
    psi = atan2((-sin(phi)*nx + cos(phi)*ny),(-sin(phi)*ox + cos(phi)*oy));
    if(psi < 0)
        psi = psi +pi;
    end
    disp('(x,y,z,ψ,θ,φ) =')
    disp([px py pz phi thetaa psi])
    %Solve theta1
    theta1_1 = atan2(py,px);
    theta1_2 = atan2(py,px)-pi;
    theta1_1 = wrapToPi(theta1_1);
    theta1_2 = wrapToPi(theta1_2);
    %Solve theta2
    gama1 = atan2(-pz,(cos(theta1_1)*px+sin(theta1_1)*py)-a1);
    gama2 = atan2(-pz,(cos(theta1_2)*px+sin(theta1_2)*py)-a1);
    R1 = (((cos(theta1_1)*px+sin(theta1_1)*py)-a1)^2 + pz^2)^(0.5);
    R2 = (((cos(theta1_2)*px+sin(theta1_2)*py)-a1)^2 + pz^2)^(0.5);
    aphar1 = acos((R1^2+a2^2-a3^2)/(2*R1*a2));
    aphar2 = acos((R2^2+a2^2-a3^2)/(2*R2*a2));
    theta2_1 = (gama1 - aphar1);
    theta2_2 = (gama1 + aphar1);
    theta2_3 = (gama2 - aphar2);
    theta2_4 = (gama2 + aphar2);
    theta2_1 = wrapToPi(theta2_1);
    theta2_2 = wrapToPi(theta2_2);
    theta2_3 = wrapToPi(theta2_3);
    theta2_4 = wrapToPi(theta2_4);
    %Solve theta3
    beta1 = acos((R1^2+a3^2-a2^2)/(2*R1*a3));
    beta2 = acos((R2^2+a3^2-a2^2)/(2*R2*a3));
    theta3_1 = aphar1 + beta1;
    theta3_2 = -aphar1 - beta1;
    theta3_3 = aphar2 + beta2;
    theta3_4 = -aphar2 - beta2;
    theta3_1 = wrapToPi(theta3_1);
    theta3_2 = wrapToPi(theta3_2);
    theta3_3 = wrapToPi(theta3_3);
    theta3_4 = wrapToPi(theta3_4);

    tan_theta4_1 = (-cos(theta1_1)*sin(theta2_1+theta3_1)*ax - ...
    sin(theta1_1)*sin(theta2_1+theta3_1)*ay - cos(theta2_1+theta3_1)*az) /...
    (cos(theta1_1)*cos(theta2_1+theta3_1)*ax + sin(theta1_1)*...
    cos(theta2_1+theta3_1)*ay - sin(theta2_1+theta3_1)*az);
    theta4_1_1 = atan(tan_theta4_1);
    theta4_1_1 = wrapToPi(theta4_1_1);
    theta4_1_2 = theta4_1_1 - pi;
    theta4_1_2 = wrapToPi(theta4_1_2);
    
    tan_theta4_2 = (-cos(theta1_1)*sin(theta2_2+theta3_2)*ax ...
    - sin(theta1_1)*sin(theta2_2+theta3_2)*ay - cos(theta2_2+theta3_2)*az)...
    / (cos(theta1_1)*cos(theta2_2+theta3_2)*ax + sin(theta1_1)*cos(theta2_2...
    +theta3_2)*ay - sin(theta2_2+theta3_2)*az);
    theta4_2_1 = atan(tan_theta4_2);
    theta4_2_1 = wrapToPi(theta4_2_1);
    theta4_2_2 = theta4_2_1 - pi;
    theta4_2_2 = wrapToPi(theta4_2_2);
    
    tan_theta4_3 = (-cos(theta1_2)*sin(theta2_3+theta3_3)*ax - sin(theta1_2)...
    *sin(theta2_3+theta3_3)*ay - cos(theta2_3+theta3_3)*az) / (cos(theta1_2)...
    *cos(theta2_3+theta3_3)*ax + sin(theta1_2)*cos(theta2_3+theta3_3)*ay - ...
    sin(theta2_3+theta3_3)*az);
    theta4_3_1 = atan(tan_theta4_3);
    theta4_3_1 = wrapToPi(theta4_3_1);
    theta4_3_2 = theta4_3_1 - pi;
    theta4_3_2 = wrapToPi(theta4_3_2);

    tan_theta4_4 = (-cos(theta1_2)*sin(theta2_4+theta3_4)*ax - sin(theta1_2)...
    *sin(theta2_4+theta3_4)*ay - cos(theta2_4+theta3_4)*az) / (cos(theta1_2)*...
    cos(theta2_4+theta3_4)*ax + sin(theta1_2)*cos(theta2_4+theta3_4)*ay ...
    - sin(theta2_4+theta3_4)*az);
    theta4_4_1 = atan(tan_theta4_4);
    theta4_4_1 = wrapToPi(theta4_4_1);
    theta4_4_2 = theta4_4_1 - pi;
    theta4_4_2 = wrapToPi(theta4_4_2);
    
    theta5_1 = atan2(ax*cos(theta1_1)*cos(theta2_1+theta3_1+theta4_1_1) +...
    ay*sin(theta1_1)*cos(theta2_1+theta3_1+theta4_1_1) - ...
    az*sin(theta2_1+theta3_1+theta4_1_1) , -ax*sin(theta1_1)+ay*cos(theta1_1));
    theta5_2 = atan2(ax*cos(theta1_1)*cos(theta2_1+theta3_1+theta4_1_2) +...
    ay*sin(theta1_1)*cos(theta2_1+theta3_1+theta4_1_2) - ...
    az*sin(theta2_1+theta3_1+theta4_1_2) , -ax*sin(theta1_1)+ay*cos(theta1_1));
    theta5_3 = atan2(ax*cos(theta1_1)*cos(theta2_2+theta3_2+theta4_2_1) +...
    ay*sin(theta1_1)*cos(theta2_2+theta3_2+theta4_2_1) - ...
    az*sin(theta2_2+theta3_2+theta4_2_1) , -ax*sin(theta1_1)+ay*cos(theta1_1));
    theta5_4 = atan2(ax*cos(theta1_1)*cos(theta2_2+theta3_2+theta4_2_2) + ...
    ay*sin(theta1_1)*cos(theta2_2+theta3_2+theta4_2_2) - ...
    az*sin(theta2_2+theta3_2+theta4_2_2) , -ax*sin(theta1_1)+ay*cos(theta1_1));
    theta5_5 = atan2(ax*cos(theta1_2)*cos(theta2_3+theta3_3+theta4_3_1) + ...
    ay*sin(theta1_2)*cos(theta2_3+theta3_3+theta4_3_1) - ...
    az*sin(theta2_3+theta3_3+theta4_3_1) , -ax*sin(theta1_2)+ay*cos(theta1_2));
    theta5_6 = atan2(ax*cos(theta1_2)*cos(theta2_3+theta3_3+theta4_3_2) + ...
    ay*sin(theta1_2)*cos(theta2_3+theta3_3+theta4_3_2) - ...
    az*sin(theta2_3+theta3_3+theta4_3_2) , -ax*sin(theta1_2)+ay*cos(theta1_2));
    theta5_7 = atan2(ax*cos(theta1_2)*cos(theta2_4+theta3_4+theta4_4_1) + ...
    ay*sin(theta1_2)*cos(theta2_4+theta3_4+theta4_4_1) - ...
    az*sin(theta2_4+theta3_4+theta4_4_1) , -ax*sin(theta1_2)+ay*cos(theta1_2));
    theta5_8 = atan2(ax*cos(theta1_2)*cos(theta2_4+theta3_4+theta4_4_2) + ...
    ay*sin(theta1_2)*cos(theta2_4+theta3_4+theta4_4_2) - ...
    az*sin(theta2_4+theta3_4+theta4_4_2) , -ax*sin(theta1_2)+ay*cos(theta1_2));
    
    theta6_1 = atan2(-nx*cos(theta1_1)*sin(theta2_1+theta3_1+theta4_1_1) - ...
    ny*sin(theta1_1)*sin(theta2_1+theta3_1+theta4_1_1) - nz*cos(theta2_1+theta3_1+theta4_1_1) , ...
    -ox*cos(theta1_1)*sin(theta2_1+theta3_1+theta4_1_1) ...
    -oy*sin(theta1_1)*sin(theta2_1+theta3_1+theta4_1_1)-oz*cos(theta2_1+theta3_1+theta4_1_1));
    theta6_2 = atan2(-nx*cos(theta1_1)*sin(theta2_1+theta3_1+theta4_1_2) - ...
    ny*sin(theta1_1)*sin(theta2_1+theta3_1+theta4_1_2)-nz*cos(theta2_1+theta3_1+theta4_1_2) , ...
    -ox*cos(theta1_1)*sin(theta2_1+theta3_1+theta4_1_2) ...
    -oy*sin(theta1_1)*sin(theta2_1+theta3_1+theta4_1_2)-oz*cos(theta2_1+theta3_1+theta4_1_2));
    theta6_3 = atan2(-nx*cos(theta1_1)*sin(theta2_2+theta3_2+theta4_2_1) - ...
    ny*sin(theta1_1)*sin(theta2_2+theta3_2+theta4_2_1)-nz*cos(theta2_2+theta3_2+theta4_2_1) , ...
    -ox*cos(theta1_1)*sin(theta2_2+theta3_2+theta4_2_1)- ...
    oy*sin(theta1_1)*sin(theta2_2+theta3_2+theta4_2_1)-oz*cos(theta2_2+theta3_2+theta4_2_1));
    theta6_4 = atan2(-nx*cos(theta1_1)*sin(theta2_2+theta3_2+theta4_2_2) ...
    -ny*sin(theta1_1)*sin(theta2_2+theta3_2+theta4_2_2)-nz*cos(theta2_2+theta3_2+theta4_2_2) , ...
    -ox*cos(theta1_1)*sin(theta2_2+theta3_2+theta4_2_2)- ...
    oy*sin(theta1_1)*sin(theta2_2+theta3_2+theta4_2_2)-oz*cos(theta2_2+theta3_2+theta4_2_2));
    theta6_5 = atan2(-nx*cos(theta1_2)*sin(theta2_3+theta3_3+theta4_3_1)- ...
    ny*sin(theta1_2)*sin(theta2_3+theta3_3+theta4_3_1)-nz*cos(theta2_3+theta3_3+theta4_3_1) , ...
    -ox*cos(theta1_2)*sin(theta2_3+theta3_3+theta4_3_1)- ...
    oy*sin(theta1_2)*sin(theta2_3+theta3_3+theta4_3_1)-oz*cos(theta2_3+theta3_3+theta4_3_1));
    theta6_6 = atan2(-nx*cos(theta1_2)*sin(theta2_3+theta3_3+theta4_3_2)- ...
    ny*sin(theta1_2)*sin(theta2_3+theta3_3+theta4_3_2)-nz*cos(theta2_3+theta3_3+theta4_3_2) , ...
    -ox*cos(theta1_2)*sin(theta2_3+theta3_3+theta4_3_2)- ...
    oy*sin(theta1_2)*sin(theta2_3+theta3_3+theta4_3_2)-oz*cos(theta2_3+theta3_3+theta4_3_2));
    theta6_7 = atan2(-nx*cos(theta1_2)*sin(theta2_4+theta3_4+theta4_4_1)- ...
    ny*sin(theta1_2)*sin(theta2_4+theta3_4+theta4_4_1)-nz*cos(theta2_4+theta3_4+theta4_4_1) , ...
    -ox*cos(theta1_2)*sin(theta2_4+theta3_4+theta4_4_1)- ...
    oy*sin(theta1_2)*sin(theta2_4+theta3_4+theta4_4_1)-oz*cos(theta2_4+theta3_4+theta4_4_1));
    theta6_8 = atan2(-nx*cos(theta1_2)*sin(theta2_4+theta3_4+theta4_4_2)- ...
    ny*sin(theta1_2)*sin(theta2_4+theta3_4+theta4_4_2)-nz*cos(theta2_4+theta3_4+theta4_4_2) , ...
    -ox*cos(theta1_2)*sin(theta2_4+theta3_4+theta4_4_2)- ...
    oy*sin(theta1_2)*sin(theta2_4+theta3_4+theta4_4_2)-oz*cos(theta2_4+theta3_4+theta4_4_2));
    
    theta = [theta1_1 theta2_1 theta3_1 theta4_1_1 theta5_1 theta6_2;
             theta1_1 theta2_1 theta3_1 theta4_1_2 theta5_2 theta6_1;
             theta1_1 theta2_2 theta3_2 theta4_2_1 theta5_3 theta6_3;
             theta1_1 theta2_2 theta3_2 theta4_2_2 theta5_4 theta6_4;
             theta1_2 theta2_3 theta3_3 theta4_3_1 theta5_5 theta6_5;
             theta1_2 theta2_3 theta3_3 theta4_3_2 theta5_6 theta6_6;
             theta1_2 theta2_4 theta3_4 theta4_4_1 theta5_7 theta6_7;
             theta1_2 theta2_4 theta3_4 theta4_4_2 theta5_8 theta6_8;];
    Solutions = rad2deg(theta);
end
function theta = wrapToPi(the)
    if the>pi
        theta = the-2*pi;
    elseif the<-pi
        theta = the+2*pi;
    else
        theta = the;
    end
end

function check_joint_limits(Solutions)
    joint_limits = [
        -150, 150;  % θ1
        -30, 100;   % θ2
        -120, 0;    % θ3
        -110, 110;  % θ4
        -180, 180;  % θ5
        -180, 180   % θ6
    ];
    
    fprintf("Checking joint limits:\n");
    for i = 1:size(Solutions, 1)
        fprintf("Solution %d: \n", i);
        solution = Solutions(i, :);
        valid = true;
        for j = 1:size(joint_limits, 1)
            if solution(j) < joint_limits(j, 1) || solution(j) > joint_limits(j, 2)
                fprintf("θ%d out of range! (%.2f)\n", j, solution(j));
                valid = false;
            end
        end
        if valid
            fprintf("All joint angles within range.\n");
        end
    end
end