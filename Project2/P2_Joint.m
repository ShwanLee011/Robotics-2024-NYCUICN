clear all; 
clc;
close all;
Tacc = 0.2; %tacc=0.2 要求
T = 0.5; %A-->B, B-->C各0.5s
r = (T-Tacc)/T;
Ts = 0.002; %Sampling time(discretlize the system)
%定義機械手臂的出發點、經過點以及目標點(ABC)，以及相對應的n,o,a,p矩陣
A = [0 1  0  0.4;
     0 0 -1  0.2;
    -1 0  0 -0.3;
     0 0  0  1];
B = [ 0 0 -1 0.4;
     -1 0 0 -0.3;
      0 1 0  0.1;
      0 0 0  1];
 
C = [1  0  0 0.3;
     0 -1  0 0.3;
     0  0 -1 0.2;
     0  0  0 1];
% 計算ABC的joint angle應該為何(各只有一組符合範圍內的解)
theta_A = inverse_kinematics(A);
theta_B = inverse_kinematics(B);
theta_C = inverse_kinematics(C);

theta_p = [];
theta_v = [];
theta_a = [];
%根據時間範圍進行模擬，有三個階段，此處寫成function的形式來計算，只有position, velocity和accleration vector
for t = -T:Ts:T
    [theta_p_t, theta_v_t, theta_a_t] = compute_theta(t, T, Tacc, theta_A, theta_B, theta_C, r);
    theta_p = [theta_p theta_p_t'];
    theta_v = [theta_v theta_v_t'];
    theta_a = [theta_a theta_a_t'];
end

t = linspace(0, 1, 1/Ts + 1);
p = [];
zAxis = [];
for i = 1:(1/Ts+1)
    T = forward_kinematics((theta_p(:,i))');
    p = [p T(1:3, 4)];
    temp = T*[0 0 0.1 1]';
    zAxis = [zAxis temp(1:3)];
end
titles = {'Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6'};
% joint1-6 角加速度
figure
title('Angular accleration')
for i = 1:6
    subplot(3, 2, i);
    plot(t, theta_a(i, :)); % 或 theta_v、theta_p
    title(titles{i});
    xlabel('time(s)');
    ylabel('deg/s^2');
end
% joint1-6 角速度
figure
title("Angular velocity")
for i = 1:6
    subplot(3, 2, i);
    plot(t, theta_v(i, :)); % 或 theta_v、theta_p
    title(titles{i});
    xlabel('time(s)');
    ylabel('deg/s');
end
% joint1-6 角度
figure
title("Angle")
for i = 1:6
    subplot(3, 2, i);
    plot(t, theta_p(i, :)); % 或 theta_v、theta_p
    title(titles{i});
    xlabel('time(s)');
    ylabel('deg');
end
% 軌跡
figure
plot3(p(1,:),p(2,:),p(3,:), 'LineWidth',1,'color','b');
hold on;

% 座標
Ax = A*[0.1 0 0 1]';
plot3([A(1,4) Ax(1)],[A(2,4) Ax(2)],[A(3,4) Ax(3)],'r','LineWidth',1);
Ay = A*[0 0.1 0 1]';
plot3([A(1,4) Ay(1)],[A(2,4) Ay(2)],[A(3,4) Ay(3)],'g','LineWidth',1);
Az = A*[0 0 0.1 1]';
plot3([A(1,4) Az(1)],[A(2,4) Az(2)],[A(3,4) Az(3)],'b','LineWidth',1);

Bx = B*[0.1 0 0 1]';
plot3([B(1,4) Bx(1)],[B(2,4) Bx(2)],[B(3,4) Bx(3)],'r','LineWidth',1);
By = B*[0 0.1 0 1]';
plot3([B(1,4) By(1)],[B(2,4) By(2)],[B(3,4) By(3)],'g','LineWidth',1);
Bz = B*[0 0 0.1 1]';
plot3([B(1,4) Bz(1)],[B(2,4) Bz(2)],[B(3,4) Bz(3)],'b','LineWidth',1);

Cx = C*[0.1 0 0 1]';
plot3([C(1,4) Cx(1)],[C(2,4) Cx(2)],[C(3,4) Cx(3)],'r','LineWidth',1);
Cy = C*[0 0.1 0 1]';
plot3([C(1,4) Cy(1)],[C(2,4) Cy(2)],[C(3,4) Cy(3)],'g','LineWidth',1);
Cz = C*[0 0 0.1 1]';
plot3([C(1,4) Cz(1)],[C(2,4) Cz(2)],[C(3,4) Cz(3)],'b','LineWidth',1);

% ABC位置
scatter3(A(1,4),A(2,4),A(3,4),[],'k','.');text(A(1,4)+0.02,A(2,4),A(3,4),'PA(0.4 -0.3 0.1)');
scatter3(B(1,4),B(2,4),B(3,4),[],'k','.');text(B(1,4),B(2,4),B(3,4)+0.02,'PB(0.3 0.3 0.2)');
scatter3(C(1,4),C(2,4),C(3,4),[],'k','.');text(C(1,4),C(2,4),C(3,4)-0.04,'PC(0.4 0.2 -0.3)');
for i = 1:length(zAxis)
    plot3([p(1,i) zAxis(1,i)], [p(2,i) zAxis(2,i)], [p(3,i) zAxis(3,i)],'color', 'c');
end

title('3D path of Joint Move');
xlabel('x(m)'), ylabel('y(m)'), zlabel('z(m)');
axis equal;
set(gca,'XGrid','on');
set(gca,'YGrid','on');
set(gca,'ZGrid','on');
hold off;
%寫入txt file
fileID = fopen('Joint_angle.txt', 'w');
fprintf(fileID, '\tjoint1\t\tjoint2\t\tjoint3\t\tjoint4\t\tjoint5\t\tjoint6\t\n');
for i = 1:(1/Ts+1)
    fprintf(fileID, '%d\t',i);
    fprintf(fileID, '%f\t', (theta_p(:,i))');
    fprintf(fileID, '\n');
end
fclose(fileID);

function [theta_p, theta_v, theta_a] = compute_theta(t, T, Tacc, theta_A, theta_B, theta_C, r)
    if t < -Tacc
        theta_p = (theta_B - theta_A) * (t + 0.5) / T + theta_A;
        theta_v = (theta_B - theta_A) / T;
        theta_a = zeros(size(theta_A));
    elseif t <= Tacc && t >= -Tacc
        k = (t + Tacc) / (2 * Tacc);
        common_term = (theta_C - theta_B) * Tacc / T + ((theta_A + (theta_B - theta_A) * r) - theta_B);
        theta_p = (common_term * (2 - k) * k^2 - 2 * ((theta_A + (theta_B - theta_A) * r) - theta_B)) * k ...
                  + ((theta_A + (theta_B - theta_A) * r) - theta_B) + theta_B;
        theta_v = (common_term * (1.5 - k) * 2 * k^2 - ((theta_A + (theta_B - theta_A) * r) - theta_B)) / Tacc;
        theta_a = (common_term * (1 - k) * 3 * k / Tacc^2);
    else
        theta_p = (theta_C - theta_B) * t / T + theta_B;
        theta_v = (theta_C - theta_B) / T;
        theta_a = zeros(size(theta_A));
    end
end

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
    px = Cartesian_point(1,4);
    py = Cartesian_point(2,4);
    pz = Cartesian_point(3,4);
    a1 = 0.12;
    a2 = 0.25;
    a3 = 0.26;
    phi = atan2(ay,ax);
    thetaa = atan2((cos(phi)*ax + sin(phi)*ay),az);
    psi = atan2((-sin(phi)*nx + cos(phi)*ny),(-sin(phi)*ox + cos(phi)*oy));
    if(psi < 0)
        psi = psi +pi;
    end
    %Solve theta1
    theta1_1 = atan2(py,px);
    theta1_2 = atan2(py,px)-pi;
    theta1_1 = wrapToPi(theta1_1);
    theta1_2 = wrapToPi(theta1_2);
    %Solve theta2
    gama1 = atan2(-pz,(cos(theta1_1)*px+sin(theta1_1)*py)-a1);
    gama2 = atan2(-pz,(cos(theta1_2)*px+sin(theta1_2)*py)-a1);
    R1 = ((((cos(theta1_1)*px+sin(theta1_1)*py )^(2)+(sin(theta1_1)*px-cos(theta1_1)*py)^(2))^(0.5)-a1)^2 + pz^2)^(0.5);
    R2 = ((((cos(theta1_1)*px+sin(theta1_1)*py )^(2)+(sin(theta1_1)*px-cos(theta1_1)*py)^(2))^(0.5)-a1)^2 + pz^2)^(0.5);
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
    theta = rad2deg(theta);
    Solutions = check_joint_limits(theta);
end
function ValidSolutions = check_joint_limits(theta)
    joint_limits = [
        -150, 150;  % θ1
        -30, 100;   % θ2
        -120, 0;    % θ3
        -110, 110;  % θ4
        -180, 180;  % θ5
        -180, 180   % θ6
    ];
    % 初始化有效解矩陣
    ValidSolutions = [];
    
    for i = 1:size(theta, 1)
        solution = theta(i, :);
        is_valid = true; 
        % 檢查每個關節角度是否在範圍內
        for j = 1:size(joint_limits, 1)
            if solution(j) < joint_limits(j, 1) || solution(j) > joint_limits(j, 2)
                %fprintf("Row %d: θ%d out of range! (%.2f)\n", i, j, solution(j));
                is_valid = false;
                break; % 跳出內層迴圈，不再檢查其他角度
            end
        end
        % 如果解有效，加入有效解矩陣
        if is_valid
            ValidSolutions = [ValidSolutions; solution]; % 添加有效的解
        end
    end
end
function T = forward_kinematics(joint_angles)
    % Mitsubishi RV-M2 six-axis robotic arm DH parameters
    % joint_angles: six joint angles (rad)
    % Output:
    % T: Homogeneous transformation matrix for the entire manipulator
    % pose: End-effector position and orientation (x, y, z, φ, θ, ψ)

    % Joint limits (rad)
    joint_limits_deg = [
        -150, 150;
        -30, 100;
        -120, 0;
        -110, 110;
        -180, 180;
        -180, 180
    ];
    

    % Check if the number of joint angles is correct
    if length(joint_angles) ~= 6
        error('6 joint angles are required.');
    end

    % Check if each joint angle is within range
    for i = 1:6
        if joint_angles(i) < joint_limits_deg(i, 1) || joint_angles(i) > joint_limits_deg(i, 2)
            error('Joint %d is out of range: angle %.2f degrees (allowed range: %.2f ~ %.2f degrees).', ...
                i, rad2deg(joint_angles(i)), joint_limits_deg(i, 1), joint_limits_deg(i, 2));
        end
    end

    % DH parameter table [theta d a alpha]
    dh_table = [
        joint_angles(1) 0 120 -pi/2;
        joint_angles(2) 0 250 0;
        joint_angles(3) 0 260 0;
        joint_angles(4) 0 0 -pi/2;
        joint_angles(5) 0 0 pi/2;
        joint_angles(6) 0 0 0
    ];

    % Initialize the homogeneous transformation matrix
    T = eye(4); % 4x4 identity matrix
    for i = 1:size(dh_table, 1)
        theta = dh_table(i, 1);
        d = dh_table(i, 2);
        a = dh_table(i, 3);
        alpha = dh_table(i, 4);

        % Compute the transformation matrix for each joint
        A_i = [
            cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta);
            sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
            0,          sin(alpha),             cos(alpha),            d;
            0,          0,                      0,                     1
        ];

        % Accumulate the product
        T = T * A_i;
    end

end