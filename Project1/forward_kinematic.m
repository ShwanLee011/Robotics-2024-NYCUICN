clear all
clc
% Test joint angles (degrees)
input_T1 = [0.349, 0.349, -0.349, 0.349, 0.349, 0.349]; % rad
input_T2 = [0.873, 0.873, -0.873, 0.873, 0.873, 0.873]; % rad

% call the forward kinematic function
[result_T1, pose_T1] = forward_kinematics(input_T1);
[result_T2, pose_T2] = forward_kinematics(input_T2);

% Show the Result
fprintf('Result for Input_T1:\n');
disp(result_T1);
fprintf('Pose for Input_T1 (x, y, z, φ, θ, ψ):\n');
disp(pose_T1);

fprintf('Result for Input_T2:\n');
disp(result_T2);
fprintf('Pose for Input_T2 (x, y, z, φ, θ, ψ):\n');
disp(pose_T2);

function [T, pose] = forward_kinematics(joint_angles)
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
    joint_limits_rad = deg2rad(joint_limits_deg); % Convert to radians

    % Check if the number of joint angles is correct
    if length(joint_angles) ~= 6
        error('6 joint angles are required.');
    end

    % Check if each joint angle is within range
    for i = 1:6
        if joint_angles(i) < joint_limits_rad(i, 1) || joint_angles(i) > joint_limits_rad(i, 2)
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
            0, sin(alpha), cos(alpha), d;
            0, 0, 0, 1
        ];

        % Accumulate the product
        T = T * A_i;
    end

    % Extract position (x, y, z)
    x = T(1, 4);
    y = T(2, 4);
    z = T(3, 4);

    % Extract rotation matrix
    R = T(1:3, 1:3);
    % Calculate Euler angles (φ, θ, ψ)
    
    
    phi = atan2(R(2, 3), R(1, 3)); % Roll
    theta = atan2((cos(phi)*T(1, 3)+sin(phi)*T(2, 3)) , T(3, 3)); % Pitch
    psi = atan2((-sin(phi)*T(1, 1)+cos(phi)*T(2,1)),(-sin(phi)*T(1, 2)+cos(phi)*T(2,2))); % Yaw

    % Final pose
    pose = [x, y, z, phi*180/pi, theta*180/pi, psi*180/pi];

end
