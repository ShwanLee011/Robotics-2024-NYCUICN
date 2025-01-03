clear all;
clc;
close all

Tacc = 0.2;  
T = 0.5;  % A到B和B到C各0.5s
r = (T-Tacc)/T;
Ts = 0.002;
% ABC (為 noap matrix form)
A = [0 1 0 0.4;
     0 0 -1 0.2;
    -1 0 0 -0.3;
     0 0 0 1];
 
B = [0 0 -1 0.4;
     -1 0 0 -0.3;
     0 1 0 0.1;
     0 0 0 1];
 
C = [1 0 0 0.3;
     0 -1 0 0.3;
     0 0 -1 0.2;
     0 0 0 1];
% 計算A到B以及B到C之間的中間姿態以及旋轉矩陣
pose_a = D1(A,B);
pose_a = [pose_a r];
Ra = rotation(pose_a);
% A到B之間的過渡位置Ap
Ap = A*Ra;
pose_ap = D1(B,Ap);
pose_bc = D1(B,C);
delta_B = [pose_ap(1) pose_ap(2) pose_ap(3)];
delta_C = [pose_bc(1) pose_bc(2) pose_bc(3)];
%初始化各個vairable，儲存位置、速度和加速度
p = [];
v = [];
a = [];
%j_set負責儲存所有的joint angle
j_set = [];
tempX = [];
tempZ = [];
Xaxis = [];
Zaxis = [];
%提取各個位置姿態的各項數據，用於路徑規劃
XA = pose_ap(1);
YA = pose_ap(2);
ZA = pose_ap(3);
psiA = pose_ap(4);
thetaA = pose_ap(5);
phiA = pose_ap(6);
XC = pose_bc(1);
YC = pose_bc(2);
ZC = pose_bc(3);
psiC = pose_bc(4);
thetaC = pose_bc(5);
phiC  = pose_bc(6);
%根據時間範圍進行模擬，有三個階段
for t = -T:Ts:T
    if t < -Tacc
        %加速段：從A到B起始過渡
        h = (t+0.5)/T;
        Dr = rotation(pose_a); %旋轉矩陣
        Dv = [pose_a(1)/T;pose_a(2)/T;pose_a(3)/T;0];
        joint = inverse_kinematics(A*Dr); %逆向運動學計算角度
        j_set = [j_set,joint']; %儲存關節角度數據
        %更新位置、速度，加速度為0
        p = [p,A(1:3,:)*Dr(:,4)]; 
        v = [v,A(1:3,:)*Dv];
        a = [a,zeros(3,1)];
        tempX = A*Dr*[0.1 0 0 1]';
        tempZ = A*Dr*[0 0 0.1 1]';
        Xaxis = [Xaxis tempX(1:3,1)];
        Zaxis = [Zaxis tempZ(1:3,1)];
    elseif  Tacc>=t && t>=-Tacc
        % 過渡段：從B到C的過渡
        h = (t+Tacc)/(2*Tacc);
        Dp = [((XC*Tacc/T+XA)*(2-h)*h^2-2*XA)*h + XA;
            ((YC*Tacc/T+YA)*(2-h)*h^2-2*YA)*h + YA;
            ((ZC*Tacc/T+ZA)*(2-h)*h^2-2*ZA)*h + ZA;(psiC-psiA)*h+psiA;
            ((thetaC*Tacc/T+thetaA)*(2-h)*h^2-2*thetaA)*h + thetaA;
            ((phiC*Tacc/T+phiA)*(2-h)*h^2-2*phiA)*h + phiA];
        Dv = [((XC*Tacc/T+XA)*(1.5-h)*2*h^2-XA)/Tacc;
            ((YC*Tacc/T+YA)*(1.5-h)*2*h^2-YA)/Tacc;
            ((ZC*Tacc/T+ZA)*(1.5-h)*2*h^2-ZA)/Tacc;
            0];
        Da = [(XC*Tacc/T+XA)*(1-h)*3*h/Tacc^2;
            (YC*Tacc/T+YA)*(1-h)*3*h/Tacc^2;
            (ZC*Tacc/T+ZA)*(1-h)*3*h/Tacc^2;0];
        Dr = rotation([Dp(1),Dp(2),Dp(3),Dp(4),Dp(5),Dp(6),1]);
        joint = inverse_kinematics(B*Dr);
        j_set = [j_set,joint'];          
        p = [p,B(1:3,:)*Dr(:,4)];
        v = [v,B(1:3,:)*Dv];
        a = [a,B(1:3,:)*Da];        
        tempZ = B*Dr*[0 0 0.1 1]';
        tempX = B*Dr*[0.1 0 0 1]';
        Zaxis = [Zaxis tempZ(1:3,1)];
        Xaxis = [Xaxis tempX(1:3,1)];
    elseif t > Tacc
        % 減速段：完成從B到C的過渡
        h = t/T;
        Dr = rotation([XC,YC,ZC,psiC,thetaC,phiC,h]);
        Dv = [delta_C(1)/T;delta_C(2)/T;delta_C(3)/T;0];
        joint = inverse_kinematics(B*Dr);
        j_set = [j_set,joint'];
        p = [p,B(1:3,:)*Dr(:,4)];
        v = [v,B(1:3,:)*Dv];
        a = [a,zeros(3,1)];        
        tempZ = B*Dr*[0 0 0.1 1]';
        tempX = B*Dr*[0.1 0 0 1]';
        Zaxis = [Zaxis tempZ(1:3,1)];
        Xaxis = [Xaxis tempX(1:3,1)];
    end
end
t = linspace(0,1,1/Ts+1);
labels = {'X', 'Y', 'Z'}; % 軸標籤
quantities = {'Position', 'Velocity', 'Acceleration'}; % 物理量名稱
data = {p, v, a}; % 對應的數據陣列
ylabels = {'Position(m)', 'Velocity(m/s)', 'Acceleration(m/s^2)'}; % Y軸標籤

for i = 1:length(data)
    figure;
    sgtitle(quantities{i}); % 設置整體圖表標題
    for j = 1:3
        subplot(3, 1, j);
        plot(t, data{i}(j, :));
        title(labels{j}); % X, Y, Z 標題
        xlabel('time(s)');
        ylabel(ylabels{i});
    end
end

% 軌跡
figure
plot3(p(1,:),p(2,:),p(3,:), 'LineWidth',1,'color','b');
hold on;
% 座標
Ax = A*[0.1 0 0 1]';plot3([A(1,4) Ax(1)],[A(2,4) Ax(2)],[A(3,4) Ax(3)],'r','LineWidth',1);
Ay = A*[0 0.1 0 1]';plot3([A(1,4) Ay(1)],[A(2,4) Ay(2)],[A(3,4) Ay(3)],'g','LineWidth',1);
Az = A*[0 0 0.1 1]';plot3([A(1,4) Az(1)],[A(2,4) Az(2)],[A(3,4) Az(3)],'b','LineWidth',1);

Bx = B*[0.1 0 0 1]';plot3([B(1,4) Bx(1)],[B(2,4) Bx(2)],[B(3,4) Bx(3)],'r','LineWidth',1);
By = B*[0 0.1 0 1]';plot3([B(1,4) By(1)],[B(2,4) By(2)],[B(3,4) By(3)],'g','LineWidth',1);
Bz = B*[0 0 0.1 1]';plot3([B(1,4) Bz(1)],[B(2,4) Bz(2)],[B(3,4) Bz(3)],'b','LineWidth',1);

Cx = C*[0.1 0 0 1]';plot3([C(1,4) Cx(1)],[C(2,4) Cx(2)],[C(3,4) Cx(3)],'r','LineWidth',1);
Cy = C*[0 0.1 0 1]';plot3([C(1,4) Cy(1)],[C(2,4) Cy(2)],[C(3,4) Cy(3)],'g','LineWidth',1);
Cz = C*[0 0 0.1 1]';plot3([C(1,4) Cz(1)],[C(2,4) Cz(2)],[C(3,4) Cz(3)],'b','LineWidth',1);

% ABC位置
scatter3(A(1,4),A(2,4),A(3,4),[],'k','.');text(A(1,4)+0.02,A(2,4),A(3,4),'PA(0.4 0.2 -0.3)');
scatter3(B(1,4),B(2,4),B(3,4),[],'k','.');text(B(1,4),B(2,4),B(3,4)+0.02,'PB(0.4 -0.3 0.1)');
scatter3(C(1,4),C(2,4),C(3,4),[],'k','.');text(C(1,4),C(2,4),C(3,4)-0.04,'PC(0.3 0.3 0.2)');

% 移動中的XZ軸
for i = 1:length(Xaxis)
    plot3([p(1,i) Xaxis(1,i)],[p(2,i) Xaxis(2,i)],[p(3,i) Xaxis(3,i)],'LineWidth',1,'color','m');
    plot3([p(1,i) Zaxis(1,i)],[p(2,i) Zaxis(2,i)],[p(3,i) Zaxis(3,i)],'LineWidth',1,'color','c');   
end

title('3D path of Cartesian Move');
xlabel('x(m)'),ylabel('y(m)'),zlabel('z(m)');
axis equal;
set(gca,'XGrid','on'),set(gca,'YGrid','on'),set(gca,'ZGrid','on');
hold off;
%把數據寫入txt file內
fileID = fopen('Cartesian_angle.txt', 'w');
fprintf(fileID, '\tjoint1\t\tjoint2\t\tjoint3\t\tjoint4\t\tjoint5\t\tjoint6\t\n');
for i = 1:501
    fprintf(fileID, '%d\t',i);
    fprintf(fileID, '%f\t', (j_set(:,i))');
    fprintf(fileID, '\n');
end
fclose(fileID);

function pose = D1(P1, P2)
    n1 = P1(:, 1);
    o1 = P1(:, 2);
    a1 = P1(:, 3);
    p1 = P1(:, 4);
    n2 = P2(:, 1);
    o2 = P2(:, 2);
    a2 = P2(:, 3);
    p2 = P2(:, 4);
    %使用dot來計算position vector的投影
    x = dot(n1, (p2 - p1)); 
    y = dot(o1, (p2 - p1));
    z = dot(a1, (p2 - p1));
    psi = atan2(dot(o1, a2), dot(n1,a2));
    theta = atan2(nthroot((dot(n1,a2)^2+dot(o1,a2)^2),2), dot(a1,a2));
    phiS = -sin(psi)*cos(psi)*(1-cos(theta))*dot(n1, n2)+...
    (cos(psi)^2*(1-cos(theta))+cos(theta))*dot(o1,n2)-sin(psi)*sin(theta)*dot(a1,n2);
    phiC = -sin(psi)*cos(psi)*(1-cos(theta))*dot(n1, o2)+...
    (cos(psi)^2*(1-cos(theta))+cos(theta))*dot(o1,o2)-sin(psi)*sin(theta)*dot(a1,o2);
    %算完以後都需要經過wrapToPi函式以確保輸出範圍
    phi = atan2(phiS, phiC);
    psi = wrapToPi(psi);
    theta = wrapToPi(theta);
    phi = wrapToPi(phi);
    pose = [x,y,z,psi,theta,phi];

end
function R = rotation(pose)
    x = pose(1);
    y = pose(2);
    z = pose(3);
    psi = pose(4);
    theta = pose(5);
    phi = pose(6);
    r = pose(7);
    R = zeros(4,4);
    R(4,4) = 1;
    V = 1 - cos(r*theta);
    R(1:3,2) = [-sin(r*phi)*(sin(psi)^2*V+cos(r*theta)) + cos(r*phi)*(-sin(psi)*cos(psi)*(1-cos(r*theta)));
                 -sin(r*phi)*(-sin(psi)*cos(psi)*(1-cos(r*theta))) + cos(r*phi)*(cos(psi)^2*(1-cos(r*theta))+cos(r*theta));
                 -sin(r*phi)*(-cos(psi)*sin(r*theta)) + cos(r*phi)*(-sin(psi)*sin(r*theta))];
    R(1:3,3) = [cos(psi)*sin(r*theta);
                sin(psi)*sin(r*theta);
                cos(r*theta)];
    R(1:3,4) = [r*x;
                r*y;
                r*z];
    R(1:3,1) = cross(R(1:3,2),R(1:3,3));
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

    %theta1 ~ theta3 用幾何法計算
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
    % theta4 ~ theta6 用代數法計算
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
function theta = wrapToPi(the)
    if the>pi
        theta = the-2*pi;
    elseif the<-pi
        theta = the+2*pi;
    else
        theta = the;
    end
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
        is_valid = true; % 假設該解有效
        
        % 檢查每個關節角度是否在範圍內
        for j = 1:size(joint_limits, 1)
            if solution(j) < joint_limits(j, 1) || solution(j) > joint_limits(j, 2)
                is_valid = false;
                break; % 跳出內層迴圈，不再檢查其他角度
            end
        end
        
        % 如果解有效，加入有效解矩陣
        if is_valid
            ValidSolutions = [ValidSolutions; solution]; % 僅回傳有效解
        end
    end
end
