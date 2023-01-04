clc;
clear;
DtoR = pi/180;
RtoD = 180/pi;
%Parameter definition
theta = [ 0, 0, 0, 0, 0, 0 ]; %save theta(12456)d(3)
constraint_up = [ 160, 125, 30, 140, 100, 260 ];
constraint_down = [ -160, -125, -30, -140, -100, -260 ];
fprintf('Forward kinematics\n');
%input joint value
NUMBER = 1;
while( NUMBER<=6 )
    if NUMBER ~= 3
        fprintf('Enter the joint variables theta %d \n',NUMBER);
        joint=input(''); 
        if( constraint_down(NUMBER)<=joint&&joint<=constraint_up(NUMBER) )
            theta(NUMBER) = joint;
            NUMBER = NUMBER+1;
        else
            fprintf('theta%d is out of range\nPlease input again \n',NUMBER);
        end
    else
        fprintf('Enter the joint variables d%d (in) : \n',NUMBER);
        len_d=input('');
        if( constraint_down(NUMBER)<=joint&&joint<=constraint_up(NUMBER) )
            theta(NUMBER) = len_d;
            NUMBER = NUMBER+1;
        else
            fprintf('d%d is out of range\nPlease input again: \n',NUMBER);
        end
    end 
end
%Forward kinematics

A1 = trnsformation(0, 0, -90*DtoR, theta(1)*DtoR);
A2 = trnsformation(6.375, 0, 90*DtoR, theta(2)*DtoR);
A3 = trnsformation(theta(3), 0, 0, 0);
A4 = trnsformation(0, 0, -90*DtoR, theta(4)*DtoR);
A5 = trnsformation(0, 0, 90*DtoR, theta(5)*DtoR);
A6 = trnsformation(0, 0, 0, theta(6)*DtoR);

T6=A1*A2*A3*A4*A5*A6;          
nx = T6(1,1); ny = T6(2,1); nz = T6(3,1);
ox = T6(1,2); oy = T6(2,2); oz = T6(3,2);
ax = T6(1,3); ay = T6(2,3); az = T6(3,3);
px = T6(1,4); py = T6(2,4); pz = T6(3,4);
x = px; y = py; z = pz;
phi = atan2(ay, ax) * RtoD;
theta = atan2(sqrt(ax^2 + ay^2), az) * RtoD;
psi = atan2(oz, -nz) * RtoD;
p = [ x, y, z, phi,theta, psi];

fprintf('Cartesian Point : ( n, o, a, p ) \n')
disp(T6); 
fprintf('(x , y , z , phi , theta , psi ): \n');
fprintf('=');
disp(p);
%Forward kinematics procedure the end

%Inverse
%kinematics------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
skip = input('Press Enter to continue');
fprintf('Inverse kinematic: \n');

T = input('Please input T : \n');
nx = T(1,1);  ny = T(2,1);  nz = T(3,1);
ox = T(1,2);  oy = T(2,2); oz = T(3,2);
ax = T(1,3);  ay = T(2,3);  az = T(3,3);
px = T(1,4);  py = T(2,4); pz = T(3,4);

%deal joint(123)
%theta1 (2 solution)
P = sqrt(px*px+py*py);
fi = atan2(py, px);
lo1 = -6.375/P;
lo2 = (1-(6.375/P)^2)^0.5;
theta1 = [fi+atan2(lo1, lo2), fi+atan2(lo1, -lo2)]* RtoD;
%theta2 (4 solution)
%depend theta1 (2 solution) and atan2 (2 solution)
temp1 = (cos(theta1(1)*DtoR)*px+sin(theta1(1)*DtoR)*py);
temp2 = (cos(theta1(2)*DtoR)*px+sin(theta1(2)*DtoR)*py);
theta2 = [atan2(temp1,pz), atan2(-temp1,-pz), atan2(temp2,pz), atan2(-temp2,-pz)]* RtoD;
%d 3
%depend theta2 (4 solution) 
temp1 = px.*cos(theta1(1)*DtoR)+py.*sin(theta1(1)*DtoR);
temp2 = px.*cos(theta1(2)*DtoR)+py.*sin(theta1(2)*DtoR);
d3 = [temp1/sin(theta2(1)*DtoR), temp1/sin(theta2(2)*DtoR), temp2/sin(theta2(3)*DtoR), temp2/sin(theta2(4)*DtoR)];
%%unify 4 theta solution of joint(123)
list1_theta = [theta1(1), theta2(1), d3(1)];
list2_theta = list1_theta;
list3_theta = [theta1(1), theta2(2), d3(2)];
list4_theta = list3_theta;
list5_theta = [theta1(2), theta2(3), d3(3)];
list6_theta = list5_theta;
list7_theta = [theta1(2), theta2(4), d3(4)];
list8_theta = list7_theta;

%theta4 (2 solution)
for i = 1:4
    if(i==1)
        T3 = A123(list1_theta, DtoR);
        T36 = inv(T3)*T6;
        theta4 = atan2(T36(2,3), T36(1,3))*RtoD;
        [theta5, theta6] = implment456(theta4,T36,DtoR,RtoD);
        list1_theta = [list1_theta, theta4, theta5, theta6];
        theta4 = atan2(-T36(2,3), -T36(1,3))*RtoD;
        [theta5, theta6] = implment456(theta4,T36,DtoR,RtoD);
        list2_theta = [list2_theta, theta4, theta5, theta6];
    elseif(i==2)
        T3 = A123(list3_theta, DtoR);
        T36 = inv(T3)*T6;
        theta4 = atan2(T36(2,3), T36(1,3))*RtoD;
        [theta5, theta6] = implment456(theta4,T36,DtoR,RtoD);
        list3_theta = [list3_theta, theta4, theta5, theta6];
        theta4 = atan2(-T36(2,3), -T36(1,3))*RtoD;
        [theta5, theta6] = implment456(theta4,T36,DtoR,RtoD);
        list4_theta = [list4_theta, theta4, theta5, theta6];
    elseif(i==3)
        T3 = A123(list5_theta, DtoR);
        T36 = inv(T3)*T6;
        theta4 = atan2(T36(2,3), T36(1,3))*RtoD;
        [theta5, theta6] = implment456(theta4,T36,DtoR,RtoD);
        list5_theta = [list5_theta, theta4, theta5, theta6];
        theta4 = atan2(-T36(2,3), -T36(1,3))*RtoD;
        [theta5, theta6] = implment456(theta4,T36,DtoR,RtoD);
        list6_theta = [list6_theta, theta4, theta5, theta6];
    elseif(i==4)
        T3 = A123(list7_theta, DtoR);
        T36 = inv(T3)*T6;
        theta4 = atan2(T36(2,3), T36(1,3))*RtoD;
        [theta5, theta6] = implment456(theta4,T36,DtoR,RtoD);
        list7_theta = [list7_theta, theta4, theta5, theta6];
        theta4 = atan2(-T36(2,3), -T36(1,3))*RtoD;
        [theta5, theta6] = implment456(theta4,T36,DtoR,RtoD);
        list8_theta = [list8_theta, theta4, theta5, theta6];
    end 
end
fprintf('------------------------------------------------\n');

print_output(list1_theta, constraint_up, constraint_down);
print_output(list2_theta, constraint_up, constraint_down);
print_output(list3_theta, constraint_up, constraint_down);
print_output(list4_theta, constraint_up, constraint_down);
print_output(list5_theta, constraint_up, constraint_down);
print_output(list6_theta, constraint_up, constraint_down);
print_output(list7_theta, constraint_up, constraint_down);
print_output(list8_theta, constraint_up, constraint_down);

%function
function A = trnsformation(d, a, alpha, theta)
    A = [ cos(theta)    -sin(theta)*cos(alpha)     sin(theta)*sin(alpha)    a*cos(theta)
    sin(theta)    cos(theta)*cos(alpha)      -cos(theta)*sin(alpha)    a*sin(theta)        
    0         sin(alpha)           cos(alpha)                    d 
    0          0             0                     1 ]; 
end
function T3 = A123(list_theta,DtoR)
    A1 = trnsformation(0, 0, -90*DtoR, list_theta(1)*DtoR);
    A2 = trnsformation(6.375, 0, 90*DtoR, list_theta(2)*DtoR);
    A3 = trnsformation(list_theta(3), 0, 0, 0);
    T3 = A1*A2*A3;
end
function [theta5, theta6] = implment456(theta4,T36,DtoR,RtoD)
    A4 = trnsformation(0, 0, -90*DtoR, theta4*DtoR);
    T46 = inv(A4)*T36;
    theta5 = atan2(T46(1,3), -T46(2,3))*RtoD;
    A5 = trnsformation(0, 0, 90*DtoR, theta5*DtoR);
    T56 = inv(A5)*T46;
    A6 = T56;
    theta6 = atan2(-T56(1,2), T56(2,2))*RtoD;
end
function print_output(list_theta, constraint_up, constraint_down)
    fprintf('Corresponding variables ( theta1, theta2, theta3, theta4, theta5, theta6 ) \n');
    for i = 1:6
        if(list_theta(i) < constraint_down(i) || list_theta(i) > constraint_up(i) )
            fprintf('theta%d out of range!\n',i);
        end
    end
    disp(list_theta);
    fprintf('------------------------------------------------\n');
end

