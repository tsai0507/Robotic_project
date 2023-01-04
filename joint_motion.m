clc
clear all
close all

sampling_rate = 0.002;
A = [0 0 -1 10; -1 0 0 20; 0 1 0 30; 0 0 0 1];  
B = [1 0 0 30; 0 1 0  20; 0 0 1 10; 0 0 0 1];  
C = [0 -1 0 -10; 0 0 1 -20; -1 0 0 -30; 0 0 0 1]; 

% thita_A=inversekinematic(A);
% thita_B=inversekinematic(B);
% thita_C=inversekinematic(C);

thita_A=[-70.1668 -27.2045 33.7313 -107.7851 81.0769 64.1944];
thita_B=[7.0042 72.7549 33.7313 -0.0000 -72.7549 -7.0042];
thita_C=[109.8332 27.2045 -33.7313 -22.0744 64.5293 9.8929];
thitaA=thita_A(1,:)';thitaB=thita_B(1,:)';thitaC=thita_C(1,:)';                          
%~~~~~~~~~~~~~~~~~(partA) 
s1=0;
for t=-0.5:sampling_rate:-0.2
    s1=s1+1;
    h=(t+0.5)/0.5;
    dthitaA(:,s1)=thitaA+(thitaB-thitaA)*h;                   
    domegaA(:,s1)=(thitaB-thitaA)/0.5;
    dalphaA(:,s1)=[0;0;0;0;0;0];
    p1=kinematic(dthitaA(:,s1)');
    x1(s1)=p1(1,4);y1(s1)=p1(2,4);z1(s1)=p1(3,4);
    ori_1(:,s1) = [p1(1,3) p1(2,3) p1(3,3)];
end
%~~~~~~~~~~~~~~~~~~(partC)
s3=0;
for t=0.2:sampling_rate:0.5;
    s3=s3+1;
    h=t/0.5;
    dthitaC(:,s3)=thitaB+(thitaC-thitaB)*h;
    domegaC(:,s3)=(thitaC-thitaB)/0.5;
    dalphaC(:,s3)=[0;0;0;0;0;0];
    p3=kinematic(dthitaC(:,s3)');
    x3(s3)=p3(1,4);y3(s3)=p3(2,4);z3(s3)=p3(3,4);
    ori_3(:,s3) = [p3(1,3) p3(2,3) p3(3,3)];
end
%~~~~~~~~~~~~~~~~~~~(partB) 
dB=dthitaA(:,s1)-thitaB;
dC=dthitaC(:,1)-thitaB;
s2=0;
for t=(-0.2+sampling_rate):sampling_rate:(0.2-sampling_rate)
    s2=s2+1;
    h=(t+0.2)/0.4;
    dthitaB(:,s2)=((dC+dB)*(2-h)*h^2-2*dB)*h+dB+thitaB;        
    domegaB(:,s2)=((dC+dB)*(1.5-h)*2*h^2-dB)/0.2;
    dalphaB(:,s2)=(dC+dB)*(1-h)*3*h/0.2^2;
    p2=kinematic(dthitaB(:,s2)');
    x2(s2)=p2(1,4);y2(s2)=p2(2,4);z2(s2)=p2(3,4);
    ori_2(:,s2) = [p2(1,3) p2(2,3) p2(3,3)];
end

%~~~~~~~~~~~~Joint View~~~~~~~~~~%
t=-0.5:sampling_rate:0.5;
figure(1)   %position
thita1=[dthitaA(1,:) dthitaB(1,:) dthitaC(1,:)];                     
subplot(3,2,1);plot(t,thita1);
grid;title('joint1 angle');ylabel('Angle');

thita2=[dthitaA(2,:) dthitaB(2,:) dthitaC(2,:)];                     
subplot(3,2,2);plot(t,thita2);
grid;title('joint2 angle');ylabel('Angle');

thita3=[dthitaA(3,:) dthitaB(3,:) dthitaC(3,:)];                  
subplot(3,2,3);plot(t,thita3);
grid;title('joint3 angle');ylabel('Angle');

thita4=[dthitaA(4,:) dthitaB(4,:) dthitaC(4,:)];                    
subplot(3,2,4);plot(t,thita4);
grid;title('joint4 angle');ylabel('Angle');

thita5=[dthitaA(5,:) dthitaB(5,:) dthitaC(5,:)];                  
subplot(3,2,5);plot(t,thita5);
grid;title('joint5 angle');ylabel('Angle');

thita6=[dthitaA(6,:) dthitaB(6,:) dthitaC(6,:)];                      
subplot(3,2,6);plot(t,thita6);
grid;title('joint6 angle');ylabel('Angle');
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
figure(2)   %velocity
subplot(3,2,1);plot(t,[domegaA(1,:) domegaB(1,:) domegaC(1,:)]);  
grid;title('joint1');ylabel('Angular Velocity');

subplot(3,2,2);plot(t,[domegaA(2,:) domegaB(2,:) domegaC(2,:)]);   
grid;title('joint2');ylabel('Angular Velocity');

subplot(3,2,3);plot(t,[domegaA(3,:) domegaB(3,:) domegaC(3,:)]); 
grid;title('joint3');ylabel('Angular Velocity');

subplot(3,2,4);plot(t,[domegaA(4,:) domegaB(4,:) domegaC(4,:)]);    
grid;title('joint4');ylabel('Angular Velocity');

subplot(3,2,5);plot(t,[domegaA(5,:) domegaB(5,:) domegaC(5,:)]);  
grid;title('joint5');ylabel('Angular Velocity');

subplot(3,2,6);plot(t,[domegaA(6,:) domegaB(6,:) domegaC(6,:)]);   
grid;title('joint6');ylabel('Angular Velocity');
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
figure(3)   %acceleration
subplot(3,2,1);plot(t,[dalphaA(1,:) dalphaB(1,:) dalphaC(1,:)]);
grid;title('joint1');ylabel('Angular Acceleration');

subplot(3,2,2);plot(t,[dalphaA(2,:) dalphaB(2,:) dalphaC(2,:)]);
grid;title('joint2');ylabel('Angular Acceleration');

subplot(3,2,3);plot(t,[dalphaA(3,:) dalphaB(3,:) dalphaC(3,:)]);
grid;title('joint3');ylabel('Angular Acceleration');

subplot(3,2,4);plot(t,[dalphaA(4,:) dalphaB(4,:) dalphaC(4,:)]);
grid;title('joint4');ylabel('Angular Acceleration');

subplot(3,2,5);plot(t,[dalphaA(5,:) dalphaB(5,:) dalphaC(5,:)]);
grid;title('joint5');ylabel('Angular Acceleration');

subplot(3,2,6);plot(t,[dalphaA(6,:) dalphaB(6,:) dalphaC(6,:)]);
grid;title('joint6');ylabel('Angular Acceleration');
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
figure(4)% Cartesian
x_all = [x1 x2 x3]; y_all = [y1 y2 y3]; z_all = [z1 z2 z3]; 
ori_all = [ori_1 ori_2 ori_3];
% plot3(x_all,y_all,z_all);
quiver3(x_all,y_all,z_all,ori_all(1,:),ori_all(2,:),ori_all(3,:));
xlabel('x(cm)');ylabel('y(cm)');zlabel('z(cm)');
text(10,20,30,'A(10,20,30)');
text(30,20,10,'B(30,20,10)');
text(-10,-20,-30,'C(-10,-20,-30)');
title('Cartesion Motion')

