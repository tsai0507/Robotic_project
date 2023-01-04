clear all
close all

A = [0 0 -1 10; -1 0 0 20; 0 1 0 30; 0 0 0 1];  
B = [1 0 0 30; 0 1 0 20; 0 0 1 10; 0 0 0 1];  
C = [0 -1 0 -10; 0 0 1 -20; -1 0 0 -30; 0 0 0 1];  
sampling_rate = 0.002;
[xA,yA,zA,RA,PA,YA] = xyzRPY(A);
[xB,yB,zB,RB,PB,YB] = xyzRPY(B);
[xC,yC,zC,RC,PC,YC] = xyzRPY(C);

%Straight A
indexA = 0;   %  the index of the data of the matrix 
x = xB - xA;y = yB - yA;z = zB - zA;
R = RB - RA;P = PB - PA;Y = YB - YA;
for t=-0.5:sampling_rate:-0.2
    indexA=indexA+1;
    h=(t+0.5)/0.5;
    dx=x*h;dy=y*h;dz=z*h;
    dR=R*h;dP=P*h;dY=Y*h;
    xA_B(:,indexA)=xA+dx;yA_B(:,indexA)=yA+dy;zA_B(:,indexA)=zA+dz; 
    R_A(:,indexA) = RA+dR;P_A(:,indexA) =PA+dP;Y_A(:,indexA) =YA+dY;
end
for i = 1:indexA
    ori_A(:,i) = Orientate(R_A(:,i),P_A(:,i),Y_A(:,i));
end

%Straight C
indexC = 0;   %  the index of the data of the matrix 
x = xC - xB;y = yC - yB;z = zC - zB;
R = RC - RB;P = PC - PB;Y = YC - YB;
for t=0.2:sampling_rate:0.5
    indexC=indexC+1;
    h=(t)/0.5;
    dx=x*h;dy=y*h;dz=z*h;
    dR=R*h;dP=P*h;dY=Y*h;
    x_C(:,indexC)=xB+dx;y_C(:,indexC)=yB+dy;z_C(:,indexC)=zB+dz; 
    R_C(:,indexC) =RB+dR;P_C(:,indexC) =PB+dP;Y_C(:,indexC) =YB+dY;
end
for i = 1:indexC
    ori_C(:,i) = Orientate(R_C(:,i),P_C(:,i),Y_C(:,i));
end

%Curve B
%The end point of Straight A is the start point of curve B
xA = xA_B(:,indexA)-xB;yA = yA_B(:,indexA)-yB;zA = zA_B(:,indexA)-zB;
RA = R_A(:,indexA)-RB;PA = P_A(:,indexA)-RB;YA = Y_A(:,indexA)-RB;
%The atart point of Straight C is the end point of curve B 
xC = x_C(:,1)-xB;yC = y_C(:,1)-yB;zC = z_C(:,1)-zB;
RC = R_C(:,1)-RB;PC = P_C(:,1)-PB;YC = Y_C(:,1)-YB;
indexB=0; 
for t=(-0.2+sampling_rate):sampling_rate:(0.2-sampling_rate)
    indexB=indexB+1;
    h=(t+0.2)/(2*0.2);
    x_B(:,indexB)=xB + ((xC+xA)*(2-h)*h^2-2*xA)*h+xA;
    y_B(:,indexB)=yB + ((yC+yA)*(2-h)*h^2-2*yA)*h+yA;
    z_B(:,indexB)=zB + ((zC+zA)*(2-h)*h^2-2*zA)*h+zA;
    R_B(:,indexB)=RB + ((RC+RA)*(2-h)*h^2-2*RA)*h+RA;
    P_B(:,indexB)=PB + ((PC+PA)*(2-h)*h^2-2*PA)*h+PA;
    Y_B(:,indexB)=YB + ((YC+YA)*(2-h)*h^2-2*YA)*h+YA;   
end
for i = 1:indexB
    ori_B(:,i) = Orientate(R_B(:,i),P_B(:,i),Y_B(:,i));
end

% 3D路徑圖
figure(1);
x_all = [xA_B x_B x_C]; y_all = [yA_B y_B y_C]; z_all = [zA_B z_B z_C]; 
ori_all = [ori_A ori_B ori_C];
quiver3(x_all,y_all,z_all,ori_all(1,:),ori_all(2,:),ori_all(3,:));
xlabel('x(cm)');ylabel('y(cm)');zlabel('z(cm)');
text(10,20,30,'A(10,20,30)');
text(30,20,10,'B(30,20,10)');
text(-10,-20,-30,'C(-10,-20,-30)');
title('Cartesion Motion')

% XYZ的位置變化情形
X=[xA_B x_B x_C];Y=[yA_B y_B y_C];Z=[zA_B z_B z_C];
X1=[xA_B x_B];Y1=[yA_B y_B];Z1=[zA_B z_B];
t=0:sampling_rate:1;
figure(2);
subplot(3,1,1);plot(t,X);
xlabel('Time(s)');ylabel('Position(cm)');
title('position of x');grid;
%%
subplot(3,1,2);plot(t,Y);
xlabel('Time(s)');ylabel('Position(cm)');
title('position of y');grid;
%%
subplot(3,1,3);plot(t,Z);
xlabel('Time(s)');ylabel('Position(cm)');
title('position of z');grid;

% XYZ的速度變化情形
%Diff 1
dt=t(2:501);
dX=diff(X)/sampling_rate;
dY=diff(Y)/sampling_rate;
dZ=diff(Z)/sampling_rate;
figure(3)
subplot(3,1,1);plot(dt,dX);
xlabel('Time(s)');ylabel('Velocity(cm/s)');
title('velocity of x');grid;
%%
subplot(3,1,2);plot(dt,dY);
xlabel('Time(s)');ylabel('Velocity(cm/s)');
title('velocity of y');grid;
%%
subplot(3,1,3);plot(dt,dZ);
xlabel('Time(s)');ylabel('Velocity(cm/s)');
title('velocity of z');grid;

% XYZ的加速度變化情形
%Diff 2
dt2=t(3:501);
dX2=diff(dX)/sampling_rate;
dY2=diff(dY)/sampling_rate;
dZ2=diff(dZ)/sampling_rate;
figure(4);
subplot(3,1,1);plot(dt2,dX2);
xlabel('Time(s)');ylabel('Acceleration(cm/s^2)');
title('acceleration of x');grid;
%%
subplot(3,1,2);plot(dt2,dY2);
xlabel('Time(s)');ylabel('Acceleration(cm/s^2)');
title('acceleration of y');grid;
%%
subplot(3,1,3);plot(dt2,dZ2);
xlabel('Time(s)');ylabel('Acceleration(cm/s^2)');
title('acceleration of z');grid;


function [x,y,z,phi,theta,psi] = xyzRPY(T6)
    RtoD = 180/pi;   
    nx = T6(1,1); ny = T6(2,1); nz = T6(3,1);
    ox = T6(1,2); oy = T6(2,2); oz = T6(3,2);
    ax = T6(1,3); ay = T6(2,3); az = T6(3,3);
    px = T6(1,4); py = T6(2,4); pz = T6(3,4);
    x = px; y = py; z = pz;
    phi = atan2(ay, ax) * RtoD;
    theta = atan2(sqrt(ax^2 + ay^2), az) * RtoD;
    psi = atan2(oz, -nz) * RtoD;
    % p = [ x, y, z, phi,theta, psi];
end
function ori = Orientate(R,P,Y)
    R = R*pi/180;
    P = P*pi/180;
    Y = Y*pi/180;
    ori = [cos(R)*sin(P) sin(R)*sin(P) cos(P)];
end