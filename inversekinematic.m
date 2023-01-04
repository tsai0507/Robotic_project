function Jthita = inversekinematic(T);
    T6 = T;
    DtoR = pi/180;
    RtoD = 180/pi;
    nx = T(1,1);  ny = T(2,1);  nz = T(3,1);
    ox = T(1,2);  oy = T(2,2); oz = T(3,2);
    ax = T(1,3);  ay = T(2,3);  az = T(3,3);
    px = T(1,4);  py = T(2,4); pz = T(3,4);

    %deal joint(123)
    %theta1 (2 solution)
    P = sqrt(px*px+py*py);
    fi = atan2(py, px);
    lo1 = -6.375*2.54/P;
    lo2 = (1-(6.375*2.54/P)^2)^0.5;
    theta1 = [fi+atan2(lo1, lo2), fi+atan2(lo1, -lo2)]* RtoD
    %theta2 (4 solution)
    %depend theta1 (2 solution) and atan2 (2 solution)
    temp1 = (cos(theta1(1)*DtoR)*px+sin(theta1(1)*DtoR)*py);
    temp2 = (cos(theta1(2)*DtoR)*px+sin(theta1(2)*DtoR)*py);
    theta2 = [atan2(temp1,pz), atan2(-temp1,-pz), atan2(temp2,pz), atan2(-temp2,-pz)]* RtoD;
    %d3
    %depend theta2 (4 solution) 
    temp1 = px.*cos(theta1(1)*DtoR)+py.*sin(theta1(1)*DtoR);
    temp2 = px.*cos(theta1(2)*DtoR)+py.*sin(theta1(2)*DtoR);
    d3 = [temp1/sin(theta2(1)*DtoR), temp1/sin(theta2(2)*DtoR), temp2/sin(theta2(3)*DtoR), temp2/sin(theta2(4)*DtoR)];
    %%unify 4 theta solution of joint(123)
    list1_theta = [theta1(1), theta2(1), d3(1)];list2_theta = list1_theta;
    list3_theta = [theta1(1), theta2(2), d3(2)];list4_theta = list3_theta;
    list5_theta = [theta1(2), theta2(3), d3(3)];list6_theta = list5_theta;
    list7_theta = [theta1(2), theta2(4), d3(4)];list8_theta = list7_theta;

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
            list7_theta = [list7_theta, theta4, theta5, theta6];    nx = T(1,1);  ny = T(2,1);  nz = T(3,1);
            ox = T(1,2);  oy = T(2,2); oz = T(3,2);
            ax = T(1,3);  ay = T(2,3);  az = T(3,3);
            px = T(1,4);  py = T(2,4); pz = T(3,4);
        
            %deal joint(123)
            %theta1 (2 solution)
            P = sqrt(px*px+py*py);
            fi = atan2(py, px);
            lo1 = -6.375*2.54/P;
            lo2 = (1-(6.375*2.54/P)^2)^0.5;
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
                    list1_theta = fix([list1_theta, theta4, theta5, theta6])
                    theta4 = atan2(-T36(2,3), -T36(1,3))*RtoD;
                    [theta5, theta6] = implment456(theta4,T36,DtoR,RtoD);
                    list2_theta = fix([list2_theta, theta4, theta5, theta6])
                elseif(i==2)
                    T3 = A123(list3_theta, DtoR);
                    T36 = inv(T3)*T6;
                    theta4 = atan2(T36(2,3), T36(1,3))*RtoD;
                    [theta5, theta6] = implment456(theta4,T36,DtoR,RtoD);
                    list3_theta = fix([list3_theta, theta4, theta5, theta6])
                    theta4 = atan2(-T36(2,3), -T36(1,3))*RtoD;
                    [theta5, theta6] = implment456(theta4,T36,DtoR,RtoD);
                    list4_theta = fix([list4_theta, theta4, theta5, theta6])
                elseif(i==3)
                    T3 = A123(list5_theta, DtoR);
                    T36 = inv(T3)*T6;
                    theta4 = atan2(T36(2,3), T36(1,3))*RtoD;
                    [theta5, theta6] = implment456(theta4,T36,DtoR,RtoD);
                    list5_theta = fix([list5_theta, theta4, theta5, theta6])
                    theta4 = atan2(-T36(2,3), -T36(1,3))*RtoD;
                    [theta5, theta6] = implment456(theta4,T36,DtoR,RtoD);
                    list6_theta = fix([list6_theta, theta4, theta5, theta6])
                elseif(i==4)
                    T3 = A123(list7_theta, DtoR);
                    T36 = inv(T3)*T6;
                    theta4 = atan2(T36(2,3), T36(1,3))*RtoD;
                    [theta5, theta6] = implment456(theta4,T36,DtoR,RtoD);
                    list7_theta = fix([list7_theta, theta4, theta5, theta6])
                    theta4 = atan2(-T36(2,3), -T36(1,3))*RtoD;
                    [theta5, theta6] = implment456(theta4,T36,DtoR,RtoD);
                    list8_theta = fix([list8_theta, theta4, theta5, theta6])
                end 
            end
        end
    end
    Jthita = list5_theta;
    
end

%function
function A = trnsformation(d, a, alpha, theta)
    A = [ cos(theta)    -sin(theta)*cos(alpha)     sin(theta)*sin(alpha)    a*cos(theta)
    sin(theta)    cos(theta)*cos(alpha)      -cos(theta)*sin(alpha)    a*sin(theta)        
    0         sin(alpha)           cos(alpha)                    d 
    0          0             0                     1 ]; 
end
function T3 = A123(list_theta,DtoR)
    A1 = trnsformation(0, 0, -90*DtoR, list_theta(1)*DtoR);
    A2 = trnsformation(6.375*2.54, 0, 90*DtoR, list_theta(2)*DtoR);
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
function list_theta = fix(list_theta)
    for i= 1:6
        temp = list_theta(i);
        if(temp < -180)
            temp = temp + 360 ;
        elseif(temp > 180)
            temp = temp - 360;
        end
        list_theta(i) = temp;
    end
end


