function T6 = kinematic(theta);

    DtoR = pi/180;
    RtoD = 180/pi;

    A1 = trnsformation(0, 0, -90*DtoR, theta(1)*DtoR);
    A2 = trnsformation(6.375*2.54, 0, 90*DtoR, theta(2)*DtoR);
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
end

function A = trnsformation(d, a, alpha, theta)
    A = [ cos(theta)    -sin(theta)*cos(alpha)     sin(theta)*sin(alpha)    a*cos(theta)
    sin(theta)    cos(theta)*cos(alpha)      -cos(theta)*sin(alpha)    a*sin(theta)        
    0         sin(alpha)           cos(alpha)                    d 
    0          0             0                     1 ]; 
end
