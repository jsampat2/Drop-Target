function [xdot] = freefall(t,x,Cd, A, m, rho, g)
    k = ((0.5) * (rho) * A * Cd)/m;
    %v_wind = rand(1);
    xdot(1) = x(2);
    xdot(2) = -k * (x(2))^2;
    xdot(3) = x(4);
    xdot(4) = 0;
    xdot(5) = x(6);
    xdot(6) = -g - k * x(4)^2;
    
    xdot = [xdot(1); xdot(2); xdot(3); xdot(4); xdot(5); xdot(6)];
end