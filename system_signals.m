function Y = system_signals(x, xd, x_d, xd_d, xdd_d, e, ed, lambda)
    % Args:
    % x [2 1]: actual position output of the system
    % x_d [2 1]: desired trajectory
    % e [2 1]: error signal, such that: e = x_d - x
    % lambda [2 2]: parameter
    %   Detailed explanation goes here

    xd_r = xd_d - lambda * e;
    xdd_r = xdd_d - lambda * ed;

    x_1 = x(1);
    x_2 = x(2);
    xd_1 = xd(1);
    xd_2 = xd(2);
    xd_r1 = xd_r(1);
    xd_r2 = xd_r(2);
    xdd_r1 = xdd_r(1);
    xdd_r2 = xdd_r(2);

    y_11 = xdd_r1;
    y_12 = xdd_r2;
    y_21 = 0;
    y_22 = xdd_r1 + xdd_r2;
    y_13 = (2*xdd_r1 + xdd_r2) * cos(x_2) - (xd_2*xd_r1 + xd_1 * xd_r2 + xd_2 * xd_r2) * sin(x_2);
    y_14 = (2*xdd_r1 + xdd_r2) * sin(x_2) - (xd_2*xd_r1 + xd_1 * xd_r2 + xd_2 * xd_r2) * cos(x_2); 
    y_23 = xdd_r1 * cos(x_2) + xd_1 * xd_r1 * sin(x_2);
    y_24 = xdd_r1 * sin(x_2) + xd_1 * xd_r1 * cos(x_2);

    Y = [y_11 y_12 y_13 y_14 ; y_21 y_22 y_23 y_24];
end
