function u = ASMC(xdd_d, e, ed, e_int, Y, H, lambda, k_D, estimation_gain, t)
    % Define persistent variables to hold the state between time steps
    persistent integral_ad;
    persistent prev_time;

    % Initialize the persistent variables
    if isempty(integral_ad)
        integral_ad = zeros(4, 1); % Initialize the integral state to zero
        prev_time = 0; % Initialize previous time
    end

    % Get the current simulation time
    curr_time = t;

    % estimated values of unmodeled dynamics
    b_inv = 5 * H;
    f = 5 * (xdd_d - 2 * lambda * ed - lambda * lambda * e);

    % sliding surface definition
    s = ed + 2 * lambda * e + lambda * lambda * e_int;
    disp("sliding surface definition: ");
    disp(s);

    % estimate parameters update - eq(21)
    ad = -1 * estimation_gain * transpose(Y) * s;
    disp("estimate parameters update - derivative: ");
    disp(ad);

    % Calculate the time step
    dt = curr_time - prev_time;

    % Perform the integration using the trapezoidal method
    integral_ad = integral_ad + ad * dt;

    % Update the previous time
    prev_time = curr_time;

    % Output the integrated signal
    a = integral_ad;

    % equivalent control term
    u_eq = b_inv * (xdd_d - f - 2 * lambda * ed - lambda * lambda * e);

    % adaptive control term
    u_a = b_inv * (Y * a - k_D * s);

    % control law, aka, input signal to the system plant
    u = u_a + u_eq;
    disp(u);

end
