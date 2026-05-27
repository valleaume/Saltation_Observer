function J = ASLIP_Compute_Jacobian(qb, qt, lb)
    % Compute the Jacobian of T_bl explicitly (3x5 matrix)
    %
    % Args:
    %   qb: Body configuration [x_b, y_b, theta_b] (array)
    %   qt: Toe position [x_t, y_t] (array)
    %   lb: Distance from hip to COM (float)
    %
    % Returns:
    %   J: Jacobian matrix (3x5 array)

    x_b = qb(1);
    y_b = qb(2);
    theta_b = qb(3);

    x_t = qt(1);
    y_t = qt(2);

    % Compute A, B, r
    A = y_b - lb * sin(theta_b) - y_t;
    B = x_b - lb * cos(theta_b) - x_t;
    r = sqrt(A^2 + B^2);

    % Precompute common terms
    cos_theta_b = cos(theta_b);
    sin_theta_b = sin(theta_b);
    term1 = (B * cos_theta_b + A * sin_theta_b) / r^2;
    term2 = (B * sin_theta_b - A * cos_theta_b) / r;

    % Row 1: d(theta_t)/d[x_b, y_b, theta_b, x_t, y_t]
    dtheta_t_dxb = -A / r^2;
    dtheta_t_dyb = B / r^2;
    dtheta_t_dthetab = -lb * term1;
    dtheta_t_dxt = A / r^2;
    dtheta_t_dyt = -B / r^2;

    % Row 2: d(theta_b_leg)/d[x_b, y_b, theta_b, x_t, y_t]
    dtheta_b_leg_dxb = A / r^2;
    dtheta_b_leg_dyb = -B / r^2;
    dtheta_b_leg_dthetab = 1 + lb * term1;
    dtheta_b_leg_dxt = -A / r^2;
    dtheta_b_leg_dyt = B / r^2;

    % Row 3: d(l_l)/d[x_b, y_b, theta_b, x_t, y_t]
    dl_l_dxb = B / r;
    dl_l_dyb = A / r;
    dl_l_dthetab = lb * term2;
    dl_l_dxt = -B / r;
    dl_l_dyt = -A / r;

    % Construct the Jacobian matrix
    J = [
        dtheta_t_dxb,      dtheta_t_dyb,      dtheta_t_dthetab,      dtheta_t_dxt,      dtheta_t_dyt;
        dtheta_b_leg_dxb,  dtheta_b_leg_dyb,  dtheta_b_leg_dthetab,  dtheta_b_leg_dxt,  dtheta_b_leg_dyt;
        dl_l_dxb,          dl_l_dyb,          dl_l_dthetab,          dl_l_dxt,          dl_l_dyt
    ];
end
