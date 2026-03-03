function var_dot = QuadrotorEOM(t, var, g, m, I, d, km, nu, mu, motor_forces)
  phi   = var(4);
  theta = var(5);
  psi   = var(6);
  velocity = var(7 : 9);
  angular_velocity = var(10 : 12);

  R = rotation_matrix(phi, theta, psi);
  T = [
    1, sin(phi) * tan(theta), cos(phi) * tan(theta);
    0, cos(phi), -sin(phi);
    0, sin(phi) / cos(theta), cos(phi) / cos(theta);
  ];

  I_B = -I + (2 * diag(diag(I)));
  M = zeros(3);

  var_dot = [
    R * velocity;
    T * angular_velocity;
    cross(velocity, angular_velocity) + g * (R' * [0; 0; 1]) + (1 / m) [0; 0; sum(motor_forces)];
    inverse(I_B) * (M - cross(angular_velocity, I_B * angular_velocity));
  ];
end
