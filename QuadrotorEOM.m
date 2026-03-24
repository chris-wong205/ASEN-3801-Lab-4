function var_dot = QuadrotorEOM(t, var, g, m, I, d, km, nu, mu, motor_forces)
  phi      = var(4);
  theta    = var(5);
  psi      = var(6);
  velocity = var(7 : 9);
  angular_velocity = var(10 : 12);

  R = rotation_matrix([psi, theta, phi]);
  T = [
    1, sin(phi) * tan(theta), cos(phi) * tan(theta);
    0, cos(phi), -sin(phi);
    0, sin(phi) / cos(theta), cos(phi) / cos(theta);
  ];

  I_B = -I + (2 * diag(diag(I)));

  %Aerodynamic Forces and Moments
  drag_force  = -nu * norm(velocity)         * velocity;
  drag_moment = -mu * norm(angular_velocity) * angular_velocity;

  % Control Moments
  m_arm = d / sqrt(2);

  % Allocation matrix
  A = [-1,     -1,     -1,     -1;
       -m_arm, -m_arm,  m_arm,  m_arm;
        m_arm, -m_arm, -m_arm,  m_arm;
        km,    -km,     km,    -km ];

  % Output vector [Zc; Lc; Mc; Nc]
  u = A * motor_forces;

  Zc = u(1);
  Lc = u(2);
  Mc = u(3);
  Nc = u(4);

  %Derivative of State Function
  var_dot = [
    R' * velocity;
    T * angular_velocity;
   -cross(angular_velocity, velocity) + (R * [0; 0; g]) + (1 / m) * (drag_force + [0; 0; Zc]);
    I_B \ ( -cross(angular_velocity, I_B * angular_velocity) + drag_moment + [Lc; Mc; Nc] );
  ];
  
  var_dot(var_dot < 10e-10) = 0;
end
