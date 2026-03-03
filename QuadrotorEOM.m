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

  %Aerodynamic Forces and Moments

      AeroForce = (1/m) * (-nu)*velocity*norm(velocity);
    
      %Aeroforce = [0;0;0];

      % Calculate aerodynamic moments
      AeroMoment = -mu * angular_velocity .* norm(angular_velocity);

      %AeroMoment = [0;0;0];


    






% Control Moments

% Force vector
f = motor_forces;

% Allocation matrix
A = [ -1,        -1,        -1,        -1;
     -d/sqrt(2), -d/sqrt(2),  d/sqrt(2),  d/sqrt(2);
      d/sqrt(2), -d/sqrt(2), -d/sqrt(2),  d/sqrt(2);
      km,        -km,        km,        -km ];

% Output vector [Zc; Lc; Mc; Nc]
u = A * f;

Zc = u(1);
Lc = u(2);
Mc = u(3);
Nc = u(4);

%Derivative of State Function

 var_dot = [
    R * velocity;
    T * angular_velocity;
    cross(velocity, angular_velocity) + g * (R' * [0; 0; 1]) + AeroForce +  (1/m) * [0; 0; Zc];
    (1/diag(I_B))' .* (- cross(angular_velocity, diag(I_B) .* angular_velocity)) + (1./diag(I_B)) .* AeroMoment + (1./diag(I_B)).*[Lc;Mc;Nc]
];
end
