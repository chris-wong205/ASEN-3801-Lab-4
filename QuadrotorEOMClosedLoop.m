function [var_dot,Fc,Gc] = QuadrotorEOMClosedLoop(t, var, g, m, I)
  phi      = var(4);
  theta    = var(5);
  psi      = var(6);
  velocity = var(7 : 9);
  angular_velocity = var(10 : 12);

  [Fc, Gc] = InnerLoopFeedback(var);

  R = rotation_matrix([psi, theta, phi]);
  T = [
    1, sin(phi) * tan(theta), cos(phi) * tan(theta);
    0, cos(phi), -sin(phi);
    0, sin(phi) / cos(theta), cos(phi) / cos(theta);
  ];

  Ix = I(1,1);
  Iy = I(2,2);
  Iz = I(3,3);

  Zc = Fc(3,1);
  Lc = Gc(1,1);
  Mc = Gc(2,1);
  Nc = Gc(3,1);

  %Derivative of State Function
  var_dot = [
    velocity;
    angular_velocity;
    
    g * phi;
    -g * theta;
    (1/m) * Zc;
    
    1/(Ix) * Lc;
    1/(Iy) * Mc;
    1/(Iz) * (Nc);];

  var_dot(var_dot<10e-10) = 0;
end
