function [var_dot,Control] = QuadrotorEOMClosedLoop(t, var, g, m, I)
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


Control.Z = Zc;
Control.L = Lc;
Control.M = Mc;
Control.N = Nc;

  %Derivative of State Function
  var_dot = [
    R'* velocity;
    T'*angular_velocity;
    
    -g * theta;
    g * phi;
     0;
    
    1/(Ix) * Lc;
    1/(Iy) * Mc;
    1/(Iz) * (Nc);]

  
end
