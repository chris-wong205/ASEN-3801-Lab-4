function [var_dot,Control] = QuadrotorEOMNonLinearClosedLoop(t, var, g, m, I, d, km, nu, mu, motor_forces)

%This function will return the Non Linear Equations of motion for the
%system. The system will then use ODE 45 in order to solve the the states
%of the Quadrotor


%Intialize Variables and Values
  phi      = var(4);
  theta    = var(5);
  psi      = var(6);
  velocity = var(7 : 9);
  angular_velocity = var(10 : 12);

  [Fc, Gc] = InnerLoopFeedback(var);

  
  Zc = Fc(3,1);
  Lc = Gc(1,1);
  Mc = Gc(2,1);
  Nc = Gc(3,1);

 
%Rotation Matrix
  R = rotation_matrix([psi, theta, phi]);
  T = [
    1, sin(phi) * tan(theta), cos(phi) * tan(theta);
    0, cos(phi), -sin(phi);
    0, sin(phi) / cos(theta), cos(phi) / cos(theta);
  ];


%Post Processing
Control.Z = Zc;
Control.L = Lc;
Control.M = Mc;
Control.N = Nc;


  I_B = -I + (2 * diag(diag(I)));

  %Aerodynamic Forces and Moments
  drag_force  = -nu * norm(velocity) * velocity;
  drag_moment = -mu * norm(angular_velocity) * angular_velocity;

  % Control Moments
  m_arm = d / sqrt(2);

  % Allocation matrix
  A = [-1,     -1,     -1,     -1;
       -m_arm, -m_arm,  m_arm,  m_arm;
        m_arm, -m_arm, -m_arm,  m_arm;
        km,    -km,     km,    -km ];

  
  Rmat = R' * velocity;
  Omat = T * angular_velocity;
  Vmat = -cross(angular_velocity, velocity) + (R * [0; 0; g]) + (1 / m) * (drag_force + [0; 0; Zc;]);
  OmegaMat = I_B \ ( -cross(angular_velocity, I_B * angular_velocity) + drag_moment + [Lc; Mc; Nc] );
  %Derivative of State Function
  
  var_dot = [Rmat;Omat;Vmat;OmegaMat];
    
   

  
end
