function [R] = rotation_matrix(phi, theta, psi) % This DCM is B to E rotation
  c_phi   = cos(phi);
  s_phi   = sin(phi);
  c_theta = cos(theta);
  s_theta = sin(theta);
  c_psi   = cos(psi);
  s_psi   = sin(psi);
  
  R = [
    c_theta*c_psi,  s_phi*s_theta*c_psi - c_phi*s_psi,  c_phi*s_theta*c_psi + s_phi*s_psi;
    c_theta*s_psi,  s_phi*s_theta*s_psi + c_phi*c_psi,  c_phi*s_theta*s_psi - s_phi*c_psi;
   -s_theta,        s_phi*c_theta,                      c_phi*c_theta
  ];
end