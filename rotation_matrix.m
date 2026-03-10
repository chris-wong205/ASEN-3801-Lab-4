function R = rotation_matrix(attitude321) % E to B DCM
  R = roll( attitude321(3) ) * pitch( attitude321(2) ) * yaw( attitude321(1) );
end
