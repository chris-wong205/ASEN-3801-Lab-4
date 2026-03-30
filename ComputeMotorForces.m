function motor_forces = ComputeMotorForces(Fc, Gc, d, km)

matrixF = [Fc(3), Gc]; % control forces and moments matrix
matrixF = matrixF';

matrixK = [-1, -1, -1, -1;
           -d/sqrt(2), -d/sqrt(2), d/sqrt(2), d/sqrt(2);
            d/sqrt(2), -d/sqrt(2), -d/sqrt(2), d/sqrt(2);
            km, -km, km, -km];
matrixK = inv(matrixK);

motor_forces = matrixK .* matrixF;
motor_forces = motor_forces(1,:)


end