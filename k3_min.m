function [k3_min, eigenvalues] = k3_min(A, A_size, k3)
   len = length(k3);
   eigenvalues = zeros(A_size, len);
   
   for i = 1 : len
       control_matrix = A(k3(i));
       [~, D] = eig(control_matrix);

       eigenvalues(:, i) = diag(D); % store eigenvalues
   end

   sigma_max                = -1 / 1.25;
   eig_real                 = real(eigenvalues);
   eig_im                   = imag(eigenvalues);
   eig_diff                 = eig_real - sigma_max;
   eig_discard              = any( and(eig_real ~= 0, or(eig_diff > 0, eig_im ~= 0)), 1 );
   eig_diff(:, eig_discard) = inf;
   [~, eig_diff_min]        = min(abs(eig_diff), [], "all");
   [~, eig_diff_min_col]    = ind2sub(size(eigenvalues), eig_diff_min);
   k3_min                   = k3(eig_diff_min_col);
end
