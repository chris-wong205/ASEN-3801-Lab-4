function t_new = gradient_transform(t, factor)
   t_new = sign(t) .* (abs(t) .^ factor);
   t_min = min(t_new);
   t_max = max(t_new);
   t_new = (t_new - t_min) / (t_max - t_min);
end
