function pp=project_points(cal_dist, cal_span_x, cal_span_y, ...
                           view_angle_x, view_angle_y, ...
                           offset_x, offset_y, distance, ...
                           use_meters, do_plot)

if nargin <= 8
  use_meters = false
end
reference_points = get_reference_points(use_meters);
normalized_reference_points =  [ ...
    2 * reference_points(:, 1)' / cal_span_x; ...
    2 * reference_points(:, 2)' / cal_span_y ]';
delta_d = distance - cal_dist;
fpx = frustum_projection(cal_span_x, view_angle_x, delta_d);
fpy = frustum_projection(cal_span_y, view_angle_y, delta_d);
tau_x = 2 * offset_x / cal_span_x;
tau_y = 2 * offset_y / cal_span_y;
shift_vector = [ ones(1, length(reference_points(:, 1))) * tau_x; ...
                 ones(1, length(reference_points(:, 2))) * tau_y ]';
shifted_points = normalized_reference_points - shift_vector;
pp = [ shifted_points(:, 1)' .* fpx; shifted_points(:, 2)' .* fpy ]';
if nargin > 9
  if do_plot
    plot(normalized_reference_points(:, 1), ...
         normalized_reference_points(:, 2), 'x', ...
         pp(:, 1), pp(:, 2), 'o')
    grid
  end
end
