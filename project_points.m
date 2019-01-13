function pp=project_points(cal_dist, cal_span_x, cal_span_y, ...
                           view_angle_x, view_angle_y, distance, ...
                           use_meters, do_plot)

if nargin <= 6
  use_meters = false
end
reference_points = get_reference_points(use_meters);
normalized_reference_points =  [ ...
    reference_points(:, 1)' / cal_span_x; ...
    reference_points(:, 2)' / cal_span_y ]';
delta_d = distance - cal_dist;
fpx = frustum_projection(cal_span_x, view_angle_x, delta_d);
fpy = frustum_projection(cal_span_y, view_angle_y, delta_d);
pp = [ normalized_reference_points(:, 1)' .* fpx; ...
       normalized_reference_points(:, 2)' .* fpy ]';
if nargin > 7
  if do_plot
    plot(normalized_reference_points(:, 1), ...
         normalized_reference_points(:, 2), 'x', ...
         pp(:, 1), pp(:, 2), 'o')
    grid
  end
end
