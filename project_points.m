function pp=project_points(cal_dist, cal_span, view_angle, distance, ...
                           use_meters, do_plot)

if nargin <= 4
  use_meters = false
end
reference_points = get_reference_points(use_meters);
normalized_reference_points = reference_points / cal_span;
delta_d = distance - cal_dist;

fp = frustum_projection(cal_span, view_angle, delta_d);
pp = normalized_reference_points .* fp;

if nargin > 5
  if do_plot
    plot(normalized_reference_points(:, 1), ...
         normalized_reference_points(:, 2), 'x', ...
         pp(:, 1), pp(:, 2), 'o')
    grid
  end
end