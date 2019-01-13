function [x, y, d] = sim_estimation(cal_dist, cal_span_x, cal_span_y, ...
                                    view_angle_x, view_angle_y, ...
                                    offset_x, offset_y, distance, ...
                                    bias, noise, ...
                                    use_meters, do_plot)
if nargin <= 10
  use_meters = false
end

reference_points = get_reference_points(use_meters);
normalized_reference_points =  [ ...
    2 * reference_points(:, 1)' / cal_span_x; ...
    2 * reference_points(:, 2)' / cal_span_y ]';
projected_points = project_points(cal_dist, cal_span_x, cal_span_y, ...
                                  view_angle_x, view_angle_y, ...
                                  offset_x, offset_y, distance, ...
                                  use_meters);
observed_points = add_error(projected_points, bias, noise);

% good stuff: estimate the distance and offset
Ax = [ normalized_reference_points(:, 1)'; ...
       -ones(1, length(normalized_reference_points(:, 1)))]';
Ay = [ normalized_reference_points(:, 2)'; ...
       -ones(1, length(normalized_reference_points(:, 2)))]';
soltn_x = (Ax' * Ax)^-1*Ax' * observed_points(:, 1);
soltn_y = (Ay' * Ay)^-1*Ay' * observed_points(:, 2);
% work backwards to get the distance and offset
sigma_dx = soltn_x(1);
sigma_dy = soltn_y(1);
tau_x = soltn_x(2)/sigma_dx;
tau_y = soltn_y(2)/sigma_dy;
ttx = tan(deg2rad(view_angle_x / 2));
tty = tan(deg2rad(view_angle_y / 2));
dist_est_x = cal_dist + cal_span_x  / (2 * ttx) * (1 - sigma_dx) / sigma_dx;
dist_est_y = cal_dist + cal_span_y  / (2 * tty) * (1 - sigma_dy) / sigma_dy;
d = (dist_est_x + dist_est_y) / 2;
x = cal_span_x / 2 * tau_x;
y = cal_span_y / 2 * tau_y;
if nargin > 11
  if do_plot
    plot(normalized_reference_points(:, 1), ...
         normalized_reference_points(:, 2), 'x', ...
         observed_points(:, 1), observed_points(:, 2), 'o')
    grid
  end
end
