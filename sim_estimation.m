function [x, y, d] = sim_estimation(cal_dist, cal_span_x, cal_span_y, ...
                                    view_angle_x, view_angle_y, ...
                                    offset_x, offset_y, distance, ...
                                    bias, noise, ...
                                    use_meters, do_plot)
%
%function [x, y, d] = sim_estimation(cal_dist, cal_span_x, cal_span_y, ...
%                                    view_angle_x, view_angle_y, ...
%                                    offset_x, offset_y, distance, ...
%                                    bias, noise, ...
%                                    use_meters, do_plot)
%
% Simulates the position estimation algorithm for FRC Deep Space
% 2019 game using reflective-stripe guides placed at the walls
% of the rocket, cargo ship, and loading area. The assumption is
% that before running this algorithm the four corner points
% of each stripe have been identified and that their coordinates
% have been determined. The identification of corner-points can be
% done using a number of well known vision processing algorithms
% (OpenCV gives a bunch of options that can be stitched together
% with some "plumbing" code).
%
% This model will start from some assumed (ground truth) position
% of the camera facing the reflective stripes. It will then
% calculate what the camera would see and what corner points would
% be identified by the vision processing. Next, it will add noise
% and bias error. Observed points determined in this way are
% then run through the estimator and the distance, and shift
% (along X and Y axis) are recovered. The estimator works
% by finding the scale and shift that minimizes the mean
% square error between observed and reference points.
%
% In the absence of noise, the estimator will recover
% the ground truth.
%
% If the noise or bias are present, the result will
% have an error. Running the model multiple times for same
% parameters will produce a set of results that can be used
% to understand the statistical properties of the error
%
% The model requires that the camera properties (geometric
% properties of the viewing frustum) are known, which can be
% determined by calibration.
%
% Parameters
%
% cal_dist: distance at which the camera has been calibrated
% cal_span_x: width of the area seen at calibration position
% cal_span_y: height of the area seen at calibration position
% view_angle_x: camera view angle along X axis
% view_angle_y: camera view angle along Y axis
% offset_x: ground truth location of the camera along X axis
% offset_y: ground truth location of the camera along Y axis
% distance: ground truth camera distance from the pattern
% bias: systematic error (bias) stddev (Gaussian process assumed)
% noise: noise stddev (Gaussian process assumed)
% use_meters: set to true to use metric units (meters)
%             otherwise imperial units are assumed (feet)
% do_plot: set to true to show the plot of the reference points
%          and observed points
%
% Returns:
%
% Estimates of x, y position and distance to the marker.
%
% Limitations:
%
% The model assumes that the camera is facing the
% marker directly at 90-degree angle. If the pattern is viewed
% from some other angle, an estimation error will be introduced.
% It should not be too hard to add the angular offset to the model.
%

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
