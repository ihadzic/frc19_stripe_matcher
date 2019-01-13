function reference_points=get_reference_points(use_meters)

% construct reference points and convert them to feet
% REVISIT: need more accurate coordinates
right_stripe = [ 4 2.5; 6 3; 7.4 -2.3; 5.4 -2.8 ];
left_stripe = [ -right_stripe(:, 1)'; right_stripe(:, 2)']';
reference_points = [ right_stripe; left_stripe ] / 12.0;
if nargin > 0
  if use_meters
    reference_points = reference_points * 0.0252;
  end
end