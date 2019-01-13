function points_with_error=add_error(points, bias, noise)

x_bias = randn() * bias;
y_bias = randn() * bias;

bias_vector = [ x_bias * ones(1, length(points(:, 1))); ...
                y_bias * ones(1, length(points(:, 2))) ]';
points_with_error = points + noise * randn(size(points)) + bias_vector;
