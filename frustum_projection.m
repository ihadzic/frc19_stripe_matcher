function p=frustum_projection(cal_span, view_angle, delta_d, do_plot)

tt = tan(deg2rad(view_angle / 2));
p = cal_span ./ (cal_span + 2 * delta_d * tt);
if nargin > 3
  if do_plot
    plot(delta_d, p)
  end
end
