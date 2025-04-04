%%%%%% Plotting functions %%%%%%
function plot_map(omap, start, goal)
  imshow(omap', 'InitialMagnification', 1000)
  colorbar
  hold on
  plot(start(1), start(2), 'gp');
  plot(goal(1), goal(2), 'rp');
  xlabel('x')
  ylabel('y')
end