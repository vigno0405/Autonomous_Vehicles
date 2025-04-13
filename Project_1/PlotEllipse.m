function [ell_plot]=PlotEllipse(start,goal,cost_best)
%
% Gives back "ell_plot" plot object of the ellipse for informed sampling in
% informed RRT*, basing on the foci "start", "goal" and on the axis defined
% by "cost_best".


center=(start+goal)/2;
c_min=norm(goal-start);
dir=(goal-start)/norm(goal-start);
R=[dir; [-dir(2),dir(1)]];
L1=cost_best/2;
L2=sqrt(cost_best^2-c_min^2)/2;
theta=linspace(0,2*pi,100);
x_local=[L1*cos(theta); L2*sin(theta)];
x_global=R'*x_local+center';
ell_plot=plot(x_global(2,:),x_global(1,:),'--','Color',[0 0 1],'LineWidth',2);

end