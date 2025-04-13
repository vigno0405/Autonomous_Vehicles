function x_rand=InformedSample(start,goal,c_best,n,m)
%
% Samples a point within the ellipsoidal region defined by start, goal and
% the solution path.
%
%   x_rand=informed_sample(start,goal,c_best,n,m)
%
% Inputs:
%   start,goal....positions of start and final point: define ellipsoid focuses
%   c_best........cost of the current best path: defines ellipsoid axis
%   n, m..........dimensions of the map
%
% Output:
%   x_rand........randomly sampled point within the ellipsoid


% Compute center and axes of the ellipsoid

center=(start+goal)/2;
c_min=norm(goal-start);

% Define rotation matrix to align the ellipsoid

dir=(goal-start)/norm(goal-start);
R=[dir;[-dir(2),dir(1)]];           % 2D rotation matrix

% Scale factors for the ellipsoid axes (geometric formulas)

L1=c_best/2;                    % semi-minor
L2=sqrt(c_best^2-c_min^2)/2;    % semi-major

% Sample a random point in the unit circle

theta=2*pi*rand();  % polar coordinates: angle
r=sqrt(rand());     % polar coordinates: radius
x=[r*cos(theta); r*sin(theta)];

% Scale and rotate the point to fit in the ellipsoid

x_ellipsoid=[L1,0;0,L2]*x;          % ellipsoid local coordinates
x_world=R'*x_ellipsoid+center';     % global coordinates

% Clamp to map boundaries (between 1,1 and n,m)

x_rand=round(max([1,1],min([n,m],x_world')));

end