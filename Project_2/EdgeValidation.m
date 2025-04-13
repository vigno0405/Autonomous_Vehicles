function valid=EdgeValidation(map,point1,point2)
%
% Checks wether the edge between point1 and point2 crosses an obstacle of 
% the map or not.
%
%   valid=EdgeValidation(map,point1,point2)
%
% Inputs:
%   map..............binary matrix: nxm binary matrix with 1
%                    (free nodes) and 0 (not free nodes)
%   point1,point2....vertices of the segment
%
% Outputs:
%   valid............true if the segment does not intersect any obstacle


% Extract coordinates

x1=point1(1); 
y1=point1(2);
x2=point2(1); 
y2=point2(2);

% Compute edge length

dx=x2-x1;
dy=y2-y1;

% Discretise along the segment

steps=max(abs(dx),abs(dy));

x_inc=dx/steps;
y_inc=dy/steps;

% Check if the "discrete points" of the segment are meeting an obstacle

[n,m]=size(map);
x=x1;
y=y1;
for ii=1:steps

    % Approximate the "segment-point" with a point in the grid
    xii=round(x);
    yii=round(y);

    % Check if each "segment-point" is valid (within the map and not an obstacle)
    if xii<1 || xii>n || yii<1 || yii>m || map(xii,yii)==0
        valid=false;
        return
    end

    % Next "segment-point"
    x=x+x_inc;
    y=y+y_inc;

end

% If all "segment-points" are good, the edge is valid
valid=true;

end