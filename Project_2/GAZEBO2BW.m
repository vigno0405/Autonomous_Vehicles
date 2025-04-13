function [path_BW]=GAZEBO2BW(path_GAZEBO,n_pix)
%
% Change coordinates from GAZEBO to BW references.
%
%   [pat_BW]=GAZEBO2BW(path_GAZEBO,n_pix)
%
% Inputs:
%    path_GAZEBO...path in GAZEBO coordinates
%    n_pix.........resolution [px] of BW
%
% Output:
%    path_GAZEBO...path in BW coordinates

res_BW=0.05*384/n_pix; 
path=path_GAZEBO+10;
path=(path+res_BW/2)/res_BW;
path_BW=[n_pix+1-path(:,2),path(:,1)];

end