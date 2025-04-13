function [path_GAZEBO,res_BW]=BW2GAZEBO(path_BW,n_pix)
%
% Change coordinates from BW to GAZEBO references.
%
%   [path_GAZEBO,res_BW]=BW2GAZEBO(path_BW,n_pix)
%
% Inputs:
%    path_BW.......path defined in the binary map (with origin at the
%                  top-left corner)
%    n_pix.........resolution [px] of BW
%
% Output:
%    path_GAZEBO...path in GAZEBO coordinates
%    res_BW........resolution [m] of a pixel in BW


res_BW=0.05*384/n_pix;                       % resolution of a single pixel in BW [m]
path=[path_BW(:,2),n_pix+1-path_BW(:,1)];    % rotation of coordinate system from BW [px] to gmapping [px]
path=path*res_BW-res_BW/2;                   % conversion from [px] to [m] in gmapping reference
path_GAZEBO=path-10;                         % conversion from gmapping [m] to GAZEBO [m] reference (gmapping origin at [-10,10] [m] (GAZEBO, from metadata))

end