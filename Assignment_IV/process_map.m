function [BW, G, start, goal] = process_map(png_file, n, m, diag)
    
    map_RGB=imread(png_file);     % RGB: (N,M,3) matrix
    map_HSV=rgb2hsv(map_RGB);   % HSV: (N,M,3) matrix

    N=size(map_HSV,1);          % original number of pixels
    M=size(map_HSV,2);          % original number of pixels

    H=map_HSV(:,:,1);           % extract hue values

    [start_i,start_j]=find(H>0.2 & H<0.5);       % coordinates in original image (green)
    [goal_i,goal_j]=find(H>0.9 | H<0.1 & H~=0);  % coordinates in original image (red)

    map_RGB(start_i,start_j,:)=255;   % set to white (free) the pixels of the starting positions
    map_RGB(goal_i,goal_j,:)=255;   % set to white (free) the pixels of the goal positions

    start_i=round(mean(start_i)*n/N);  % rescale coordinates
    start_j=round(mean(start_j)*m/M);
    goal_i=round(mean(goal_i)*n/N);
    goal_j=round(mean(goal_j)*m/M);

    start_i=min(max(start_i,1),n);     % make sure index is within the borders
    start_j=min(max(start_j,1),m);
    goal_i=min(max(goal_i,1),n);
    goal_j=min(max(goal_j,1),m);

    start=[start_j start_i];
    goal=[goal_j goal_i];

    % Create nodes
    BW_tresh=0.99;

    map_resized=imresize(map_RGB,[n m],'Method','nearest');     % resize to (n,m,3), with n=m=30
    map_BW=im2gray(map_resized);                                % convert in gray scale: (n,m) matrix
    map_bin=imbinarize(map_BW,BW_tresh);                        % logic: 1 if node (ii,jj) is free, 0 if obstacle; (n,m) matrix

    % Assign outputs

    BW=map_bin;
    
    G = -1*ones(size(BW, 1)*size(BW, 2)); % edge-matrix (starting)
    for i=1:size(BW,1)
        for j=1:size(BW,2)
            if BW(i,j)==1

                % itself
                G((i-1)*size(BW,1)+j,(i-1)*size(BW,1)+j)=0;

                % 4 directions
                if i+1 >0 && i+1<=size(BW,1) && BW(i+1,j)==1
                    G((i-1)*size(BW,1)+j,(i+1-1)*size(BW,1)+j)=1;
                end
                if i-1 >0 && i-1<=size(BW,1) && BW(i-1,j)==1
                    G((i-1)*size(BW,1)+j,(i-1-1)*size(BW,1)+j)=1;
                end
                if j+1 >0 && j+1<=size(BW,2) && BW(i,j+1)==1
                    G((i-1)*size(BW,1)+j,(i-1)*size(BW,1)+j+1)=1;
                end
                if j-1 >0 && j-1<=size(BW,2) && BW(i,j-1)==1
                    G((i-1)*size(BW,1)+j,(i-1)*size(BW,1)+j-1)=1;
                end

                % diagonal
                if i+1 <= size(BW,1) && j+1 <= size(BW,2) && BW(i+1, j+1) == 1 && diag==true
                    G((i-1)*size(BW,2)+j, (i)*size(BW,2)+j+1) = 1;
                end
                if i-1 > 0 && j+1 <= size(BW,2) && BW(i-1, j+1) == 1 && diag==true
                    G((i-1)*size(BW,2)+j, (i-2)*size(BW,2)+j+1) = 1;
                end
                if i+1 <= size(BW,1) && j-1 > 0 && BW(i+1, j-1) == 1 && diag==true
                    G((i-1)*size(BW,2)+j, (i)*size(BW,2)+j-1) = 1;
                end
                if i-1 > 0 && j-1 > 0 && BW(i-1, j-1) == 1 && diag==true
                    G((i-1)*size(BW,2)+j, (i-2)*size(BW,2)+j-1) = 1;
                end
            end
        end
    end
end