function plot_runtime(act_node,BW)
    dr_i=0;
    dr_j=act_node;
    while size(BW,1)<dr_j
        dr_i=dr_i+1;
        dr_j=dr_j-size(BW,1);
    end
    dr_i=(dr_i)+1;
    plot(dr_j,dr_i,'xb');
    drawnow
end