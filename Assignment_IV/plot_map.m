function plot_map(BW)

figure()
hold on
for i=1:size(BW,1)
    for j=1:size(BW,2)
        if BW(i,j)==1
            plot(j,i,'oc', 'MarkerFaceColor','c') 
        else
            plot(j,i,'ok','MarkerFaceColor','k')
        end
    end
end

axis ij