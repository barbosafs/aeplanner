function [] = plotStats(filename)
%PLOTSTATS Summary of this function goes here
%   Detailed explanation goes here
stats = csvread(filename,1,0); %time, min, max, current, mean, router_distance
time = stats(:,1) - stats(1,1);
figure('units','normalized','outerposition',[0 0 1 1])
set(gca,'FontSize',16,'Fontname','Timesnewroman');
grid on;
if stats(2,2) == -1 && stats(3,3) == -1
    plot(time,stats(:,4),"b", time,stats(:,5),"r-.",'LineWidth',2)
    pbaspect([1 1 1])
    l = legend('current','mean','Location','NorthEast');
elseif stats(2,2) == -1
    plot(time,stats(:,3),"k-.",...
        time,stats(:,4),"g", time,stats(:,5),"r-.",'LineWidth',2)
    pbaspect([1 1 1])
    l = legend('max','current','mean','Location','NorthEast');
elseif stats(3,3) == -1
    plot(time,stats(:,2),"m:",...
        time,stats(:,4),"k", time,stats(:,5),"k-.",'LineWidth',2)
    pbaspect([1 1 1])
    l = legend('min','current','mean','Location','NorthEast');
else
    plot(time,stats(:,2),"m:", time,stats(:,3),"k-.",...
        time,stats(:,4),"b", time,stats(:,5),"r-.",'LineWidth',2)
    pbaspect([1 1 1])
    l = legend('min','max','current','mean','Location','NorthEast');
end
    
set(l,'Interpreter','Latex','Fontsize',16,'Fontname','Timesnewroman');    
xlabel('time ($s$)','Interpreter','latex'); 
ylabel('m','Interpreter','latex');

end

