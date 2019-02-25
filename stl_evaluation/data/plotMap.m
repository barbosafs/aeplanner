function [] = plotMap(filename)
%PLOTMAP Summary of this function goes here
%   Detailed explanation goes here
figure('units','normalized','outerposition',[0 0 1 1])
grid on
map = csvread(filename,1,0); %time, total volume, current volume
time_map = map(:,1) - map(1,1)
plot(time_map,map(:,3),"r:", time_map, map(:,2), "b", 'LineWidth',2);
pbaspect([1 1 1])
xlabel('time ($s$)','Interpreter','latex'); 
ylabel('$m^3$','Interpreter','latex');
set(gca,'FontSize',16,'Fontname','Timesnewroman');
% l = legend('min','max','current','mean');
% set(l,'Interpreter','Latex','Fontsize',16,'Fontname','Timesnewroman');

end

