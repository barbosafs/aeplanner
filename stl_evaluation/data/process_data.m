clear all; close all; clc;
pause on % to enable pause function

path = '/home/dduberg/ros_ws/catkin_ws/src/fernando/aeplanner/stl_evaluation/data/';
environment = 'office';
experiment = 'aep_real_bbx_3';

print_what = [1 0 0];
print_what = [0 1 0];
% print_what = [0 0 1];

% When saving figure stuff
set(gcf, 'PaperUnits','centimeters');
set(gcf, 'Units','centimeters');
pos=get(gcf,'Position');
set(gcf, 'PaperSize', [pos(3) pos(4)]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition',[0 0 pos(3) pos(4)]);

% Pose
if print_what(1) == 1
    file_name = [path environment '/' experiment '/pose'];
    plotPose([file_name '.txt'], environment)
    print('-depsc2', file_name);
    saveas(gcf,[file_name '.eps'],'epsc');
    system(['epstopdf',' ',[file_name '.eps']]);
    system(['pdfcrop',' ',[file_name,'.pdf']]);
    system(['mv',' ',[file_name,'-crop.pdf'],' ',[file_name,'.pdf']]);
    system(['rm', ' ', [file_name '.eps']]);
end


% Stats
if print_what(2) == 1
    file_name = [path environment '/' experiment '/stats'];
    plotStats([file_name '.txt'])
    print('-depsc2', file_name);
    saveas(gcf,[file_name '.eps'],'epsc');
    system(['epstopdf',' ',[file_name '.eps']]);
    system(['pdfcrop',' ',[file_name,'.pdf']]);
    system(['mv',' ',[file_name,'-crop.pdf'],' ',[file_name,'.pdf']]);
    system(['rm', ' ', [file_name '.eps']]);
end

% Map
if print_what(3) == 1
    file_name = [strcat(path, environment, '/aep_0/map.txt'); strcat(path, environment, '/stl_0/map.txt')];
    
    figure('units','normalized','outerposition',[0 0 1 1])
    grid on
    hold on
    
    map = csvread(strcat(path, environment, '/aep_0/map.txt'),1,0); %time, total volume, current volume
    time_map = map(:,1) - map(1,1);
    plot(time_map, map(:,2), "k:", time_map,map(:,3),"m", 'LineWidth',2);
    
%     map = csvread(strcat(path, environment, '/aep_real_bbx_1/map.txt'),1,0); %time, total volume, current volume
%     time_map = map(:,1) - map(1,1);
%     plot(time_map,map(:,3),"b", 'LineWidth',2);
    
    map = csvread(strcat(path, environment, '/stl_0/map.txt'),1,0); %time, total volume, current volume
    time_map = map(:,1) - map(1,1);
    plot(time_map,map(:,3),"c", 'LineWidth',2);
    
    pbaspect([1 1 1])
    xlabel('time ($s$)','Interpreter','latex'); 
    ylabel('$m^3$','Interpreter','latex');
    set(gca,'FontSize',16,'Fontname','Timesnewroman');
    
    l = legend('Total','AEP','STL','Location','SouthEast');
    
    set(l,'Interpreter','Latex','Fontsize',16,'Fontname','Timesnewroman');
    
    file_name = strcat(path, environment, '/map_1');
    print('-depsc2', file_name);
    saveas(gcf,[file_name '.eps'],'epsc');
    system(['epstopdf',' ',[file_name '.eps']]);
    system(['pdfcrop',' ',[file_name,'.pdf']]);
    system(['mv',' ',[file_name,'-crop.pdf'],' ',[file_name,'.pdf']]);
    system(['rm', ' ', [file_name '.eps']]);
end
%plotPose3([environment '/' experiment '/pose.txt'], environment)
