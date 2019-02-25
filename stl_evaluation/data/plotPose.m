function [] = plotPose(filename,environment)
%PLOTPOSE Summary of this function goes here
%   Detailed explanation goes here
pose = csvread(filename,1,0); %time, x, y, z, yaw
%Ploting 2D
figure('units','normalized','outerposition',[0 0 1 1])
hold on
plotEnv(environment)
plot(pose(:,2),pose(:,3),"r",'LineWidth',2)
xlabel('$x$','Interpreter','latex'); 
ylabel('$y$','Interpreter','latex');
set(gca,'FontSize',16,'Fontname','Timesnewroman');
% l = legend(p1,{'$x_1$','$x_2$'});
% set(l,'Interpreter','Latex','Fontsize',16,'Fontname','Timesnewroman');
end

