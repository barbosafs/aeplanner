function [] = plotPose3(filename,environment)
%PLOTPOSE3 Summary of this function goes here
%   Detailed explanation goes here
pose = csvread(filename,1,0); %time, x, y, z, yaw
%Ploting 3D
figure()
hold on
plotEnv3(environment)
plot3(pose(:,2),pose(:,3),pose(:,4),"k",'LineWidth',2)
xlabel('$x$','Interpreter','latex'); 
ylabel('$y$','Interpreter','latex');
zlabel('$z$','Interpreter','latex'); 
set(gca,'FontSize',16,'Fontname','Timesnewroman');
end

