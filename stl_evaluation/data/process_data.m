clear all; close all; clc;
%%
filename = 'stl_0_map.txt';
map = csvread(filename,1,0); %time, total volume, current volume
time_map = map(:,1) - map(1,1);
figure()
plot(time_map,map(:,3),"r:", time_map, map(:,2), "b -.", 'LineWidth',2); grid on
xlabel('time ($s$)','Interpreter','latex'); 
ylabel('$m^3$','Interpreter','latex');
set(gca,'FontSize',16,'Fontname','Timesnewroman');
% l = legend('min','max','current','mean');
% set(l,'Interpreter','Latex','Fontsize',16,'Fontname','Timesnewroman');

%%
filename = 'stl_0_pose.txt';
pose = csvread(filename,1,0); %time, x, y, z, yaw
%Ploting 2D
figure()
plot(pose(:,2),pose(:,3),"k",'LineWidth',2)
xlabel('$x$','Interpreter','latex'); 
ylabel('$y$','Interpreter','latex');
set(gca,'FontSize',16,'Fontname','Timesnewroman');
% l = legend(p1,{'$x_1$','$x_2$'});
% set(l,'Interpreter','Latex','Fontsize',16,'Fontname','Timesnewroman');
hold on
plotEnv('aep_office');

%Ploting 3D
figure
plot3(pose(:,2),pose(:,3),pose(:,4),"k",'LineWidth',2)
xlabel('$x$','Interpreter','latex'); 
ylabel('$y$','Interpreter','latex');
zlabel('$z$','Interpreter','latex'); 
set(gca,'FontSize',16,'Fontname','Timesnewroman');

%%
filename = 'stl_0_stats.txt';
stats = csvread(filename,1,0); %time, min, max, current, mean
time = stats(:,1) - stats(1,1);
figure()
plot(time,stats(:,2),"r:", time,stats(:,3),"r-.",...
    time,stats(:,4),"k", time,stats(:,5),"k-.",'LineWidth',2)
grid on;
xlabel('time ($s$)','Interpreter','latex'); 
ylabel('m','Interpreter','latex');
set(gca,'FontSize',16,'Fontname','Timesnewroman');
l = legend('min','max','current','mean');
set(l,'Interpreter','Latex','Fontsize',16,'Fontname','Timesnewroman');

%%
% AEP Apartment
clear all; close all; clc;
outer_walls = [-10 -5;
               -10, 5;
                10, 5;
                10, -5;
               -10, -5
              ];
          
figure()
hold on
plot(outer_walls(:,1), outer_walls(:,2), "k", 'LineWidth', 0.15)
plot([0, 0],[-5, -2], "k", 'LineWidth', 0.15)
plot([-4, 4],[-2, -2], "k", 'LineWidth', 0.15)
plot([-7, -7],[-2, 5], "k", 'LineWidth', 0.15)
plot([7, 7, 0],[5, 2, 2], "k", 'LineWidth', 0.15)
axis([-11 11 -11 11])

%%
% AEP Office
clear all; close all; clc;
outer_walls = [-27 -3;
               -27, 17;
                3, 17;
                3, -3;
               -27, -3
              ];

figure()
hold on
plot(outer_walls(:,1), outer_walls(:,2), "k", 'LineWidth', 0.15)
plot([1, 3],[10.5, 10.5], "k", 'LineWidth', 0.15)
plot([-2, -3.5],[10.5, 10.5], "k", 'LineWidth', 0.15)
plot([-3.5, -6.3],[10.5, 13.12], "k", 'LineWidth', 0.15)
plot([-6.3, -6.57],[13.12, 12.86], "k", 'LineWidth', 0.15)
plot([-3.5, -10.58],[10.5, 3.45], "k", 'LineWidth', 0.15)
plot([-10.58, -10.58],[3.45, -3], "k", 'LineWidth', 0.15)
plot([-13.46, -13.46],[-3, -1.56], "k", 'LineWidth', 0.15)
plot([-13.46, -14.08],[-1.56, -0.94], "k", 'LineWidth', 0.15)
plot([-19.37, -19.37],[-3, 1.75], "k", 'LineWidth', 0.15)
plot([-21.73, -16.53],[1.75, 1.75], "k", 'LineWidth', 0.15)
plot([-16.53, -15.9],[1.75, 1.16], "k", 'LineWidth', 0.15)
plot([-21.73, -21.73],[1.75, 1.45], "k", 'LineWidth', 0.15)
plot([-21.73, -21.73],[-0.15, -1], "k", 'LineWidth', 0.15)
plot([-21.73, -19.37],[-0.55, -0.55], "k", 'LineWidth', 0.15)
plot([-21.73, -21.73],[-3, -2.55], "k", 'LineWidth', 0.15)
plot([-24.58, -24.58],[-3, -2.55], "k", 'LineWidth', 0.15)
plot([-24.58, -24.58],[-0.15, -1], "k", 'LineWidth', 0.15)
plot([-24.58, -27],[-0.55, -0.55], "k", 'LineWidth', 0.15)
plot([-27, -24.58],[1.75, 1.75], "k", 'LineWidth', 0.15)
plot([-24.58, -24.58],[1.75, 1.45], "k", 'LineWidth', 0.15)
plot([-27, -20.33],[5.58, 5.58], "k", 'LineWidth', 0.15)
plot([-20.33, -19.37],[5.58, 4.61], "k", 'LineWidth', 0.15)

plot([-20.76, -19.62, -23.98, -23.98, -23.02, -22.77],[9.66, 8.53, 8.53, 10.9, 11.83, 11.58], "k", 'LineWidth', 0.15)

plot([-20.90 -14.61, -14.65],[13.97, 7.69, 7.03], "k", 'LineWidth', 0.15)
plot([-15.2 -12.6, -13.17],[8.31, 10.6, 10.85], "k", 'LineWidth', 0.15)
plot([-15.19, -15.6, -17.93],[12.84, 13.28, 10.97], "k", 'LineWidth', 0.15)

plot([-15.17, -13.86],[17, 15.58], "k", 'LineWidth', 0.15)
plot([-11.86, -10.89, -8.36, -10.08],[13.58, 12.62, 15.17, 17], "k", 'LineWidth', 0.15)

plot([-6.35, -9.05, -8.6],[7.7, 10.41, 10.87], "k", 'LineWidth', 0.15)
plot([-9.05, -9.55],[10.41, 9.94], "k", 'LineWidth', 0.15)
plot([-11.53, -11.77, -9.06],[7.94, 7.7, 4.97], "k", 'LineWidth', 0.15)

axis([-28 4 -4 18])
pbaspect([1 1 1])

%%
% AEP Maze

%%
% Office

%%
% Garage

%%
% Triangle

%%
% Power plant
