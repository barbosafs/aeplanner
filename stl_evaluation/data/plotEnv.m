function [] = plotEnv(env)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
switch env
    case 'aep_office'
        thickness = 1;
        
        rectangle('Position',[-23.5 -3 30 20],'EdgeColor','k', 'LineWidth', thickness)
        
        plot([4.5, 6.5],[10.5, 10.5], "k", 'LineWidth', thickness)
        plot([1.5, 0],[10.5, 10.5], "k", 'LineWidth', thickness)
        plot([0, -2.8],[10.5, 13.12], "k", 'LineWidth', thickness)
        plot([-2.8, -3.07],[13.12, 12.86], "k", 'LineWidth', thickness)
        plot([0, -7.08],[10.5, 3.45], "k", 'LineWidth', thickness)
        plot([-7.08, -7.08],[3.45, -3], "k", 'LineWidth', thickness)
        plot([-9.96, -9.96],[-3, -1.56], "k", 'LineWidth', thickness)
        plot([-9.96, -10.58],[-1.56, -0.94], "k", 'LineWidth', thickness)
        plot([-19.37, -19.37]+3.5,[-3, 1.75], "k", 'LineWidth', thickness)
        plot([-21.73, -16.53]+3.5,[1.75, 1.75], "k", 'LineWidth', thickness)
        plot([-16.53, -15.9]+3.5,[1.75, 1.16], "k", 'LineWidth', thickness)
        plot([-21.73, -21.73]+3.5,[1.75, 1.45], "k", 'LineWidth', thickness)
        plot([-21.73, -21.73]+3.5,[-0.15, -1], "k", 'LineWidth', thickness)
        plot([-21.73, -19.37]+3.5,[-0.55, -0.55], "k", 'LineWidth', thickness)
        plot([-21.73, -21.73]+3.5,[-3, -2.55], "k", 'LineWidth', thickness)
        plot([-24.58, -24.58]+3.5,[-3, -2.55], "k", 'LineWidth', thickness)
        plot([-24.58, -24.58]+3.5,[-0.15, -1], "k", 'LineWidth', thickness)
        plot([-24.58, -27]+3.5,[-0.55, -0.55], "k", 'LineWidth', thickness)
        plot([-27, -24.58]+3.5,[1.75, 1.75], "k", 'LineWidth', thickness)
        plot([-24.58, -24.58]+3.5,[1.75, 1.45], "k", 'LineWidth', thickness)
        plot([-27, -20.33]+3.5,[5.58, 5.58], "k", 'LineWidth', thickness)
        plot([-20.33, -19.37]+3.5,[5.58, 4.61], "k", 'LineWidth', thickness)
        
        plot([-20.76, -19.62, -23.98, -23.98, -23.02, -22.77]+3.5,[9.66, 8.53, 8.53, 10.9, 11.83, 11.58], "k", 'LineWidth', thickness)
        
        plot([-20.90 -14.61, -14.65]+3.5,[13.97, 7.69, 7.03], "k", 'LineWidth', thickness)
        plot([-15.2 -12.6, -13.17]+3.5,[8.31, 10.6, 10.85], "k", 'LineWidth', thickness)
        plot([-15.19, -15.6, -17.93]+3.5,[12.84, 13.28, 10.97], "k", 'LineWidth', thickness)
        
        plot([-15.17, -13.86]+3.5,[17, 15.58], "k", 'LineWidth', thickness)
        plot([-11.86, -10.89, -8.36, -10.08]+3.5,[13.58, 12.62, 15.17, 17], "k", 'LineWidth', thickness)
        
        plot([-6.35, -9.05, -8.6]+3.5,[7.7, 10.41, 10.87], "k", 'LineWidth', thickness)
        plot([-9.05, -9.55]+3.5,[10.41, 9.94], "k", 'LineWidth', thickness)
        plot([-11.53, -11.77, -9.06]+3.5,[7.94, 7.7, 4.97], "k", 'LineWidth', thickness)
        
        xlim([-24.5 7.5])
        ylim([-4 18])
        pbaspect([1 1 1])
        
        case 'aep_apartment'
            thickness = 1;

            rectangle('Position',[-10 -5 20 10],'EdgeColor','k', 'LineWidth', thickness)
            plot([0, 0],[-5, -2], "k", 'LineWidth', thickness)
            plot([-4, 4],[-2, -2], "k", 'LineWidth', thickness)
            plot([-7, -7],[-2, 5], "k", 'LineWidth', thickness)
            plot([7, 7, 0],[5, 2, 2], "k", 'LineWidth', thickness)
            
            xlim([-11 11])
            ylim([-6 6])
            pbaspect([1 1 1])
            
        case 'garage'
            % Routers range
            range = 17;
            alpha = 0.25;
            pos = [(22.9929-range) (-1.89837-range) (range*2) (range*2)]; 
            rectangle('Position',pos,'Curvature',[1 1],'FaceColor',[0 1 0 alpha],'EdgeColor','g', 'LineWidth', 0.1)
            pos = [(-9.45421-range) (-2.53462-range) (range*2) (range*2)]; 
            rectangle('Position',pos,'Curvature',[1 1],'FaceColor',[0 1 0 alpha],'EdgeColor','g', 'LineWidth', 0.1)
            pos = [(-9.26742-range) (29.9073-range) (range*2) (range*2)]; 
            rectangle('Position',pos,'Curvature',[1 1],'FaceColor',[0 1 0 alpha],'EdgeColor','g', 'LineWidth', 0.1)
            pos = [(23.2973-range) (29.7449-range) (range*2) (range*2)]; 
            rectangle('Position',pos,'Curvature',[1 1],'FaceColor',[0 1 0 alpha],'EdgeColor','g', 'LineWidth', 0.1)
            
            % Walls
            thickness = 1;
            rectangle('Position',[-10 -3 34 33.5],'EdgeColor','k', 'LineWidth', thickness)
            
            rectangle('Position',[-5.5 3 1 1],'FaceColor','k','EdgeColor','k', 'LineWidth', thickness)
            rectangle('Position',[-5.5 10 1 1],'FaceColor','k','EdgeColor','k', 'LineWidth', thickness)
            rectangle('Position',[-5.5 17 1 1],'FaceColor','k','EdgeColor','k', 'LineWidth', thickness)
            rectangle('Position',[-5.5 24 1 1],'FaceColor','k','EdgeColor','k', 'LineWidth', thickness)
            
            rectangle('Position',[6.5 3 1 1],'FaceColor','k','EdgeColor','k', 'LineWidth', thickness)
            rectangle('Position',[6.5 10 1 1],'FaceColor','k','EdgeColor','k', 'LineWidth', thickness)
            rectangle('Position',[6.5 17 1 1],'FaceColor','k','EdgeColor','k', 'LineWidth', thickness)
            rectangle('Position',[6.5 24 1 1],'FaceColor','k','EdgeColor','k', 'LineWidth', thickness)
            
            rectangle('Position',[18.5 3 1 1],'FaceColor','k','EdgeColor','k', 'LineWidth', thickness)
            rectangle('Position',[18.5 10 1 1],'FaceColor','k','EdgeColor','k', 'LineWidth', thickness)
            rectangle('Position',[18.5 17 1 1],'FaceColor','k','EdgeColor','k', 'LineWidth', thickness)
            rectangle('Position',[18.5 24 1 1],'FaceColor','k','EdgeColor','k', 'LineWidth', thickness)
            
            
            % Routers
            rectangle('Position',[(22.9929-0.1) (-1.89837-0.1) 0.2 0.2],'FaceColor','b','EdgeColor','b', 'LineWidth', 0.1)
            rectangle('Position',[(-9.45421-0.1) (-2.53462-0.1) 0.2 0.2],'FaceColor','b','EdgeColor','b', 'LineWidth', 0.1)
            rectangle('Position',[(-9.26742-0.1) (29.9073-0.1) 0.2 0.2],'FaceColor','b','EdgeColor','b', 'LineWidth', 0.1)
            rectangle('Position',[(23.2973-0.1) (29.7449-0.1) 0.2 0.2],'FaceColor','b','EdgeColor','b', 'LineWidth', 0.1)
            
            
            xlim([-11 25])
            ylim([-4 31.5])
            pbaspect([1 1 1])
            
        case 'triangle_small'
            % Walls
            thickness = 1;
            outer_walls = [-9.99 10.5;
                           0.03, -3.05;
                           21.59, 9.35;
                           55.04 9.32;
                           76.59 -3.05;
                           86.57 14.24;
                           65.05 26.73;
                           48.32 55.67;
                           48.32 80.57;
                           28.32 80.58;
                           28.28 55.65;
                           11.56 26.73;
                           -9.99 10.5
                          ];
            plot(outer_walls(:,1), outer_walls(:,2), "k", 'LineWidth', thickness)
            plot([28.3, 37.8],[70.58, 70.58], "k", 'LineWidth', thickness)
            plot([38.8, 48.32],[70.58, 70.58], "k", 'LineWidth', thickness)
            
            xlim([-11 88])
            ylim([-4 81])
            pbaspect([1 1 1])
          
        case 'empty_room_big'
            % Walls
            thickness = 1;
            rectangle('Position',[-38 -2 40 30],'EdgeColor','k', 'LineWidth', thickness)
            
            xlim([-39 3])
            ylim([-3 29])
            pbaspect([1 1 1])
            
end
end