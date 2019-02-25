function [] = plotEnv3(env)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
switch env
    case 'aep_office'
        outer_walls = [-27 -3;
            -27, 17;
            3, 17;
            3, -3;
            -27, -3
            ];
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
        
        case 'aep_apartment'
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
            
        case 'garage'
            % Walls
            wall_height = 2.5;
            wall_alpha = 0.5;
            
            % Outer walls
            points = lineToPlane([-10 24], [-3 -3], [0 wall_height]);
            w = fill3(points(:,1),points(:,2),points(:,3),'k');
            alpha(w, wall_alpha)
            points = lineToPlane([-10 -10], [-3 30.5], [0 wall_height]);
            w = fill3(points(:,1),points(:,2),points(:,3),'k');
            alpha(w, wall_alpha)
            points = lineToPlane([-10 24], [30.5 30.5], [0 wall_height]);
            w = fill3(points(:,1),points(:,2),points(:,3),'k');
            alpha(w, wall_alpha)
            points = lineToPlane([24 24], [-3 30.5], [0 wall_height]);
            w = fill3(points(:,1),points(:,2),points(:,3),'k');
            alpha(w, wall_alpha)
            
            % Pillars
            points = lineToPlane([-5.5 -4.5], [3 3], [0 wall_height]);
            w = fill3(points(:,1),points(:,2),points(:,3),'k');
            alpha(w, wall_alpha)
            points = lineToPlane([-5.5 -4.5], [4 4], [0 wall_height]);
            w = fill3(points(:,1),points(:,2),points(:,3),'k');
            alpha(w, wall_alpha)
            points = lineToPlane([-5.5 -5.5], [3 4], [0 wall_height]);
            w = fill3(points(:,1),points(:,2),points(:,3),'k');
            alpha(w, wall_alpha)
            points = lineToPlane([-4.5 -4.5], [3 4], [0 wall_height]);
            w = fill3(points(:,1),points(:,2),points(:,3),'k');
            alpha(w, wall_alpha)
            
%             rectangle('Position',[-5.5 3 1 1],'FaceColor','k','EdgeColor','k', 'LineWidth', thickness)
%             rectangle('Position',[-5.5 10 1 1],'FaceColor','k','EdgeColor','k', 'LineWidth', thickness)
%             rectangle('Position',[-5.5 17 1 1],'FaceColor','k','EdgeColor','k', 'LineWidth', thickness)
%             rectangle('Position',[-5.5 24 1 1],'FaceColor','k','EdgeColor','k', 'LineWidth', thickness)
%             
%             rectangle('Position',[6.5 3 1 1],'FaceColor','k','EdgeColor','k', 'LineWidth', thickness)
%             rectangle('Position',[6.5 10 1 1],'FaceColor','k','EdgeColor','k', 'LineWidth', thickness)
%             rectangle('Position',[6.5 17 1 1],'FaceColor','k','EdgeColor','k', 'LineWidth', thickness)
%             rectangle('Position',[6.5 24 1 1],'FaceColor','k','EdgeColor','k', 'LineWidth', thickness)
%             
%             rectangle('Position',[18.5 3 1 1],'FaceColor','k','EdgeColor','k', 'LineWidth', thickness)
%             rectangle('Position',[18.5 10 1 1],'FaceColor','k','EdgeColor','k', 'LineWidth', thickness)
%             rectangle('Position',[18.5 17 1 1],'FaceColor','k','EdgeColor','k', 'LineWidth', thickness)
%             rectangle('Position',[18.5 24 1 1],'FaceColor','k','EdgeColor','k', 'LineWidth', thickness)
            
            % Routers
%             router_height = 0.1;
%             router_alpha = 1;
%             router_half_length = 0.1;
%             points = [(22.9929-router_half_length) (-1.89837-router_half_length) 0;
%                       (22.9929+router_half_length) (-1.89837-router_half_length) 0;
%                       (22.9929+router_half_length) (-1.89837-router_half_length) router_height;
%                       (22.9929-router_half_length) (-1.89837-router_half_length) router_height
%                      ];
%             r = fill3(points(:,1),points(:,2),points(:,3),'b');
%             alpha(r, router_alpha)
%             points = [(22.9929-router_half_length) (-1.89837-router_half_length) 0;
%                       (22.9929-router_half_length) (-1.89837+router_half_length) 0;
%                       (22.9929-router_half_length) (-1.89837+router_half_length) router_height;
%                       (22.9929-router_half_length) (-1.89837-router_half_length) router_height
%                      ];
%             r = fill3(points(:,1),points(:,2),points(:,3),'b');
%             alpha(r, router_alpha)
%             points = [(22.9929-router_half_length) (-1.89837+router_half_length) 0;
%                       (22.9929+router_half_length) (-1.89837+router_half_length) 0;
%                       (22.9929+router_half_length) (-1.89837+router_half_length) router_height;
%                       (22.9929-router_half_length) (-1.89837+router_half_length) router_height
%                      ];
%             r = fill3(points(:,1),points(:,2),points(:,3),'b');
%             alpha(r, router_alpha)
%             points = [(22.9929+router_half_length) (-1.89837-router_half_length) 0;
%                       (22.9929+router_half_length) (-1.89837+router_half_length) 0;
%                       (22.9929+router_half_length) (-1.89837+router_half_length) router_height;
%                       (22.9929+router_half_length) (-1.89837-router_half_length) router_height
%                      ];
%             r = fill3(points(:,1),points(:,2),points(:,3),'b');
%             alpha(r, router_alpha)
            
            xlim([-11 25])
            ylim([-4 31.5])
            zlim([-1 3.5])
            
end
end