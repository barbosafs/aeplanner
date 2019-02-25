function [points] = lineToPlane(x,y,z)
%LINETOPLANE Summary of this function goes here
%   Detailed explanation goes here
 points = [x(1) y(1) z(1);
           x(2) y(2) z(1);
           x(2) y(2) z(2);
           x(1) y(1) z(2)
          ];
end

