classdef HelperUtils
%HELPERUTILS Helper utilies for DynamicReplanningOnAnIndoorMapExample

% Copyright 2019 The MathWorks, Inc.

    methods
        %%
        % *|updateWorldMap|*
        %
        % Update the map, robot, ranges, and path traveled.
        function updateWorldMap(~, ...
                robotPatch, rangesLine, traveledLine, ...
                pose, ranges, angles, scale)
            if nargin < 8
                scale = 2;
            end
            
            % Update robot position.
            t = pose(1:2);
            a = pose(3);
            R = [cos(a) -sin(a); sin(a) cos(a)];
            G = [scale*R t(:); 0 0 1];
            
            RobotBody = getRobotBodyInfo;
            RobotBodyTriangleVertices = G*RobotBody.Vertices;
            
            robotPatch.XData = RobotBodyTriangleVertices(1,:);
            robotPatch.YData = RobotBodyTriangleVertices(2,:);
            
            % Update range line info.
            rangesLine.XData(1:3:end) = pose(1);
            rangesLine.XData(2:3:end) = pose(1) ...
                + ranges.*cos(angles + pose(3));
            rangesLine.YData(1:3:end) = pose(2);
            rangesLine.YData(2:3:end) = pose(2) ...
                + ranges.*sin(angles + pose(3));
            
            % Updated traveled line.
            set(traveledLine, 'XData', [traveledLine.XData, pose(1,1)], ...
                'YData', [traveledLine.YData, pose(1,2)]);
        end
        
        %%
        % *|updateRoute|*
        %
        % Set the previous route to a less prominent line and plot the new 
        % route.
        function routeLine = updateRoute(obj, routeLine, newRoute)
            routeLine.LineStyle = ':';
            routeLine = plotRoute(obj, routeLine.Parent, newRoute, ...
                routeLine.Color);
        end
        
        %%
        % *|plotRoute|*
        %
        % Plot the route.
        function routeLine = plotRoute(~, ax, route, color)
            x = route(:,1);
            y = route(:,2);
            if nargin < 4
                routeLine = line(ax, 'XData', x, 'YData', y);
            else
                routeLine = line(ax, 'XData', x, 'YData', y, ...
                    'Color', color);
            end
        end
        
        %%
        % *|plotScan|*
        %
        % Plot the range scan.
        function rangesLine = plotScan(~, ax, pose, ranges, angles)
            x = repmat(pose(1),3*length(ranges),1);
            y = repmat(pose(2),3*length(ranges),1);
            x(3:3:end) = NaN;
            y(3:3:end) = NaN;
            x(2:3:end) = pose(1) + ranges.*cos(angles + pose(3));
            y(2:3:end) = pose(2) + ranges.*sin(angles + pose(3));
            rangesLine = plot(x,y,'Parent',ax);
        end
        
        %%
        % *|plotRobot|*
        %
        % Plot the robot.
        function robotPatch = plotRobot(~, ax, pose, scale)
            if nargin < 3
                scale = 2;
            end
            
            % Set robot position.
            t = pose(1:2);
            a = pose(3);
            R = [cos(a) -sin(a); sin(a) cos(a)];
            G = [scale*R t(:); 0 0 1];
            
            RobotBody = getRobotBodyInfo;
            RobotBodyTriangleVertices = G*RobotBody.Vertices;
            RobotBodyFaceColor = RobotBody.FaceColor;
            
            robotPatch = patch(ax, RobotBodyTriangleVertices(1,:), ...
                RobotBodyTriangleVertices(2,:), RobotBodyFaceColor,'HandleVisibility','off');
        end
        
    end
    
end

%%
% *|getRobotBodyInfo|*
%
% Return the robot shape and color.
function info = getRobotBodyInfo

info.Vertices = [[[-0.3, -0.05,-0.3,0.8]; [-0.5,0,0.5,0]]; ones(1,4)];
info.FaceColor = [0.866 0.918 0.753];
end