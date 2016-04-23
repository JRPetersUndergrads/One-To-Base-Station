classdef AgentState
    
    properties
        Location;
        Attitude;
        Heading;
        Velocity;
        AngularVelocity;
        Mode;
        Dynamics;
    end
    methods
        %Constructor
        function obj = AgentState(Location, Velocity, Dynamics, Mode,  varargin)
            p = inputParser;
            p.KeepUnmatched = 1;
            defaultAttitude = [0,0,0]';
            defaultAngularVelocity = [0,0,0]';
            defaultHeading = Velocity/norm(Velocity);
            addRequired(p,'Location');
            addRequired(p,'Velocity');
            addRequired(p,'Dynamics');
            addRequired(p,'Mode');
            addOptional(p,'Attitude',defaultAttitude);
            addOptional(p,'AngularVelocity', defaultAngularVelocity);
            addOptional(p,'Heading', defaultHeading)
            parse(p,Location, Velocity, Dynamics, Mode,  varargin{:})
            obj.Location = p.Results.Location;
            obj.Velocity = p.Results.Velocity;
            obj.Dynamics = p.Results.Dynamics;
            obj.Mode = p.Results.Mode;
            obj.Attitude = p.Results.Attitude;
            obj.AngularVelocity = p.Results.AngularVelocity;
            obj.Heading = p.Results.Heading;
        end
        
        function value = PercentageInRect(obj, Rect, varargin)
            p = inputParser;
            p.KeepUnmatched = 1;
            defaultTimeWindow = NaN;
            addRequired(p,'Rect');
            addOptional(p, 'TimeWindow', defaultTimeWindow, @(x) isnumeric(x));
            parse(p,Rect, varargin{:})
            Rect = p.Results.Rect;
            TimeWindow = p.Results.TimeWindow;
            
            xv = [Rect(1), Rect(1)+Rect(3),Rect(1)+Rect(3),Rect(1),Rect(1)];
            yv = [Rect(2), Rect(2), Rect(2)+Rect(4), Rect(2)+Rect(4), Rect(2)];
            if isnan(TimeWindow);
                In = inpolygon(obj.Location(:, 1),obj.Location(:, 2), xv, yv);
            else
                In = inpolygon(obj.Location(end-TimeWindow+1:end, 1),obj.Location(end-TimeWindow+1:end, 2), xv, yv);
            end
            value = sum(In)/length(In);
        end
            
            
            
            
    end
end