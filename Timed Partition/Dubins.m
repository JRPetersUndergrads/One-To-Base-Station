classdef Dubins < AgentState
    
    properties
        MaxSpeed;
        MinSpeed;
        MaxAcceleration;
        MaxAngularSpeed;
        MinAngularSpeed;
        MaxAngularAcceleration;
        MinimumTurnRadius;
    end
    methods
        %Constructor
        function obj = Dubins(Location, Velocity, Dynamics, Mode, MinimumTurnRadius,  varargin)
            obj@AgentState(Location, Velocity, Dynamics, Mode,  varargin{:});
            p = inputParser;
            p.KeepUnmatched = 1;
            defaultMaxSpeed = {};
            defaultMinSpeed = 0;
            defaultMinAngularSpeed = 0;
            defaultMaxAcceleration = {};
            defaultMaxAngularSpeed = {};
            defaultMaxAngularAcceleration = {};
            addRequired(p,'MinimumTurnRadius');
            addOptional(p,'MinSpeed',defaultMinSpeed);
            addOptional(p, 'MaxSpeed',defaultMaxSpeed);
            addOptional(p, 'MaxAcceleration', defaultMaxAcceleration);
            addOptional(p, 'MaxAngularSpeed', defaultMaxAngularSpeed);
            addOptional(p,'MinAngularSpeed',defaultMinAngularSpeed);
            addOptional(p, 'MaxAngularAcceleration', defaultMaxAngularAcceleration);
            parse(p,MinimumTurnRadius, varargin{:});
            obj.MaxSpeed = p.Results.MaxSpeed;
            obj.MaxAcceleration = p.Results.MaxAcceleration;
            obj.MinimumTurnRadius = p.Results.MinimumTurnRadius;
            obj.MaxAngularSpeed = p.Results.MaxAngularSpeed;
            obj.MaxAngularAcceleration = p.Results.MaxAngularAcceleration;
            obj.MinSpeed = p.Results.MinSpeed;
            obj.MinAngularSpeed = p.Results.MinAngularSpeed;
        end
    end
end