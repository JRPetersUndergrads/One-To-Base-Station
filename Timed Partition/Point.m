classdef Point
    
    properties
        Index;
        x;
        y;
        Density;
        TimeActive; %for agent, number counts down to 0, for base station, tells how much time after time last update till change
        TimeLastUpdate; %for base station update
        MostRecentAgent; % for base station, give last agent's index that got this point
    end
    methods
        %Constructor
        function obj = Point(Index,x,y,Density,TimeActive)
            p = inputParser;
            p.KeepUnmatched = 1;
            addRequired(p,'Index');
            addRequired(p,'x');
            addRequired(p,'y');
            addRequired(p,'Density');
            addRequired(p,'TimeActive');
            parse(p,Index,x,y,Density,TimeActive)
            obj.Index = p.Results.Index;
            obj.x = p.Results.x;
            obj.y = p.Results.y;
            obj.Density = p.Results.Density;
            obj.TimeActive = p.Results.TimeActive;
        end
    end
end