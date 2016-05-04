classdef Point
    
    properties
        Index;
        x;
        y;
        Density;
        MostRecentAgent; % for base station, give last agent's index that got this point
    end
    methods
        %Constructor
        function obj = Point(Index,x,y,Density)
            p = inputParser;
            p.KeepUnmatched = 1;
            addRequired(p,'Index');
            addRequired(p,'x');
            addRequired(p,'y');
            addRequired(p,'Density');
            parse(p,Index,x,y,Density)
            obj.Index = p.Results.Index;
            obj.x = p.Results.x;
            obj.y = p.Results.y;
            obj.Density = p.Results.Density;
        end
    end
end