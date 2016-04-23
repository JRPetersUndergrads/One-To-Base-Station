classdef Map
    
    properties
        Points;
        PointsIndices;
    end
    methods
        %Constructor
        function obj = Map(Points)
            p = inputParser;
            p.KeepUnmatched = 1;
            addRequired(p,'Points');
            parse(p,Points)
            obj.Points ={};
            obj.PointsIndices = [];
            obj = obj.AddPoints(Points);
        end
        
        function obj = AddPoints(obj,Points)
            if iscell(Points) ==0 %change to cell array if Points is not cell array
                temp = Points;
                Points = {};
                Points{1} = temp;
            end
            for i = 1:length(Points)
                if ismember(Points{i}.Index,obj.PointsIndices) %if point is already in map, just update info
                    index = ismember(obj.PointsIndices,Points{i}.Index);
                    obj.Points{index} = Points{i};
                else %if point is not in map, add point to end of map
                    obj.Points{length(obj.Points)+1} = Points{i};
                    obj.PointsIndices(length(obj.PointsIndices)+1) = Points{i}.Index;
                end
            end
        end
        %deletePoints checked
        function obj = DeletePoints(obj,Points) %enter indices of points to be deleted
            index = ismember(obj.PointsIndices,Points);
            obj.Points(index) = [];
            obj.PointsIndices(index) = [];
        end
        
        
        function obj = CreatePoints(obj,X,Y,Density,Timer)
            for i = 1:length(X)
                arraylocation = length(obj.PointsIndices)+1;
                if arraylocation ==1
                    newindex =1;
                else
                    newindex = max(obj.PointsIndices)+1;
                end
                obj.PointsIndices(arraylocation) = newindex;
                obj.Points{arraylocation} = Point(newindex,X(i),Y(i),Density(i),Timer(i));
            end
        end
        
        function obj = CreateEvenDensity(obj)
            %will create even density
            for i = 1:length(obj.Points)
                obj.Points{i}.Density = 1;
            end
            obj = obj.Normalize;
        end
        
        function obj = AddGaussian(obj, mu, sigma, weights)
            PointLocation = [];
            for i = 1: length(obj.Points)
                PointLocation(i,1) = obj.Points{i}.x;
                PointLocation(i,2) = obj.Points{i}.y;
            end
            Val = mvnpdf(PointLocation,mu,sigma);
            for i = 1:length(obj.Points)
                obj.Points{i}.Density = weights(1)*obj.Points{i}.Density + weights(2)*Val(i);
            end
            obj = obj.Normalize;
        end
        
        function obj = Normalize(obj)
            %normalize density
            temp = zeros(length(obj.Points));
            for i = 1:length(obj.Points)
                temp(i) = obj.Points{i}.Density;
            end
            total = sum(temp);
            temp = temp/total;
            for i = 1:length(obj.Points)
                obj.Points{i}.Density = temp(i);
            end
        end
        
    end
end