classdef Map
    
    properties
        PointsIndices;
        xy;
        Densities;
        Edges;
    end
    methods
        %Constructor
        function obj = Map(x,y,density,Edges)
            obj.PointsIndices = [];
            obj.xy = [];
            obj.Densities = [];
            obj.Edges = Edges;
            obj = obj.CreatePoints(x,y,density);
        end
        
        function obj = CreatePoints(obj,x,y,density)
            for i = 1:length(x)
                arraylocation = length(obj.PointsIndices)+1;
                if arraylocation ==1
                    newindex =1;
                else
                    newindex = max(obj.PointsIndices)+1;
                end
                obj.PointsIndices(arraylocation) = newindex;
                obj.xy(arraylocation,:) = [x(i) y(i)];
                obj.Densities(1,arraylocation) = density(i);
            end
        end
        
%         function obj = AddPoints(obj,Points)
%             if iscell(Points) ==0 %change to cell array if Points is not cell array
%                 temp = Points;
%                 Points = {};
%                 Points{1} = temp;
%             end
%             for i = 1:length(Points)
%                 if ismember(Points{i}.Index,obj.PointsIndices) %if point is already in map, just update info
%                     index = ismember(obj.PointsIndices,Points{i}.Index);
%                     obj.Points{index} = Points{i};
%                 else %if point is not in map, add point to end of map
%                     obj.Points{length(obj.Points)+1} = Points{i};
%                     obj.PointsIndices(length(obj.PointsIndices)+1) = Points{i}.Index;
%                     obj.xy((size(obj.xy,1)+1),:) = [Points{i}.x Points{i}.y];
%                 end
%             end
%         end
        
        %deletePoints checked
%         function obj = DeletePoints(obj,Points) %enter indices of points to be deleted
%             index = ismember(obj.PointsIndices,Points);
%             obj.Points(index) = [];
%             obj.PointsIndices(index) = [];
%         end
        
        function obj = CreateEvenDensity(obj)
            %will create even density
            obj.Densities = ones(1,length(obj.PointsIndices));
            obj = obj.Normalize;
        end
        
        function obj = Normalize(obj)
            %normalize density
            obj.Densities = obj.Densities/sum(obj.Densities);
        end
        
        function obj = AddGaussian(obj, mu, sigma, weights)
            Val = mvnpdf(obj.xy,mu,sigma).';
            obj.Densities = weights(1)*obj.Densities + weights(2)*Val;
            obj = obj.Normalize;
        end
        
    end
end