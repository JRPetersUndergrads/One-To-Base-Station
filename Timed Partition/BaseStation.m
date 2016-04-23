classdef BaseStation
    
    properties
        Map;
        TimeOfUpdates;
        AgentWeights;
        Distances;
        Coverings;
        Centers;
        DeltaComm;
        DeltaHold;
        Indices
    end
    methods
        %Constructor
        function obj = BaseStation(Map,AgentWeights,DeltaComm,DeltaHold)
            p = inputParser;
            p.KeepUnmatched = 1;
            addRequired(p,'Map');
            addRequired(p,'AgentWeights');
            addRequired(p,'DeltaComm');
            addRequired(p,'DeltaHold');
            parse(p,Map, AgentWeights,DeltaComm,DeltaHold)
            obj.DeltaComm = p.Results.DeltaComm;
            obj.DeltaHold = p.Results.DeltaHold;
            obj.Map = p.Results.Map;
            obj.AgentWeights = AgentWeights;
            obj.TimeOfUpdates = zeros(length(AgentWeights),1);
        end
        
        function obj = CreateDistances(obj)
            numofpoints = length(obj.Map.PointsIndices);
            obj.Distances = zeros(numofpoints);
            PointLocationsx = zeros(numofpoints,1);
            PointLocationsy = zeros(numofpoints,1);
            for i = 1:numofpoints
                PointLocationsx(i)=obj.Map.Points{i}.x;
                PointLocationsy(i)=obj.Map.Points{i}.y;
            end
            obj.Distances = zeros(numofpoints);
            for i = 1:numofpoints
                for j = 1:numofpoints
                    obj.Distances(i,j) = abs(PointLocationsx(i)-PointLocationsx(j))+abs(PointLocationsy(i)-PointLocationsy(j));%sqrt((PointLocationsx(i)-PointLocationsx(j))^2+(PointLocationsy(i)-PointLocationsy(j))^2);
                end
            end
            
            %create vector of indices
            obj.Indices = obj.Map.PointsIndices;
            %             for i = 1:length(obj.Map.Points)
            %                 obj.Indices(i) = obj.Map.Points{i}.index;
            %             end
        end
        
        function obj = InitializeAgents(obj,NRegions)
            %create Agents starting at random positions with no coverings
            obj.Coverings = cell(NRegions,1);
            obj.Centers = randi(length(obj.Map.Points),NRegions,1);
            for i = 1:NRegions
                obj.Coverings{i} = obj.Centers(i);
                obj.Map.Points{obj.Centers(i)}.MostRecentAgent =i;
            end
        end
        
        function [pPlus,pMinus, c, obj] = OneToBaseUpdate(obj,Agent,time,dt)
            %create matrix of correspoding density values for points
            Densities = zeros(1,length(obj.Map.Points));
            for i = 1:length(obj.Map.Points)
                Densities(i) = obj.Map.Points{i}.Density;
            end
            Densities = ones(length(obj.Map.Points),1)*Densities;
            DensityDistance = obj.Distances.*Densities;
            
            %include weight of agents
            DensityDistance = DensityDistance./obj.AgentWeights(Agent);
            for i = 1:length(obj.Centers)
                DensityDistance(obj.Centers(i),:) = DensityDistance(obj.Centers(i),:).*obj.AgentWeights(Agent)./obj.AgentWeights(i);
            end
            
            
            %logical indexing of other indices
            otherAgents = ismember(obj.Indices,obj.Centers) & ismember(obj.Indices,obj.Centers(Agent)) ==0;
            %logical index of this center
            thiscenter = ismember(obj.Indices,obj.Centers(Agent));
            %gives vector of minimum cost of every point to other centers
            minCostOtherCenters = min(DensityDistance(otherAgents,:));
            %compares cost with cost to this Agent's center
            minCcost = min(minCostOtherCenters, DensityDistance(thiscenter,:));
            minCcost = sum(minCcost);
            %Find initial pPlus and pMinus
            pPlus = DensityDistance(thiscenter,:)<minCostOtherCenters;
            pPlus = obj.Indices(pPlus);
            totalOtherCoverings = [];
            for i = 1: length(obj.Coverings)
                if i ~= Agent
                    totalOtherCoverings = [totalOtherCoverings obj.Coverings{i}];
                end
            end
            pMinus = obj.Indices(ismember(obj.Indices,totalOtherCoverings) & ismember(obj.Indices, obj.Coverings{Agent}));
            c = obj.Centers(Agent);
            
            %move Agent's center around to see if there is a better center
            %k is logical indexing of all points that aren't centers or in
            %any other agent's partition.
            k = ismember(obj.Indices,obj.Coverings{Agent}) & ismember(obj.Indices,totalOtherCoverings)==0 & ismember(obj.Indices, obj.Centers) ==0;
            kIndices = obj.Indices(k);
            DensityDistanceK = DensityDistance(k,:);
            MatrixCostOtherCenters = ones(size(DensityDistanceK,1),1)*minCostOtherCenters;
            BelongToK = DensityDistanceK < MatrixCostOtherCenters;
            NotToK = DensityDistanceK >= MatrixCostOtherCenters;
            sumofcosts = (DensityDistanceK.*BelongToK+MatrixCostOtherCenters.*NotToK)*ones(size(DensityDistanceK,2),1);
            [minkcost,position] = min(sumofcosts);
            if minkcost<minCcost
                c = kIndices(position);
                pPlus = obj.Indices(BelongToK(position,:));
            end
            [pPlus,obj] = obj.UpdateTimePoints(Agent,pPlus,pMinus,c,time,dt);
        end
        
        function [add, obj] = UpdateTimePoints(obj,Agent,pPlus,pMinus,c,time,dt)
            newtime = [];
            %create points to add as well us update timer.
            add = {};
            totalCoverings = [];
            for i = 1: length(obj.Coverings)
                totalCoverings = [totalCoverings obj.Coverings{i}];
            end
            totalOtherCoverings = [];
            for i = 1: length(obj.Coverings)
                if i ~= Agent
                    totalOtherCoverings = [totalOtherCoverings obj.Coverings{i}];
                end
            end
            
            %create timer for new points
            for i = 1:length(pPlus)
                timeleft = max(time,obj.Map.Points{obj.Map.PointsIndices == pPlus(i)}.TimeActive);
                if isempty(timeleft)
                    timeleft=time;
                end
                add{i} = obj.Map.Points{pPlus(i)};
                add{i}.TimeLastUpdate = time;
                add{i}.MostRecentAgent = Agent;
                if ismember(pPlus(i), totalOtherCoverings) ==0
                    %if not in any of the other coverings, timer = 0;
                    add{i}.TimeActive = time;
%                 elseif obj.Map.Points{pPlus(i)}.MostRecentAgent == Agent
%                     %keep same timer if point was already meant to be for
%                     %this agent
%                     add{i}.TimeActive = timeleft;
                else
                    %if it was in other region
%                     add{i}.TimeActive = min(time + obj.DeltaComm,obj.DeltaComm +timeleft);
                    incoverings = false(length(obj.Coverings),1);
                    for j = 1:length(obj.Coverings)
                        if ismember(pPlus(i),obj.Coverings{j}) & j~=Agent
                            incoverings(j) = true;
                        end
                    end
                    add{i}.TimeActive = obj.DeltaComm + max(obj.TimeOfUpdates(incoverings));
                end
                
%                 if obj.Map.Points{pPlus(i)}.MostRecentAgent == Agent
%                     obj.Map.Points{pPlus(i)}.TimeActive = add{i}.TimeActive;
%                 else
                    obj.Map.Points{pPlus(i)}.TimeActive = add{i}.TimeActive + obj.DeltaHold;
%                 end
                obj.Map.Points{pPlus(i)}.TimeLastUpdate = time;
                obj.Map.Points{pPlus(i)}.MostRecentAgent = Agent;
            end
            %Update coverings on BaseStation
            obj.Coverings{Agent}(ismember(obj.Coverings{Agent},pMinus)) = [];
            pPlus(ismember(pPlus,obj.Coverings{Agent})) = [];
            obj.Coverings{Agent} = [obj.Coverings{Agent}, pPlus];
            obj.Centers(Agent) = c;
            obj.TimeOfUpdates(Agent) = time;
        end
        
    end
end