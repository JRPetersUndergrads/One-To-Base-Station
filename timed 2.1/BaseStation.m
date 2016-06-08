classdef BaseStation
    
    properties
        Map; %complete map
        TimeSinceUpdates; %time since last update for each agent
        AgentTimers;
        AgentWeights; %weights of each agent
        Coverings; % current covering of each agent. (active or timed)
        PerceivedCoverings;
        Centers; %updated center of each agent
        DeltaComm;
        DeltaHold;
        mpdc;
        distMethod;
        DistMatrix;
        BaseVid;
        HminCost;
        ToUpdateOrNotToUpdate
    end
    methods
        %Constructor
        function obj = BaseStation(Map,DeltaComm,DeltaHold,distMethod,BaseVid)
            p = inputParser;
            p.KeepUnmatched = 1;
            addRequired(p,'Map');
            addRequired(p,'DeltaComm');
            addRequired(p,'DeltaHold');
            addRequired(p,'distMethod');
            addRequired(p,'BaseVid');
            parse(p,Map,DeltaComm,DeltaHold,distMethod,BaseVid)
            obj.DeltaComm = p.Results.DeltaComm;
            obj.DeltaHold = p.Results.DeltaHold;
            obj.Map = p.Results.Map;
            obj.distMethod = p.Results.distMethod;
            obj.BaseVid = BaseVid;
            %create distance matrix for whole graph
            h = obj.Map.xy(2,2)-obj.Map.xy(1,2);
            index = (max(obj.Map.xy(:,2))-min(obj.Map.xy(:,2)))/h+2;
            w = obj.Map.xy(round(index),1)-obj.Map.xy(1,1);
            obj.DistMatrix = Inf(length(obj.Map.PointsIndices));
            if strcmp(obj.distMethod,'BFS')
                tempxy(:,1) = obj.Map.xy(:,1)/w;
                tempxy(:,2) = obj.Map.xy(:,2)/h;
                %make unweighted
                for i = 1:length(obj.Map.PointsIndices)
                    obj.DistMatrix(i,:) = abs(tempxy(:,1)-tempxy(i,1))+abs(tempxy(:,2)-tempxy(i,2));
                end
            elseif strcmp(obj.distMethod,'Dijkstra')
                tempxy = obj.Map.xy;
                for i = 1:length(obj.Map.PointsIndices)
                    obj.DistMatrix(i,:) = abs(tempxy(:,1)-tempxy(i,1))+abs(tempxy(:,2)-tempxy(i,2));
                end
            end
        end
        
        function obj = InitializeAgents(obj,NRegions,AgentWeights,mpdc)
            obj.mpdc = mpdc;
            %create Agents starting at random positions then use voronoi
            %partition.
            obj.AgentTimers = zeros(NRegions,1);
            obj.TimeSinceUpdates = zeros(NRegions,1);
            obj.AgentWeights = AgentWeights;
            obj.Coverings = cell(NRegions,1);
            obj.Centers = randperm(max(obj.Map.PointsIndices),NRegions);
            %calculate initial coverings (voronoi)
            for i = 1:length(obj.Centers)
                %distances(i,:) = graphshortestpath(sparse(obj.Map.Edges),obj.Centers(i),'Method',obj.distMethod)/obj.AgentWeights(i);
                distances(i,:) = obj.DistMatrix(obj.Centers(i),:)/obj.AgentWeights(i);
            end
            %distances = bfs(obj.Map.Edges,1:length(obj.Map.PointsIndices));%gives distance matrix using bfs
            %distances = distances(obj.Centers,:);
            mindist = min(distances);
            assignedpoints = [];
            for i = 1:NRegions
                obj.Coverings{i} = obj.Map.PointsIndices(distances(i,:) == mindist);
                obj.Coverings{i}(ismember(obj.Coverings{i},assignedpoints)) = [];
                assignedpoints = [assignedpoints obj.Coverings{i}];
            end
            obj.PerceivedCoverings = obj.Coverings;
        end
        
        function obj = BaseTimeUpdate(obj,dt)
            obj.AgentTimers=(max(0,obj.AgentTimers-dt));
            obj.TimeSinceUpdates = obj.TimeSinceUpdates + dt;
        end
        
        function [obj lowCost] = OneToBaseUpdate(obj,Agent,width,height,transparancy,BaseFig,dt)%,newtime,dt)
            %OneToBaseUpdate = Algorithm 2
            if obj.ToUpdateOrNotToUpdate == 1
                temp = obj.AgentTimers(Agent);
            else
                temp = 1;
            end
            if  temp <=0 %only update Agent if it doesn't have timer on it
                %first calculate new region with current center
                c = obj.Centers(Agent);
                [NewCoverings lowCost] = obj.FindRegions(Agent,c);
                if lowCost > obj.HminCost
                    NewCoverings = obj.Coverings;
                    lowCost = obj.HminCost;
                end
                %loop through and use all other points in covering as
                %center
                thisCoveringNotCenter = obj.Coverings{Agent};
                thisCoveringNotCenter(ismember(thisCoveringNotCenter,obj.Centers(Agent))) = [];
                count = 0;
                for k = thisCoveringNotCenter
                    count = count+1;
                    %clc
                    %disp([num2str(count),' out of ' ,num2str(length(thisCoveringNotCenter))]);
                    [compareCoverings compareCost] = obj.FindRegions(Agent,k);
                    %if this k gives us better cost, use k
                    if compareCost<lowCost
                        NewCoverings = compareCoverings;
                        lowCost = compareCost;
                        c = k;
                    end
                end
                obj.AgentTimers(Agent) = obj.FindTimer(Agent, NewCoverings, c);
                obj.Coverings = NewCoverings;
                obj.PerceivedCoverings{Agent} = NewCoverings{Agent};
                obj.Centers(Agent) = c;
                obj.PlotBaseRegions(width,height,transparancy,BaseFig)
                if lowCost > obj.HminCost
                    error
                end
                obj.HminCost = lowCost;
            else
                lowCost = obj.HminCost;
            end
            
                obj.TimeSinceUpdates(Agent) = 0;
        end
        
        function [NewCoverings, Cost] = FindRegions(obj,Agent,k)
            %FindRegions is Algorithm 1
            initial = true;
            NewCoverings = obj.Coverings;
            pPlusnew = [];
            %while loop to add points until no more points to add
            while ~isempty(pPlusnew) || initial
                initial = false;
                %find adjacent points
                adjacent = sum(obj.Map.Edges(NewCoverings{Agent},:),1);
                adjacent = adjacent > 0;
                adjacent = obj.Map.PointsIndices(adjacent);
                %remove points already in pPlus or in region with active
                %timer
                adjacent(ismember(adjacent,NewCoverings{Agent})) = [];
                RestrictedIndices = []; %points in region with active timer
                for i = 1:length(obj.AgentTimers);
                    if obj.AgentTimers(i) > 0
                        RestrictedIndices = [RestrictedIndices obj.Coverings{i}];
                    end
                end
                adjacent(ismember(adjacent, RestrictedIndices)) = [];
                adjacent = sort(adjacent);
                %now to compare the distance from these adjacent points to
                %the centers
                otherAgents = 1:length(NewCoverings);
                otherAgents(Agent) = [];
                %loop through each agent to compare costs
                costCompare = inf(1,length(adjacent));
                for i = otherAgents
                    if false %obj.DistType(NewCoverings{i},obj.Centers(i)) %if we can use dist Matrix
                        %disp('check')
                        thisCenterCost = obj.DistMatrix(obj.Centers(i),adjacent)/obj.AgentWeights(i);
                    else
                        %disp('no check')
                        %build Edges for this region
                        notThisRegion = ~ismember(obj.Map.PointsIndices,NewCoverings{i});
                        tempEdges = obj.Map.Edges;
                        tempEdges(notThisRegion,:) = 0; %remove edges not in subset
                        tempEdges(:,notThisRegion) = 0;
                        thisCenterCost = graphshortestpath(sparse(tempEdges),obj.Centers(i),adjacent,'Method',obj.distMethod)/obj.AgentWeights(i);
                    end
                    costCompare = min(costCompare,thisCenterCost);
                end
                %calculate cost for this agent
                if false %obj.DistType(NewCoverings{Agent},k) %if we can use dist Matrix
                    %disp('check')
                    thisCenterCost = obj.DistMatrix(k,adjacent)/obj.AgentWeights(Agent);
                else
                    %disp('no check')
                    notThisRegion = ~ismember(obj.Map.PointsIndices,[NewCoverings{Agent} adjacent]);
                    tempEdges = obj.Map.Edges;
                    tempEdges(notThisRegion,:) = 0; %remove edges not in subset
                    tempEdges(:,notThisRegion) = 0;
                    thisCenterCost = graphshortestpath(sparse(tempEdges),k,adjacent,'Method',obj.distMethod)/obj.AgentWeights(i);
                    
                end
                pPlusnew = adjacent(thisCenterCost<costCompare);
                %now we have points that we can add. add below
                NewCoverings{Agent} = sort([NewCoverings{Agent} pPlusnew]);
                %remove points from other Coverings
                for i = otherAgents
                    NewCoverings{i}(ismember(NewCoverings{i},pPlusnew)) = [];
                end
            end
            %calculate cost with new Coverings
            tempCenters = obj.Centers;
            tempCenters(Agent) = k;
            Cost = obj.Hmin(NewCoverings,tempCenters);
% %             %% Plot Calculations
%                         tempBase = obj;
%                         tempBase.Coverings = NewCoverings;
%                         tempBase.Centers(Agent) = k;
%                         transparancy = 0.25;
%                         h = obj.Map.xy(2,2)-obj.Map.xy(1,2);
%                         index = (max(obj.Map.xy(:,2))-min(obj.Map.xy(:,2)))/h+2;
%                         w = obj.Map.xy(round(index),1)-obj.Map.xy(1,1);
%                         tempBase.PlotBaseRegions(w,h,transparancy,1);
        end
        
        function timer = FindTimer(obj,Agent,NewCoverings,c)
            %UpdateTimer is Algorithm 3
            % takes in new coverings and finds timer to add to agent
            timer = 0;
            otherAgentIndex = 1:length(obj.Coverings);
            otherAgentIndex(obj.AgentTimers > 0 ) = []; %remove agents that have timer on it
            otherAgentIndex(ismember(otherAgentIndex,Agent)) = []; %find index of other agents
            %loop through other Agents to see maximum time for them to go
            %to new region
            for i = otherAgentIndex
                MaxRelocateDist = 0;
                oldRegion = obj.PerceivedCoverings{i};
                removedRegion = oldRegion(~ismember(oldRegion,NewCoverings{i}));
                if obj.AgentTimers(i) > 0 && ~isempty(removedRegion)
                    i
                    error
                end
                %build Edges for this region
                notThisRegion = ~ismember(obj.Map.PointsIndices,oldRegion);
                tempEdges = obj.Map.Edges;
                tempEdges(notThisRegion,:) = 0; %remove edges not in subset
                tempEdges(:,notThisRegion) = 0;
                %loop through every removed point
                for j = removedRegion
                    %time to relocate = distance/speed
                    tempRelocateDist = min(graphshortestpath(sparse(tempEdges),j,NewCoverings{i},'Method',obj.distMethod));
                    MaxRelocateDist = max(MaxRelocateDist,tempRelocateDist);
                end
                %now MaxRelocateDist is the maximum distance this other agent will
                %have to travel to get back to new region. Convert to time
                MaxRelocateTime = MaxRelocateDist/obj.AgentWeights(i);
                timeTillUpdate = obj.DeltaComm - obj.TimeSinceUpdates(i);
                if isempty(removedRegion)
                    timeTillUpdate = 0;
                end
                timer = max(timer, MaxRelocateTime + timeTillUpdate + obj.DeltaHold);
            end
            
            %now find max time for this agent to go to new region
            oldRegion = obj.PerceivedCoverings{Agent};  %old indices of this agents coverings
            removedRegion = oldRegion(~ismember(oldRegion,NewCoverings{Agent}));    %old indices that are not in new region
            notRemovedRegion = oldRegion(ismember(oldRegion,NewCoverings{Agent}));
            notThisRegion = ~ismember(obj.Map.PointsIndices,oldRegion);
            tempEdges = obj.Map.Edges;
            tempEdges(notThisRegion,:) = 0; %remove edges not in subset
            tempEdges(:,notThisRegion) = 0;
            MaxRelocateDist = 0;
            for j = removedRegion
                %time to relocate = distance/speed
                tempRelocateDist = min(graphshortestpath(sparse(tempEdges),j,notRemovedRegion,'Method',obj.distMethod));
                MaxRelocateDist = max(MaxRelocateDist,tempRelocateDist);
            end
            MaxRelocateTime = MaxRelocateDist/obj.AgentWeights(Agent);
            timer = max(timer, MaxRelocateTime);
        end
        
        function cost = Hmin(obj,Coverings,Centers)
            costCompare = inf(1,length(obj.Map.PointsIndices));
            for i = 1:length(Coverings)
                %build Edges for this region
                notThisRegion = ~ismember(obj.Map.PointsIndices,Coverings{i});
                tempEdges = obj.Map.Edges;
                tempEdges(notThisRegion,:) = 0; %remove edges not in subset
                tempEdges(:,notThisRegion) = 0;
                thisCenterCost = graphshortestpath(sparse(tempEdges),Centers(i),'Method',obj.distMethod)/obj.AgentWeights(i);
                costCompare = min(costCompare,thisCenterCost);
            end
            cost = sum(costCompare.*obj.Map.Densities);
        end
        
        function PlotBaseRegions(obj,w,h,transparancy,FigNum)
            figure(FigNum)
            clf
            hold on
            %get location of centers
            centersx = obj.Map.xy(obj.Centers,1);
            centersy = obj.Map.xy(obj.Centers,2);
            %go through each region and plot
            for i = 1:length(obj.Coverings)
                patchesx = [];
                patchesy = [];
                scatter(centersx(i),centersy(i),250,obj.mpdc(i,:),'filled')
                
                %get patches for each point in region
                totalx = obj.Map.xy(obj.Coverings{i},1);
                totaly = obj.Map.xy(obj.Coverings{i},2);
                for j = 1:length(obj.Coverings{i})
                    TempX = [(totalx(j)-(w/2)) (totalx(j)+(w/2)) (totalx(j)+(w/2)) (totalx(j)-(w/2))];
                    TempY = [(totaly(j)+(h/2)) (totaly(j)+(h/2)) (totaly(j)-(h/2)) (totaly(j)-(h/2))];
                    patchesx(:,j) = TempX;
                    patchesy(:,j) = TempY;
                end
                patch(patchesx,patchesy,obj.mpdc(i,:))
            end
            alpha(transparancy)
            hold off
            drawnow
            BaseFrame = getframe(figure(FigNum));
            writeVideo(obj.BaseVid,BaseFrame);
        end
        
        function useManDist = DistType(obj, tempindices,k)
            check = true;
            useManDist = false;
            if check
                %input indices of region and center value. Calculate if it is
                %safe to just use manhattan distance for
                useManDist = true; %start off allowing manhatten distance
                h = obj.Map.xy(2,2)-obj.Map.xy(1,2);
                index = (max(obj.Map.xy(:,2))-min(obj.Map.xy(:,2)))/h+2;
                w = obj.Map.xy(round(index),1)-obj.Map.xy(1,1);
                
                centerxy = obj.Map.xy(k,:);
                xy = obj.Map.xy(tempindices,:); %get xy values of this region
                %see if agent can traverse horizontally then vertically.
                uniquex = unique(xy(:,1))';
                for i = uniquex %loop through horizontally to see if agent can get to every point
                    incolumn = xy(ismember(xy(:,1),i),2); %find every y value in that column
                    useManDist = useManDist && (length(incolumn) == round((max(incolumn)-min(incolumn))/h+1)); %check to see that every element ( only for that x value) is connected
                    useManDist = useManDist && min(incolumn) <= centerxy(2) && max(incolumn) >= centerxy(2); %check to see that agent can get into that column
                end
                
                %if it can't traverse horizontally, see if it can traverse
                %vertically
                if ~useManDist
                    tempUseMan = true;
                    uniquey = unique(xy(:,2));
                    for i = uniquey %loop through vertically to see if agent can get to that point
                        inrow = xy(ismember(xy(:,2),i),1); %find every x value in that column
                        tempUseMan = tempUseMan && (length(inrow) == round((max(inrow)-min(inrow))/w+1));
                        tempUseMan = tempUseMan && min(inrow) <= centerxy(1) && max(inrow) >= centerxy(1);
                    end
                    useManDist = tempUseMan;
                end
            end
        end
    end
    
    
end
