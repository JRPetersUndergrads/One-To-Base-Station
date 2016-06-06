clc
clear all
close all;
Ngrid = [];
list = [];

BaseAnimate = VideoWriter('BaseAnimate.avi');
BaseAnimate.FrameRate = 1;
open(BaseAnimate);
AgentsAnimate = VideoWriter('AgentsAnimate.avi');
AgentsAnimate.FrameRate = 10;
open(AgentsAnimate);

%% Initialization Parameters
NGridx = 20;
NGridy = 20;
Domain.Ngridx = NGridx;
Domain.Ngridy = NGridy;
OuterBoundaries = [0, 0, 100, 100]; %[starting x, end x, width, height]
Domain.Lmin = OuterBoundaries(1:2);
Domain.Lmax = OuterBoundaries(1:2)+OuterBoundaries(3:4);
Domain.Dxsize = OuterBoundaries(3);
Domain.Dysize = OuterBoundaries(4);
CommPercent = 1;
NRegions = 4;
%AgentWeights = [.25;.25;.25;.25];
AgentWeights = ones(NRegions,1);
DeltaHold = 0;
DeltaComm = 10;

mpdc = distinguishable_colors(NRegions);
transparancy = 0.25;
dt = 0.5;
time = 10000;

wmin=-5; wmax=5;
K=10;
vehiclemodeltype=1;
nk = 50;DSMCOn=1;
Ck=zeros(nk,nk);
BaseFig = 1;
AgentFig = 2;
weighted = 'true';

%% Create Grid Points and Map

x = linspace(OuterBoundaries(1), OuterBoundaries(1)+OuterBoundaries(3),NGridx+1);
x = x(1:NGridx)+0.5*diff(x); %move to center of discrete areas
y = linspace(OuterBoundaries(2), OuterBoundaries(2)+OuterBoundaries(4),NGridy+1);
y = y(1:NGridy)+0.5*diff(y); %move to center of discrete areas
[x,y] = meshgrid(x,y);
x = reshape(x,numel(x),1); %vectorize the matrix
y = reshape(y,numel(y),1);

width = OuterBoundaries(3)/NGridx;
height = OuterBoundaries(4)/NGridy;
Domain.dxgrid = width;
Domain.dygrid = height;
edgewidth = 1;
edgeheight = 1;
distMethod = 'BFS';
if weighted
    distMethod = 'Dijkstra';
    edgewidth = width;
    edgeheight = height;
end
% index starts from bottom left, goes up column, then moves to next column
%create adjacency matrix
edges = zeros(NGridy*NGridx);
for i = 1:NGridx*NGridy
    for j = 1:NGridx*NGridy
        if j == (i-NGridy) || j == (i+NGridy) %destination is left or right of source
            edges(i,j) = edgewidth; %distance
        end
        if rem(i,NGridy) ==0 %source point is on top
            if j == i-1 %destination point is below source point
                edges(i,j) =edgeheight;
            end
        elseif rem(i,NGridy) ==1 %source point is on bottom
            if j == i+1 %destination point is above source point
                edges(i,j) =edgeheight;
            end
        else
            if j == i-1 %destination point is below source point
                edges(i,j) = edgeheight;
            elseif j == i+1 %destination point is above source point
                edges(i,j) =edgeheight;
            end
        end
    end
end
density = zeros(length(x),1);
CompleteMap = Map(x,y,density,edges);
CompleteMap = CompleteMap.CreateEvenDensity;
CompleteMap = CompleteMap.AddGaussian([14 12], [600 0;0 600], [1 10]);
%CompleteMap = CompleteMap.AddGaussian([60 60], [300 0;0 600], [1 1]);


%% Create Base Station
Base = BaseStation(CompleteMap,DeltaComm,DeltaHold,distMethod,BaseAnimate);
Base = Base.InitializeAgents(NRegions,AgentWeights,mpdc);
Base.ToUpdateOrNotToUpdate = 1;
%% Create Agents
Agents = {};
Coverage = SMC(Ck,nk,K,DSMCOn,vehiclemodeltype);

for i = 1:NRegions
    tempxy = Base.Map.xy(Base.Centers(i), :);
    Agents{i} = Agent(i,Base.Centers(i),CompleteMap, Base.Coverings{i},Coverage,Domain,Base.AgentWeights(i));
end

%% Record location density
AgentLocations = zeros(1,length(NRegions));
locationRecord = zeros(1,NGridx*NGridy);
locationRecordNorm = locationRecord;
for i = 1:NRegions
    AgentLocations(i) = Agents{i}.posIndex;
end
locationRecord(AgentLocations) = locationRecord(AgentLocations)+1;
locationRecordNorm = locationRecord/sum(locationRecord);
AvePerDev = [];
TimeCost= [];
timeSinceNoChange=0;

for t = 1:dt:time
    Base = Base.BaseTimeUpdate(dt);
    AgentToUpdate = [];
    for i = 1:NRegions
        chanceWeight(i) = 1/(DeltaComm-Base.TimeSinceUpdates(i));
        if DeltaComm-Base.TimeSinceUpdates(i) <=0
            AgentToUpdate = i;
            break
        end
    end
    totalChance = sum(chanceWeight)/CommPercent;
    if isempty(AgentToUpdate)
        pick = rand*totalChance;
        countersum =0;
        for i = 1:NRegions
            countersum + chanceWeight(i);
            if pick < (countersum + chanceWeight(i))
                AgentToUpdate = i;
                break
            end
            countersum = countersum + chanceWeight(i);
        end
    end
    list = [list AgentToUpdate];
    
    if ~isempty(AgentToUpdate)
        [Base TimeCost(end+1)] = Base.OneToBaseUpdate(AgentToUpdate,width,height,transparancy,BaseFig,dt);
        Agents{AgentToUpdate} = Agents{AgentToUpdate}.BaseUpdate(Base.Coverings{AgentToUpdate},Base.Centers(AgentToUpdate),Base.AgentTimers(AgentToUpdate)+dt,CompleteMap);
        disp(['Hmin' num2str(TimeCost(end))]);
    end
    
    if sum(Base.AgentTimers)==0
        timeSinceNoChange = timeSinceNoChange +1;
    else
        timeSinceNoChange = 0
    end
    
    disp(' ')
    disp(['Time: ' num2str(t) '   Agent To Update: ' num2str(AgentToUpdate)])
    disp(['Timers: ' num2str(Base.AgentTimers')])
    
    for i = 1:NRegions
        Agents{i} = Agents{i}.MovementAndTimeUpdate(t,dt);
    end
    
    figure(AgentFig)
    clf
    hold on
    for i = 1:NRegions
        Agents{i}.plot(mpdc(i,:), transparancy);
    end
    hold off
    
    axis([Domain.Lmin(1) Domain.Lmax(1) Domain.Lmin(2) Domain.Lmax(2)])
    drawnow
    Agentframe = getframe(figure(AgentFig));
    writeVideo(AgentsAnimate,Agentframe);
    
    %% Density caculations
    for i = 1:NRegions
        AgentLocations(i) = Agents{i}.posIndex;
    end
    locationRecord(AgentLocations) = locationRecord(AgentLocations)+1;
    locationRecordNorm = locationRecord/sum(locationRecord);
    AvePerDev(end+1) = mean(abs(locationRecordNorm - CompleteMap.Densities)./CompleteMap.Densities);
    disp(['deviation' num2str(AvePerDev(end))]);
     if timeSinceNoChange > 2*DeltaComm
         Base.ToUpdateOrNotToUpdate = 0;
     end
end
