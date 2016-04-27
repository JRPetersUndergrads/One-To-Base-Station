 clear all
close all;
Ngrid = [];
%% Initialization Parameters
NGridx = 30;
NGridy = 30;
OuterBoundaries = [0, 0, 2, 2]; %[starting x, end x, width, height]
CommPercent = .75;
NRegions = 4;
%AgentWeights = [.25;.25;.25;.25];
AgentWeights = ones(NRegions,1);
DeltaHold = 0;
DeltaComm = 10;

mpdc = distinguishable_colors(NRegions);
transparancy = 0.25;
dt = 1;
time = 1000;

uavminspeed=3;
uavmaxspeed=5;
wmin=-5; wmax=5;
K=10;
vehiclemodeltype=1;
nk = 50;DSMCOn=1;
Ck=zeros(nk,nk);
BaseFig = 1;
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
%CompleteMap = CompleteMap.AddGaussian([14 12], [600 0;0 600], [1 10]);
% CompleteMap = CompleteMap.AddGaussian([60 60], [300 0;0 600], [1 1]);

%% Create Base Station
Base = BaseStation(CompleteMap,DeltaComm,DeltaHold,distMethod);
Base = Base.InitializeAgents(NRegions,AgentWeights,mpdc);
%% Create Agents
Agents = {};
Coverage = SMC(Ck,nk,K,DSMCOn,vehiclemodeltype);

for i = 1:NRegions
    tempxy = Base.Map.xy(Base.Centers(i), :);
    State = Dubins([tempxy 0], 0 , {}, {},{},'MaxSpeed',uavmaxspeed,'MinSpeed',uavminspeed, 'MinAngularSpeed', wmin, 'MaxAngularSpeed',wmax);
    Agents{i} = Agent(i,Base.Centers(i),CompleteMap, Base.Coverings{i},State,Coverage);
end
%% One To Base Loop
% answer = 1;
% while ~isempty(answer)
%     answer = input('next Agent?');
%     Base = Base.OneToBaseUpdate(answer,width,height,transparancy,BaseFig);
% end

for t = 1:dt:time
    Agent = randi(NRegions);
    Base = Base.OneToBaseUpdate(Agent,width,height,transparancy,BaseFig,dt);
    
end