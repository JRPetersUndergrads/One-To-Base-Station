clear all
close all;

%% Initialization Parameters
CommPercent = .75;
NRegions = 4;
AgentWeights = [.25 .25 .25 .25];
NGridx = 60;
NGridy = 60;
OuterBoundaries = [0, 0, 100, 100]; %[starting x, end x, width, height]
DeltaHold = 0;
DeltaComm = 10;
T = 1000;
mpdc = distinguishable_colors(NRegions+2);
transparancy = 0.25;
dt = 1;

uavminspeed=3;
uavmaxspeed=5;
wmin=-5; wmax=5;
K=10;
vehiclemodeltype=1;
nk = 50;DSMCOn=1;
Ck=zeros(nk,nk);

figure(1)
for i = 1:NRegions
    patch([i-1/2 i+1/2 i+1/2 i-1/2],[1 1 0 0],mpdc(i,:))
end
title('Agent Colors')
xlabel('Agent Index')
axis([1/2 NRegions+1/2 0 1])
set(gca,'xtick',0:NRegions)
set(gca,'ytick',[])

total = VideoWriter('total.avi');
total.FrameRate = 10;
open(total);
active = VideoWriter('actual.avi');
active.FrameRate = 10;
open(active);

%% Create Grid Points and Map
% x = linspace(OuterBoundaries(1), OuterBoundaries(2),NGridx+1);
% width = diff(x);
% x = x(1:NGridx)+0.5*width; %move to center of discrete areas
% y = linspace(OuterBoundaries(3), OuterBoundaries(4),NGridy+1);
% height = diff(y);
% y = y(1:NGridy)+0.5*height; %move to center of discrete areas
% [x,y] = meshgrid(x,y);
% x = vec(x); %vectorize the matrix
% y = vec(y);
% width = width(1);
% height = height(1);

x = linspace(OuterBoundaries(1), OuterBoundaries(1)+OuterBoundaries(3),NGridx+1);
x = x(1:NGridx)+0.5*diff(x); %move to center of discrete areas
y = linspace(OuterBoundaries(2), OuterBoundaries(2)+OuterBoundaries(4),NGridy+1);
y = y(1:NGridy)+0.5*diff(y); %move to center of discrete areas
Contourx = x;
Contoury = y;
[x,y] = meshgrid(x,y);
x = vec(x); %vectorize the matrix
y = vec(y);
% x and y are corresponding list of x and y components

width = OuterBoundaries(3)/NGridx;
height = OuterBoundaries(4)/NGridy;


density = zeros(length(x),1);
TimeActive = zeros(length(x),1);

CompleteMap = Map({});
CompleteMap = CompleteMap.CreatePoints(x,y,density,TimeActive);
CompleteMap = CompleteMap.CreateEvenDensity;
CompleteMap = CompleteMap.AddGaussian([14 12], [600 0;0 600], [1 10]);
% CompleteMap = CompleteMap.AddGaussian([60 60], [300 0;0 600], [1 1]);
ContourDensity = zeros(NGridy,NGridx);
for i =1: NGridy*NGridx
    ContourDensity(i) = CompleteMap.Points{i}.Density;
end
figure(4)
mesh(Contourx,Contoury,ContourDensity)
drawnow

%% Create Base Station
Base = BaseStation(CompleteMap,AgentWeights,DeltaComm,DeltaHold);
Base = Base.CreateDistances;
Base = Base.InitializeAgents(NRegions);

%% Create Agents
Agents = {};
Coverage = SMC(Ck,nk,K,DSMCOn,vehiclemodeltype);

for i = 1:NRegions
    x=Base.Map.Points{Base.Centers(i)}.x;
    y=Base.Map.Points{Base.Centers(i)}.y;
    State = Dubins([Base.Map.Points{Base.Centers(i)}.x Base.Map.Points{Base.Centers(i)}.y 0], 0 , {}, {},{},'MaxSpeed',uavmaxspeed,'MinSpeed',uavminspeed, 'MinAngularSpeed', wmin, 'MaxAngularSpeed',wmax);
    Agents{i} = Agent(i,Base.Centers(i),Map(Base.Map.Points{Base.Centers(i)}),0,State,Coverage);
end


%clc
lastupdate =[];
%% Loop one to base
for time = 1:dt:T
    %%pick agent
    AgentToUpdate = [];
    for i = 1:length(Agents)
        chanceWeight(i) = 1/(DeltaComm-time+Base.TimeOfUpdates(i));
        if DeltaComm-time+Base.TimeOfUpdates(i) <=0
            AgentToUpdate = i;
            break
        end
    end
    totalChance = sum(chanceWeight)/CommPercent;
    if totalChance < Inf
        pick = rand*totalChance;
        countersum =0;
        for i = 1:length(Agents)
            if pick < (countersum + chanceWeight(i))
                AgentToUpdate = i;
                pick = totalChance*2;
                break
            end
            countersum = countersum + chanceWeight(i);
        end
    end
    figure(3)
    line1 = ['last update = Agent ' num2str(lastupdate)];
    line2 = ['next update = Agent ' num2str(AgentToUpdate)];
    line3 = ['time = ' num2str(time)];
    xlabel({line1;line2;line3})
    figure(2)
    xlabel({line1;line2;line3})
    drawnow
    frame = getframe(figure(2));
    writeVideo(total,frame);
    figure(3)
    frame = getframe(figure(3));
    writeVideo(active,frame);
    
    if isempty(AgentToUpdate) ==0
        %simulate OneToBaseUpdate
        disp('updating...')
        [pPlus,pMinus, c, Base] = OneToBaseUpdate(Base,AgentToUpdate,time,dt);
        Agents{AgentToUpdate}.Map = Agents{AgentToUpdate}.Map.DeletePoints(pMinus);
        
        Agents{AgentToUpdate}.Map = Agents{AgentToUpdate}.Map.AddPoints(pPlus);
        Agents{AgentToUpdate}.Center = c;
        Agents{AgentToUpdate}.TimeLastContact = time;
        
        %check to make sure total map of agent is same as total map of base
        check =  Base.Coverings{AgentToUpdate} == Agents{AgentToUpdate}.Map.PointsIndices;
        if min(check) ==0
            disp(['error Agent = ' num2str(AgentToUpdate)])
            error
        end
        if length(Base.Coverings{AgentToUpdate})~= length(Agents{AgentToUpdate}.Map.PointsIndices)
            disp(['not same length Agent = ' num2str(AgentToUpdate)])
        end
    end
    
    
    %plot for current time
    clf(figure(3))
    clf(figure(2))
    disp('plotting...')
    figure(2)
    title('total map')
    axis([OuterBoundaries(1), OuterBoundaries(1)+OuterBoundaries(3), OuterBoundaries(2), OuterBoundaries(2)+OuterBoundaries(4)]);
    figure(3)
    title('active map')
    axis([OuterBoundaries(1), OuterBoundaries(1)+OuterBoundaries(3), OuterBoundaries(2), OuterBoundaries(2)+OuterBoundaries(4)]);
    hold on
    for i = 1:length(Agents)
        Agents{i} = Agents{i}.TimeUpdate(time,width,height,mpdc(i,:),transparancy);
    end
    hold off
    
    
    
    
    for i = 1:NRegions
        Agents{i} = Agents{i}.smc_update_modified(time, dt, OuterBoundaries, NGridx,NGridy);
    end
    figure(3)
    hold on
    for i = 1:NRegions
        tempx = Agents{i}.State.Location(:,1);
        tempy = Agents{i}.State.Location(:,2);
        color = mpdc(i,:);
        plot(tempx,tempy,'LineWidth',3,'Color',color)
        plot(tempx(end),tempy(end),'Marker','^','MarkerFaceColor',color,'MarkerEdgeColor',color,'MarkerSize',15)
    end
    hold off
    
    lastupdate = AgentToUpdate;
end
close(total);
close(active);
close