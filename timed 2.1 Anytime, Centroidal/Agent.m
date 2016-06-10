classdef Agent
    
    properties
        ID;
        Map;
        MapPend;
        Center;
        Timer;
        ActiveDensityVector;
        Activeindex;
        State;
        Centerx;
        Centery;
        Domain;
        uavmeanspeed;
        SMCParams;
        pos;
        posIndex;
        status;
        Directions;
    end
    methods
        %Constructor
        function obj = Agent(ID,Center,CompleteMap,Covering, Coverage,Domain,uavmeanspeed)
            p = inputParser;
            p.KeepUnmatched = 1;
            addRequired(p,'ID');
            addRequired(p,'Center');
            addRequired(p,'CompleteMap');
            addRequired(p,'Covering');
            addRequired(p,'Coverage');
            addRequired(p,'Domain');
            addRequired(p,'uavmeanspeed');
            parse(p,ID,Center,CompleteMap,Covering,Coverage,Domain,uavmeanspeed);
            obj.Center = p.Results.Center;
            obj.ID = p.Results.ID;
            obj.Map = obj.MakeMap(p.Results.CompleteMap,p.Results.Covering);
            obj.MapPend = obj.Map;
            obj.Timer = 0;
            obj.SMCParams = p.Results.Coverage;
            obj.Domain = p.Results.Domain;
            obj.uavmeanspeed = p.Results.uavmeanspeed;
            obj.pos = obj.Map.xy(ismember(obj.Map.PointsIndices,obj.Center),:);
            obj.posIndex = obj.Center;
            obj.status = 'home';
        end
        
        function obj = TimeUpdate(obj,dt)
            %update timer change map if timer is 0
            obj.Timer = obj.Timer - dt;
            if obj.Timer <= 0
                %timer is up
                obj.Timer = 0;
                obj.Map = obj.MapPend;
            end
        end
        
        function obj = PlotRegion(obj,activex,activey,totalx,totaly,centerx,centery,w,h,color,transparancy)
            figure(2)
            hold on
            scatter(centerx,centery,250,color,'filled')
            patchesx = [];
            patchesy = [];
            for i = 1:length(totalx)
                TempX = [(totalx(i)-(w/2)) (totalx(i)+(w/2)) (totalx(i)+(w/2)) (totalx(i)-(w/2))];
                TempY = [(totaly(i)+(h/2)) (totaly(i)+(h/2)) (totaly(i)-(h/2)) (totaly(i)-(h/2))];
                patchesx(:,i) = TempX;
                patchesy(:,i) = TempY;
                %                 patch(TempX,TempY,color)
            end
            patch(patchesx,patchesy,color)
            alpha(transparancy)
            
            hold off
            
            figure(3)
            hold on
            scatter(centerx,centery,250,color,'filled')
            patchesx = [];
            patchesy = [];
            for i = 1:length(activex)
                TempX = [(activex(i)-(w/2)) (activex(i)+(w/2)) (activex(i)+(w/2)) (activex(i)-(w/2))];
                TempY = [(activey(i)+(h/2)) (activey(i)+(h/2)) (activey(i)-(h/2)) (activey(i)-(h/2))];
                patchesx(:,i) = TempX;
                patchesy(:,i) = TempY;
                %                 patch(TempX,TempY,color)
            end
            patch(patchesx,patchesy,color)
            alpha(transparancy)
            
            hold off
            
        end
        
        function obj = BaseUpdate(obj,NewCovering,C,Timer,CompleteMap)
            %called when Agent updates with base
            %make pending map (with timer)
            obj.MapPend = obj.MakeMap(CompleteMap,NewCovering);
            %set Agent's Timer
            obj.Timer = Timer;
            %update Agent's center
            obj.Center = C;
            obj.status = 'call home';
        end
        
        function subMap = MakeMap(obj,CompleteMap, Covering)
            %takes in Map of whole world and coverage and makes a submap
            part = ismember(CompleteMap.PointsIndices,Covering);
            x = CompleteMap.xy(part,1);
            y = CompleteMap.xy(part,2);
            Densities = CompleteMap.Densities(part);
            Edges = CompleteMap.Edges(part,part);
            subMap = Map(x,y,Densities,Edges);
            subMap.PointsIndices = CompleteMap.PointsIndices(part);
        end
        
        function obj = MovementAndTimeUpdate(obj,time,dt)
            obj.Timer = obj.Timer - dt;
            if obj.Timer <=0
                obj.Map = obj.MapPend;
            end
            Lmin=obj.Domain.Lmin;
            Lmax=obj.Domain.Lmax;
            %             Dxsize=obj.Domain.Dxsize;
            %             Dysize=obj.Domain.Dysize;
            dxgrid=obj.Domain.dxgrid;
            dygrid=obj.Domain.dygrid;
            Ngridx=obj.Domain.Ngridx;
            Ngridy=obj.Domain.Ngridy;
            agentspos=obj.pos(end,:);
            %             uavspeed=obj.uavmeanspeed;
            %
            %             Xcount = floor((agentspos(1)-Lmin(1))/dxgrid);
            %             Ycount = floor((agentspos(2)-Lmin(2))/dygrid)+1;
            %             positionIndex = Xcount*Ngridy+Ycount;
            positionIndex = obj.posIndex;
            
            
            if strcmp(obj.status,'return home')
                disp('heading home')
                %if agent is already heading home, continue using
                %directions found before to go home
                obj = obj.headhome(dt);
                if isempty(obj.Directions);
                    obj.status = 'home';
                    disp('reached home')
                    obj.ID
                    obj = obj.DeletePoints;
                end
            elseif ~ismember(positionIndex, obj.MapPend.PointsIndices)
                %if agent has just updated and needs to find path to new
                %region
                disp('heading home')
                obj.status = 'return home';
                possibleindices = ismember(obj.Map.PointsIndices,obj.MapPend.PointsIndices);    %logical index of old points still in new region
                position = ismember(obj.Map.PointsIndices,positionIndex);   %logical index of current position in old map
                indicescount = 1:length(obj.Map.PointsIndices);
                possibleindices = indicescount(possibleindices);
                position = indicescount(position);
                [distance path] = graphshortestpath(sparse(obj.Map.Edges),position,possibleindices);   %to get all distances and paths home
                [distance index] = min(distance);   %find shortest path home and find which point it is to
                path = path{index}; % this will be the path agent will take "home"
                %change path into path directions
                pathDirections = zeros(length(path)-1,2);
                obj.Directions = [];
                for i = 1:length(path)-1
                    obj.Directions(i,:) = obj.Map.xy(path(i+1),:) - obj.Map.xy(path(i),:);
                end
                obj = obj.headhome(dt); %start heading home
                if isempty(obj.Directions) %if reached home then change status and remove points in old map not in new map
                    obj.status = 'home';
                    obj=obj.DeletePoints();
                end
            else
                if ~ismember(obj.posIndex,obj.Map.PointsIndices)
                    agent
                    error('not in region')
                end
                obj = obj.DeletePoints();
                
                %run regular smc
                [obj u v] = obj.smc(time,dt);
                newPosition(1,:) = agentspos(1, :) + [u v] * dt;
                %check to see if new position is still in correct
                %region
                newpositionIndex = obj.findIndex(newPosition);
                if ~ismember(newpositionIndex,obj.Map.PointsIndices) || newPosition(1) < Lmin(1) || newPosition (1) > Lmax(1) || newPosition(2) < Lmin(2) || newPosition (2) > Lmax(2)
                    %has wondered outside of region need to fix smc position
                    xy = obj.Map.xy(ismember(obj.Map.PointsIndices,positionIndex),:); %get xy of old position
                    boundxy = xy + [sign(u)*dxgrid/2 sign(v)*dygrid/2];
                    vector = [u v];
                    %change vector from distance vector to vector of crossed points
                    tempPosition = [agentspos];
                    lastPosition = tempPosition;
                    tempIndex = obj.posIndex;
                    lastIndex = tempIndex;
                    tempdt = dt;
                    %loop through every crossedboundary to find last index that is
                    %still in boundary
                    while ismember(tempIndex,obj.Map.PointsIndices) && tempdt > 0 && tempPosition (1) > Lmin(1) && tempPosition(1) < Lmax(1) && tempPosition (2) >Lmin(2) && tempPosition (2) < Lmax(2)
                        dt = tempdt;
                        lastPosition = tempPosition;
                        lastIndex = tempIndex;
                        timetobound = (boundxy - tempPosition)./vector;
                        timetobound(timetobound<0) = Inf;
                        [timetobound verthorz] = min(timetobound);
                        boundAdd = [sign(u)*dxgrid sign(v)*dygrid];
                        boundxy(verthorz) = boundxy(verthorz) + boundAdd(verthorz);
                        tempPosition = tempPosition+timetobound*vector;
                        Nmove = [Ngridy 1];
                        tempIndex = obj.findIndex(tempPosition);
                        travelDist = tempPosition - lastPosition;
                        travelDist = sqrt(travelDist(1)^2 + travelDist(2)^2);
                        tempdt = dt - travelDist / obj.uavmeanspeed;
                    end
                    %now lastPosition gives us last index before leaving and the point
                    %before it leaves
                    %now head to center of that point
                    newPosition = lastPosition;
                    newpositionIndex = lastIndex;
                    obj.pos=[obj.pos; newPosition];
                    obj.posIndex = newpositionIndex;
                    if dt<0
                        newPosition
                        agentspos
                        obj.pos
                        error('negative dt')
                    end
                    
                    %make home the center of that point and 'head home'
                    obj.status = 'return home';
                    obj.Directions = obj.Map.xy(ismember(obj.Map.PointsIndices,newpositionIndex),:)-newPosition;
                    obj = obj.headhome(dt);
                    if isempty(obj.Directions);
                        obj.status = 'home';
                        disp('reached home')
                        obj.ID
                        obj = obj.DeletePoints;
                    end
%                     direction = direction./sqrt(direction(1)^2+direction(2)^2).*obj.uavmeanspeed;
%                     newPosition = newPosition + direction*travelTime;
%                     obj.pos=[obj.pos; newPosition];
                    
                else
                    % if new position is still in region, then use it
                    obj.pos=[obj.pos; newPosition];
                    obj.posIndex = newpositionIndex;
                end
            end
        end
        
        function [obj u v] = smc(obj,time,dt)
            %input: agent, time, timestep (dt)
            %output: agent (new smc information added, direction of
            %travel(u v)
            Lmin=obj.Domain.Lmin;
            Lmax=obj.Domain.Lmax;
            Dxsize=obj.Domain.Dxsize;
            Dysize=obj.Domain.Dysize;
            dxgrid=obj.Domain.dxgrid;
            dygrid=obj.Domain.dygrid;
            Ngridx=obj.Domain.Ngridx;
            Ngridy=obj.Domain.Ngridy;
            
            uavspeed=obj.uavmeanspeed;
            nk=obj.SMCParams.nk;
            K=obj.SMCParams.K;
            Ck=obj.SMCParams.Ck;
            
            %create mu (density)
            totalindex = 1:Ngridx*Ngridy;
            totalindex = totalindex';
            TotalDensity = zeros(Ngridx*Ngridy,1);
            TotalDensity(obj.Map.PointsIndices) = obj.Map.Densities;
            TotalDensity = TotalDensity./sum(TotalDensity);
            
            mu=reshape(TotalDensity,Ngridy,Ngridx)/(dxgrid*dygrid);
            
            agentspos=obj.pos(end,1:2);
            
            [Gridx,Gridy] = meshgrid(linspace(Lmin(1),Lmax(1),Ngridx),linspace(Lmin(2),Lmax(2),Ngridy));
            xprel = agentspos(1) - Lmin(1);
            yprel = agentspos(2) - Lmin(2);
            %% see if agent is in its new region
            Xcount = floor((agentspos(1)-Lmin(1))/dxgrid);
            Ycount = floor((agentspos(2)-Lmin(2))/dygrid)+1;
            positionIndex = Xcount*Ngridy+Ycount;
            %% ergodic motion calculations
            muk = zeros(nk, nk);
            for kx = 0:nk-1
                for ky = 0:nk-1
                    Dum=mu.*cos(kx/Dxsize*pi*Gridx).*cos(ky/Dysize*pi*Gridy)*dxgrid*dygrid;
                    muk(kx+1, ky+1) =  muk(kx+1, ky+1) + sum(sum(Dum));
                    Ck(kx+1, ky+1) = Ck(kx+1, ky+1) + cos(kx/Dxsize*pi*xprel)*cos(ky/Dysize*pi*yprel)*dt;
                end
            end
            
            kxx=0:nk-1; kyy=0:nk-1;
            [kxx,kyy]=meshgrid(kxx,kyy);
            lambdakk = 1.0./((1 + kxx.*kxx + kyy.*kyy).^(1.5));  lambdakk= lambdakk/(0.5*0.5);
            lambdakk(:,1)= lambdakk(:,1)*0.5;
            lambdakk(1,:)= lambdakk(1,:)*0.5;
            
            Bxx1=(-kxx*pi/Dxsize).*sin(kxx/Dxsize*pi*xprel).*cos(kyy/Dysize*pi*yprel);  Bxx1= Bxx1';
            Byy1=(-kyy*pi/Dysize).*cos(kxx/Dxsize*pi*xprel).*sin(kyy/Dysize*pi*yprel);  Byy1= Byy1';
            
            Bjxmat=lambdakk.*(Ck-time*muk).*Bxx1;
            Bjymat=lambdakk.*(Ck-time*muk).*Byy1;
            Bjx=sum(sum(Bjxmat));
            Bjy=sum(sum(Bjymat));
            
            Bjnorm = sqrt(Bjx^2+Bjy^2);
            u = -K*uavspeed*Bjx;
            v = -K*uavspeed*Bjy;
            Vnorm=sqrt(u^2+v^2);
            if Vnorm>uavspeed
                u=u*uavspeed/Vnorm;
                v=v*uavspeed/Vnorm;
            end
            obj.SMCParams.Ck=Ck;
        end
        
        function obj = headhome(obj,dt)
            %use this function to head towards new region. input agent
            %(obj) and dt (amount of time agent can move)
            %function adds new position at end and updates directions
            uavspeed = obj.uavmeanspeed;
            direction = obj.Directions;
            position = obj.pos(end,1:2);
            checker = sqrt(direction(1,1)^2+direction(1,2)^2);
            while checker < (dt*uavspeed)
                position = position + direction(1,:);
                dt = dt - sqrt(direction(1,1)^2+direction(1,2)^2)/uavspeed;
                direction(1,:) = [];
                if isempty(direction)
                    checker = dt*uavspeed+1;
                else
                    checker = sqrt(direction(1,1)^2+direction(1,2)^2);
                end
            end
            if ~isempty(direction)
                position = position + direction(1,:) * dt*uavspeed/sqrt(direction(1,1)^2+direction(1,2)^2);
                direction(1,:) = direction(1,:) - direction(1,:) * dt*uavspeed/sqrt(direction(1,1)^2+direction(1,2)^2);
            end
            obj.Directions = direction;
            obj.pos=[obj.pos; position];
            obj.posIndex = obj.findIndex(position);
        end
        
        function obj = DeletePoints(obj)
            %this function delete points from map that are not in mapPend
            toKeep = ismember(obj.Map.PointsIndices,obj.MapPend.PointsIndices);
            obj.Map.PointsIndices = obj.Map.PointsIndices(toKeep);
            obj.Map.xy = obj.Map.xy(toKeep,:);
            obj.Map.Densities = obj.Map.Densities(toKeep);
            obj.Map.Edges = obj.Map.Edges(toKeep,toKeep);
        end
        
        function plot(obj, color, transparancy)
            centerLogic = ismember(obj.Map.PointsIndices, obj.Center);
            centerx = obj.Map.xy(centerLogic,1);
            centery = obj.Map.xy(centerLogic,2);
            scatter(centerx,centery,250,color,'filled')
            
            patchesx = [];
            patchesy = [];
            
            %get patches for each point in region
            totalx = obj.Map.xy(:,1);
            totaly = obj.Map.xy(:,2);
            w = obj.Domain.dxgrid;
            h = obj.Domain.dygrid;
            for j = 1:length(totalx)
                TempX = [(totalx(j)-(w/2)) (totalx(j)+(w/2)) (totalx(j)+(w/2)) (totalx(j)-(w/2))];
                TempY = [(totaly(j)+(h/2)) (totaly(j)+(h/2)) (totaly(j)-(h/2)) (totaly(j)-(h/2))];
                patchesx(:,j) = TempX;
                patchesy(:,j) = TempY;
            end
            patch(patchesx,patchesy,color)
            alpha(transparancy)
%             plot(obj.pos(:,1),obj.pos(:,2),'color',color,'LineWidth',2)
%             plot(obj.pos(end,1),obj.pos(end,2),'color',color,'Marker', 'o')
        end
        
        function index = findIndex(obj,xy)
            Lmin=obj.Domain.Lmin;
            dxgrid=obj.Domain.dxgrid;
            dygrid=obj.Domain.dygrid;
            Ngridy=obj.Domain.Ngridy;
            
            Xcount = round((xy(1)-dxgrid/2)/dxgrid);
            Ycount = round((xy(2)-dygrid/2)/dygrid)+1;
            index = Ycount +Xcount * Ngridy;
        end
        
        
        
        
        
        
        
    end
end
