function Agent=smc_update(time,Agent,Domain,SimParams)
%this function takes in time, Agent, Domain(total region and also
%?restricted region?), sim Params

%returns agent with updated location.

%% initialization
Lmin=Domain.Lmin;
Lmax=Domain.Lmax;
Dxsize=Domain.Dxsize;
Dysize=Domain.Dysize;
dxgrid=Domain.dxgrid;
dygrid=Domain.dygrid;
Ngridx=Domain.Ngridx;
Ngridy=Domain.Ngridy;
poly=Domain.poly;
dt=SimParams.dt;

uavspeed=Agent.uavmeanspeed;
nk=Agent.SMCParams.nk;
K=Agent.SMCParams.K;
Ck=Agent.SMCParams.Ck;
mu=Agent.mu; %has to defined using meshgrid structure, otherwise will cause issues
agentspos=Agent.pos(end,1:2);

[Gridx,Gridy] = meshgrid(linspace(Lmin(1),Lmax(1),Ngridx),linspace(Lmin(2),Lmax(2),Ngridy));
xprel = agentspos(1) - Lmin(1);
yprel = agentspos(2) - Lmin(2);
%% see if agent is in its new region

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
%% boundary check and correction
% this produces transpose compare to looping as in lines 25-34 as using meshgrid in line 40, so take
% transpose of Bxx1 and Byy1 before using with Ck and muk

%if (agentspos(1,1) < Lmin(1) || agentspos(1,1) > Lmax(1) || agentspos(1,2) < Lmin(2) || agentspos(1,2) > Lmax(2))
%   Bjx=agentspos(1,1)-(Lmin(1)+Lmax(1))/2;
%     Bjy=agentspos(1,2)-(Lmin(2)+Lmax(2))/2;
if ~inpolygon(agentspos(1,1),agentspos(1,2),poly.polyx,poly.polyy)
    Bjx=agentspos(1,1)-poly.cenx;
    Bjy=agentspos(1,2)-poly.ceny;
else
    
    Bxx1=(-kxx*pi/Dxsize).*sin(kxx/Dxsize*pi*xprel).*cos(kyy/Dysize*pi*yprel);  Bxx1= Bxx1';
    Byy1=(-kyy*pi/Dysize).*cos(kxx/Dxsize*pi*xprel).*sin(kyy/Dysize*pi*yprel);  Byy1= Byy1';
    
    Bjxmat=lambdakk.*(Ck-time*muk).*Bxx1;
    Bjymat=lambdakk.*(Ck-time*muk).*Byy1;
    Bjx=sum(sum(Bjxmat));
    Bjy=sum(sum(Bjymat));  
    disp('in*********')
end


if Agent.SMCParas.VModeltype==1
    
    Bjnorm = sqrt(Bjx^2+Bjy^2);
    if 0
        u = -uavspeed*Bjx/Bjnorm;
        v = -uavspeed*Bjy/Bjnorm;
    else
        u = -K*uavspeed*Bjx;
        v = -K*uavspeed*Bjy;
        Vnorm=sqrt(u^2+v^2);
        if Vnorm>uavspeed
            u=u*uavspeed/Vnorm;
            v=v*uavspeed/Vnorm;
        end
    end
    agentsposnew(1, :) = agentspos(1, :) + [u v] * dt;
    phinew=0;
else
    uavminspeed=Agent.uavminspeed;
    uavmaxspeed=Agent.uavmaxspeed;
    wmin=Agent.wmin;
    wmax=Agent.wmax;
    
    phi=Agent.pos(end,3);
    
    A = Bjx*cos(phi) + Bjy*sin(phi);
    
    if (A>0)
        vspeed =uavminspeed;
    else
        vspeed =uavmaxspeed;
    end
    
    B = Bjx*(-sin(phi)) + Bjy*(cos(phi));
    
    if (B>0)
        w=wmin;
    else
        w=wmax;
    end
    
    agentsposnew(1, :)= agentspos(1, :) + vspeed*[cos(phi) sin(phi)]*dt;
    phinew = phi+ w*dt;
    
end

Agent.SMCParams.Ck=Ck;
Agent.pos=[Agent.pos; agentsposnew phinew];