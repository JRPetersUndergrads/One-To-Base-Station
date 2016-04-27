classdef SMC
    
    properties
        Ck;
        nk;
        K;
        DSMCOn;
        VModeltype;
    end
    methods
        %Constructor
        function obj = SMC(Ck,nk,K,DSMCOn,VModeltype)
            p = inputParser;
            p.KeepUnmatched = 1;
            defaultMaxSpeed = {};
            defaultMaxAcceleration = [];
            addRequired(p, 'Ck');
            addRequired(p, 'nk');
            addRequired(p, 'K');
            addRequired(p, 'DSMCOn');
            addRequired(p, 'VModeltype');
            parse(p,Ck,nk,K,DSMCOn,VModeltype);
            obj.Ck = p.Results.Ck;
            obj.nk = p.Results.nk;
            obj.K = p.Results.K;
            obj.DSMCOn = p.Results.DSMCOn;
            obj.VModeltype = p.Results.VModeltype;
        end
    end
end