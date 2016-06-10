n = 20;
CompTime_Norm = [];
CompTime_Cent = [];
CompTime_Any = [];

Steps2Comp_Norm = [];
Steps2Comp_Cent = [];
Steps2Comp_Any = [];

for i = 1:n
    rng('shuffle'); %Reset RNG so things are random
    timed2_1
    CompTime_Norm = [CompTime_Norm;updateCompTime];
    Steps2Comp_Norm = [Steps2Comp_Norm;RealupdateSteps];
    
    timed2_1_Centroidal
    CompTime_Cent = [CompTime_Cent;updateCompTime];
    Steps2Comp_Cent = [Steps2Comp_Cent;RealupdateSteps];

    timed2_1_Anytime
    CompTime_Any = [CompTime_Any;updateCompTime];
    Steps2Comp_Any = [Steps2Comp_Any;RealupdateSteps];
end

close(BaseAnimate)
close(AgentsAnimate)