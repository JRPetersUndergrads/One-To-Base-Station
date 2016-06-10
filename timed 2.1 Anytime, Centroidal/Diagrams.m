

clc;
close all;
clear all;

%Replace the following lines with wherever your data is saved! It does not
%need to be ordered.
load StepsAndCompTime_SingleSys_Bugfixed.mat %This is the file that contains the data created in DataMinerMk1.m

Norm = VideoReader('AgentsAnimate_Norm_Clear.avi');%These are video files which
Cent = VideoReader('AgentsAnimate_Cent_Clear.avi');
Any = VideoReader('AgentsAnimate_Any_Clear.avi');

Initial = read(Norm,1);

NormMid = read(Norm,191); %These values should be some interesting point in the evolution.
CentMid = read(Cent,67); %It probably makes sense to chose similar values for each.
AnyMid = read(Any,125);

NormLast = read(Norm,Inf);
CentLast = read(Cent,Inf);
AnyLast = read(Any,Inf);

figure(1)
subplot(3,3,4); image(Initial); axis off; title('Given Initial State'); 
 

subplot(3,3,2); image(NormMid); title('Intermediate Configuration'); box off;
box off; set(gca, 'XTick', []); set(gca, 'YTick', []); ylabel('Base Algorithim');

subplot(3,3,5); image(CentMid); box off; box off; set(gca, 'XTick', []); set(gca, 'YTick', []); ylabel('Centroidal');
subplot(3,3,8); image(AnyMid); box off; box off; set(gca, 'XTick', []); set(gca, 'YTick', []); ylabel('Anytime')

subplot(3,3,3); image(NormLast); axis off; title('Final Configuration')
subplot(3,3,6); image(CentLast); axis off;
subplot(3,3,9); image(AnyLast); axis off;

subplot(3,3,1); axis off;  set(gca, 'Color', 'None');

figure(2)
% CompTime_Norm = CompTime_Norm(CompTime_Norm>=0.01);
% CompTime_Cent = CompTime_Cent(CompTime_Cent>=0.01);
% CompTime_Any = CompTime_Any(CompTime_Any>=0.01);
N = cell(size(CompTime_Norm)); N(1:end) = {'Base Algorithm'};
C = cell(size(CompTime_Cent)); C(1:end) = {'Centroidal Algorithm'};
A = cell(size(CompTime_Any)); A(1:end) = {'Anytime Algorithm'};
boxplot([CompTime_Norm;CompTime_Cent;CompTime_Any],[N;C;A])
title('Computaton times Per Algorithm');
ylabel('Update Computation Time (s)');

% h=findobj(gca,'tag','Outliers'); delete(h); axis auto;

figure(3)
N = cell(size(Steps2Comp_Norm)); N(1:end) = {'Base Algorithm'};
C = cell(size(Steps2Comp_Cent)); C(1:end) = {'Centroidal Algorithm'};
A = cell(size(Steps2Comp_Any)); A(1:end) = {'Anytime Algorithm'};
boxplot([Steps2Comp_Norm;Steps2Comp_Cent;Steps2Comp_Any],[N;C;A])
title('Steps Until Convergence');
ylabel('Number of Steps');

