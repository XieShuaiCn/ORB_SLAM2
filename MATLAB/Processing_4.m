%% Andre Ruas
close all; clc; clear;
j = 87;

disp("starting...")
while j <= 87
    
if j == 18 || j == 19 || j == 20 %excluding certain bags
    j = j + 1;
    continue;
end

fileName = strcat('test',num2str(j));

%filePath = strcat('~/Documents/recorded_bags', '/', fileName, '/', fileName, '.bag');
filePath = strcat('~/Documents/recorded_bags', '/', fileName, '.bag');
%filePath = strcat('recorded_bags', '/', fileName, '.bag');

bag = rosbag(filePath);
MoP = select(bag, 'Topic', '/diag/MoP'); %is it using m or pVelocity? =
TrackTime = select(bag, 'Topic', '/diag/TrackTime'); %how long does it take per frame = 
TwMMtime = select(bag, 'Topic', '/diag/TwMMtime'); %how long does take to twMM? = 
bagName = select(bag, 'Topic', '/diag/bagName'); %=
playbackRate = select(bag, 'Topic', '/diag/playbackRate');% =
%pointsTracked = select(bag, 'Topic', '/diag/pointsTracked');
timeSpentLost = select(bag, 'Topic', '/diag/timeSpentLost'); %=
trackLossCount = select(bag, 'Topic', '/diag/trackLossCount'); %=how many times was tracking lost?
trackingState = select(bag, 'Topic', '/diag/trackingState');
transC = select(bag, 'Topic', '/diag/transC');
transV = select(bag, 'Topic', '/diag/transV');

bothUsedFail = select(bag, 'Topic', '/diag/bothUsedFail');
bothUsedSuccess = select(bag, 'Topic', '/diag/bothUsedSuccess');
length = select(bag, 'Topic', '/diag/length');
numMatches = select(bag, 'Topic', '/diag/numMatches');
useBoth = select(bag, 'Topic', '/diag/useBoth');
useVicon = select(bag, 'Topic', '/diag/useVicon');

%disp("done reading bag...") 
MoP_msg = readMessages(MoP); %%once
TrackTime_msg = readMessages(TrackTime); %%avg%% -
TwMMtime_msg = readMessages(TwMMtime); %%avg%% -
bagName_msg = readMessages(bagName); %%once
playbackRate_msg = readMessages(playbackRate);  %%once
%pointsTracked_msg = readMessages(pointsTracked);
timeSpentLost_msg = readMessages(timeSpentLost); %%once
trackLossCount_msg = readMessages(trackLossCount); %%once
trackingState_msg = readMessages(trackingState); 
transC_msg = readMessages(transC); %%avg%%
transV_msg = readMessages(transV); %%avg%%

bothUsedFail_msg = readMessages(bothUsedFail); %%once
bothUsedSuccess_msg = readMessages(bothUsedSuccess); %%once
length_msg = readMessages(length); %%once
numMatches_msg = readMessages(numMatches); %%avg%%
useBoth_msg = readMessages(useBoth); %%once
useVicon_msg = readMessages(useVicon); %%once
 
G = size(transC_msg);
s = G(1,1);
distance = zeros(s,1);
distance_edited = zeros(s,1);
numMatches = zeros(s,1);
TrackTime = zeros(s,1);
TwMMtime = zeros(s,1);
trackingState = zeros(s,1);

for i = 1:s %looping through all messages in rosbag
xc = transC_msg{i,1}.Transform.Translation.X;
yc = transC_msg{i,1}.Transform.Translation.Y;
zc = transC_msg{i,1}.Transform.Translation.Z;

xv = transV_msg{i,1}.Transform.Translation.X;
yv = transV_msg{i,1}.Transform.Translation.Y;
zv = transV_msg{i,1}.Transform.Translation.Z;

distance(i,1) = ((xc-xv)^2 + (yc-yv)^2 + (zc-zv)^2)^(.5);

    if (trackingState_msg{i,1}.Data == 2)
        distance_edited(i,1) = ((xc-xv)^2 + (yc-yv)^2 + (zc-zv)^2)^(.5);
        if ((numMatches_msg{i,1}.Data < 1200) && (numMatches_msg{i,1}.Data > 0))
        numMatches(i,1) = numMatches_msg{i,1}.Data;
        end
    end
TrackTime(i,1) = TrackTime_msg{i,1}.Data;
TwMMtime(i,1) = TwMMtime_msg{i,1}.Data;
trackingState(i,1) = trackingState_msg{i,1}.Data;

end

%calculating averages for a particular rosbag
TrackTime_avg = mean(nonzeros(TrackTime));
TwMMtime_avg = mean(nonzeros(TwMMtime));
tSL = timeSpentLost_msg{i-1,1}.Data;
tLC = trackLossCount_msg{i-1,1}.Data;
dist_avg = mean(nonzeros(distance));
dist_edit = mean(distance_edited);
numMatches_avg = mean(nonzeros(numMatches));

useP = "false";
useV = "false";
useBoth = "false";
if (MoP_msg{i-1,1}.Data)
    useP = "true";
end
if (useVicon_msg{i-1,1}.Data) 
    useV = "true";
end
if (useBoth_msg{i-1,1}.Data) 
    useBoth = "true";
end
    
bUF = bothUsedFail_msg{i-1}.Data;
bUS = bothUsedSuccess_msg{i-1}.Data;

fprintf("useP/useV/useBoth for %s is %s/%s/%s \n", fileName, useP, useV, useBoth);
fprintf("The average frame duration for %s is %8.8f \n",fileName, TrackTime_avg);
fprintf("The average twMM duration for %s is %8.8f \n",fileName, TwMMtime_avg);
fprintf("The time spent lost for %s is %8.8f \n",fileName, tSL);
fprintf("The # of times lost for %s is %1.0f \n",fileName, tLC);
fprintf("The unedited average distance for %s is %8.8f \n",fileName, dist_avg);
fprintf("The edited average distance for %s is %8.8f \n",fileName, dist_edit);
fprintf("The # of UseBothFails for %s is %8.8f \n",fileName, bUF);
fprintf("The # of UseBothSuccesses for %s is %8.8f \n",fileName, bUS);
fprintf("The edited average numMatches for %s is %4.2f \n \n",fileName, numMatches_avg);
j = j + 1;

%plotting relevant information for rosbag
%x = 1:s;
%plot(x,trackingState);
%hold on;
end

%% generating figures
% 
% t = 1:length(transC_msg);
% 
% figure
% plot(t,distance)
% hold on
% plot(t,distance2)



disp("done");







