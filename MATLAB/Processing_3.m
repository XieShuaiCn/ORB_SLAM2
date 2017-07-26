%% Andre Ruas
close all; clc; clear;
j = 44;

disp("starting...")
while j <= 45
    
if j == 18 || j == 19 || j == 20 %excluding certain bags
    j = j + 1;
    continue;
end

fileName = strcat('test',num2str(j));

%filePath = strcat('~/Documents/recorded_bags', '/', fileName, '/', fileName, '.bag');
filePath = strcat('~/Documents/recorded_bags', '/', fileName, '.bag');

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

%looping through transC and transV
s = length(transC_msg);
distance = zeros(s,1);
distance_edited = zeros(s,1);
TrackTime = zeros(s,1);
TwMMtime = zeros(s,1);

for i = 1:s
xc = transC_msg{i,1}.Transform.Translation.X;
yc = transC_msg{i,1}.Transform.Translation.Y;
zc = transC_msg{i,1}.Transform.Translation.Z;

xv = transV_msg{i,1}.Transform.Translation.X;
yv = transV_msg{i,1}.Transform.Translation.Y;
zv = transV_msg{i,1}.Transform.Translation.Z;

distance(i,1) = ((xc-xv)^2 + (yc-yv)^2 + (zc-zv)^2)^(.5);

    if (trackingState_msg{i,1}.Data == 2)
        distance_edited(i,1) = ((xc-xv)^2 + (yc-yv)^2 + (zc-zv)^2)^(.5);
    end
TrackTime(i,1) = TrackTime_msg{i,1}.Data;
TwMMtime(i,1) = TwMMtime_msg{i,1}.Data;

end

TrackTime_avg = mean(TrackTime);
TwMMtime_avg = mean(TwMMtime);
tSL = timeSpentLost_msg{i-1,1}.Data;
tLC = trackLossCount_msg{i-1,1}.Data;
dist_avg = mean(distance);
dist_edit = mean(distance_edited);

fprintf("The average frame duration for %s is %8.8f \n",fileName, TrackTime_avg);
fprintf("The average twMM duration for %s is %8.8f \n",fileName, TwMMtime_avg);
fprintf("The time spent lost for %s is %8.8f \n",fileName, tSL);
fprintf("The # of times lost for %s is %1.0f \n",fileName, tLC);
fprintf("The unedited average distance for %s is %8.8f \n",fileName, dist_avg);
fprintf("The edited average distance for %s is %8.8f \n \n",fileName, dist_edit);
j = j + 1;

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







