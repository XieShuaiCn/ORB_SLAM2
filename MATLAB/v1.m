fileName = 'test9';

filePath = strcat('~/Documents/recorded_bags', '/', fileName, '/', fileName, '.bag');

%bag = rosbag('~/Documents/recorded_bags/test30.bag')
%bag.AvailableTopics;

bag = rosbag(filePath);
transC = select(bag, 'Topic', '/error/transC'); 
transV = select(bag, 'Topic', '/error/transV');
rate = select(bag, 'Topic', '/error/rate');

disp("done reading bag...")

transC_msg = readMessages(transC);
transV_msg = readMessages(transV);
rate_msg = readMessages(rate);

%looping through transC and transV
s = length(transC_msg);
distance = zeros(s,1);

for i = 1:s
xc = transC_msg{i,1}.Transform.Translation.X;
yc = transC_msg{i,1}.Transform.Translation.Y;
zc = transC_msg{i,1}.Transform.Translation.Z;

xv = transV_msg{i,1}.Transform.Translation.X;
yv = transV_msg{i,1}.Transform.Translation.Y;
zv = transV_msg{i,1}.Transform.Translation.Z;

distance(i,1) = ((xc-xv)^2 + (yc-yv)^2 + (zc-zv)^2)^(.5);

end

dist_avg = mean(distance);

disp("done");

%% generating figures

t = 1:length(transC_msg);

plot(t,distance)












