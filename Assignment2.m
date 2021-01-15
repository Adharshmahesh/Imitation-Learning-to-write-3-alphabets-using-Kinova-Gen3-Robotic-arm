clc;
clear all;
load('I.mat');
letter_1 = demos;
load('L.mat');
letter_2 = demos;
load('M.mat')
letter_3 = demos;

num_trajectory = length(letter_1);
num_trajectory1 = length(letter_2);
num_trajectory2 = length(letter_3);

num_datapoints = size(letter_1{1}.pos,2);
num_datapoints1 = size(letter_2{1}.pos,2);
num_datapoints2 = size(letter_3{1}.pos,2);

Data = [];
Data1 = [];
Data2 = [];

for n=1:num_trajectory-1
   s(n).Data = spline(1:size(letter_1{n}.pos,2), letter_1{n}.pos, linspace(1,size(letter_1{n}.pos,2), num_datapoints)); 
   v = spline(1:size(letter_1{n}.vel,2), letter_1{n}.vel, linspace(1, size(letter_1{n}.vel,2), num_datapoints));
   Data = [Data [s(n).Data; v]];
end

DataIn = spline(1:size(letter_1{end}.pos,2), letter_1{end}.pos, linspace(1, size(letter_1{end}.pos,2), num_datapoints));
DataOut = spline(1:size(letter_1{end}.vel,2), letter_1{end}.vel, linspace(1, size(letter_1{end}.vel,2), num_datapoints));

for n=1:num_trajectory1-1
   s1(n).Data1 = spline(1:size(letter_2{n}.pos,2), letter_2{n}.pos, linspace(1,size(letter_2{n}.pos,2), num_datapoints1)); 
   v1 = spline(1:size(letter_2{n}.vel,2), letter_2{n}.vel, linspace(1, size(letter_2{n}.vel,2), num_datapoints1));
   Data1 = [Data1 [s1(n).Data1; v1]];
end

DataIn1 = spline(1:size(letter_2{end}.pos,2), letter_2{end}.pos, linspace(1, size(letter_2{end}.pos,2), num_datapoints1));
DataOut1 = spline(1:size(letter_2{end}.vel,2), letter_2{end}.vel, linspace(1, size(letter_2{end}.vel,2), num_datapoints1));

for n=1:num_trajectory2-1
   s2(n).Data2 = spline(1:size(letter_3{n}.pos,2), letter_3{n}.pos, linspace(1,size(letter_3{n}.pos,2), num_datapoints2)); 
   v2 = spline(1:size(letter_3{n}.vel,2), letter_3{n}.vel, linspace(1, size(letter_3{n}.vel,2), num_datapoints2));
   Data2 = [Data2 [s2(n).Data2; v2]];
end

DataIn2 = spline(1:size(letter_3{end}.pos,2), letter_3{end}.pos, linspace(1, size(letter_3{end}.pos,2), num_datapoints2));
DataOut2 = spline(1:size(letter_3{end}.vel,2), letter_3{end}.vel, linspace(1, size(letter_3{end}.vel,2), num_datapoints2));


% Visualize the data
figure(1), clf, hold on;
    plot(Data(1,:), Data(2,:), '.', 'markersize', 8, 'color', [.7, .7, .7]);
    quiver(Data(1,:), Data(2,:), Data(3,:), Data(4,:))
    title('Training Data');
    hold off
    
figure(2), clf, hold on;
    quiver(DataIn(1,:), DataIn(2,:), DataOut(1,:), DataOut(2,:));
    title('Testing data')
    hold off

model.nbStates = 5;
model.nbVar = 4;
model = init_GMM_kmeans(Data, model);
disp(model.Mu)

model = EM_GMM(Data, model);
%disp(model)
figure(3), hold on, axis off;
plot(Data(1,:), Data(2,:), '.', 'markersize', 8, 'color', [0.5, 0.5, 0.5]);
axis equal;
set(gca, 'Xtick', []);
set(gca, 'Ytick', []);
title('Learnt GMM')
hold off

in = 1:2;
out = 3:4;
[DataO, SigmaOut] = GMR(model, DataIn, in, out);

figure(4), hold on, 
quiver(DataIn(1,:), DataIn(2,:), DataO(1,:), DataO(2, :))
hold off

model1.nbStates = 5;
model1.nbVar = 4;
model1 = init_GMM_kmeans(Data1, model1);
disp(model1.Mu)

model1 = EM_GMM(Data1, model1);
%disp(model)
figure(3), hold on, axis off;
plot(Data1(1,:), Data1(2,:), '.', 'markersize', 8, 'color', [0.5, 0.5, 0.5]);
axis equal;
set(gca, 'Xtick', []);
set(gca, 'Ytick', []);
title('Learnt GMM')
hold off

in1 = 1:2;
out1 = 3:4;
[DataO1, ~] = GMR(model1, DataIn1, in, out1);

figure(4), hold on, 
quiver(DataIn1(1,:), DataIn1(2,:), DataO1(1,:), DataO1(2, :))
hold off

model2.nbStates = 5;
model2.nbVar = 4;
model2 = init_GMM_kmeans(Data2, model2);
disp(model2.Mu)

model2 = EM_GMM(Data2, model2);
%disp(model)
figure(3), hold on, axis off;
plot(Data2(1,:), Data2(2,:), '.', 'markersize', 8, 'color', [0.5, 0.5, 0.5]);
axis equal;
set(gca, 'Xtick', []);
set(gca, 'Ytick', []);
title('Learnt GMM')
hold off

in2 = 1:2;
out2 = 3:4;
[DataO2, ~] = GMR(model2, DataIn2, in2, out2);

figure(4), hold on, 
quiver(DataIn2(1,:), DataIn2(2,:), DataO2(1,:), DataO2(2, :))
hold off

robot = loadrobot('kinovaGen3', 'DataFormat', 'column');
endEffector = robot.BodyNames{end};
dim_steps = 200;
q = robot.homeConfiguration;
q(2) = pi/4 ;
q(4) = pi/4 ;
q(6) = pi/4 ;

q1 = robot.homeConfiguration;
q1(2) = -pi/3 ;
q1(4) = -pi/3 ;
q1(6) = -pi/3 ;

q2 = robot.homeConfiguration;
q2(2) = pi/3;
q2(4) = pi/3;
q2(6) = pi/3;

[letter_1_pos, letter_1_joint_pos] = draw_C(robot, q, DataO);

figure(1),
robot.show(letter_1_joint_pos(:,1), 'PreservePlot', false, 'Frames', 'off') ; 

hold on,
            
for i = 1 : dim_steps
            vel = letter_1_joint_pos(:,i) ;
            robot.show(vel, 'PreservePlot', false, 'Frames','off') ;
            plot3(letter_1_pos(i,1),letter_1_pos(i,2),letter_1_pos(i,3),'b.', 'MarkerSize', 5);
            drawnow;
end

[letter_2_pos, letter_2_joint_pos] = draw_C(robot, q1, DataO1);

for m = 1 : dim_steps
            vel_letter_2 = letter_2_joint_pos(:,m) ;
            robot.show(vel_letter_2, 'PreservePlot', false, 'Frames','off') ;
            plot3(letter_2_pos(m,1),letter_2_pos(m,2),letter_2_pos(m,3),'b.', 'MarkerSize', 5);
            drawnow;
end

[letter_3_pos, joint_letter] = draw_C(robot, q2, DataO2);

for m = 1 : dim_steps
            vel_letter_3 = joint_letter(:,m) ;
            robot.show(vel_letter_3, 'PreservePlot', false, 'Frames','off') ;
            plot3(letter_3_pos(m,1),letter_3_pos(m,2),letter_3_pos(m,3),'b.', 'MarkerSize', 5);
            drawnow;
end

hold off;
