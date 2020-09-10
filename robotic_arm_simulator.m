clc
clear
%------INPUT VALUES------
shouldRecalculateAll = false; %if true, recalculate all effector locations
shouldRecalculateArm = false; %if true, recalculate displayed arm orientation
%segment legnths in cm
baseL = 8;
L1 = 12.5;
L2 = 12.5;
L3 = 18.5;
%joint orientations
theta1 = 60;
theta2 = -25;
theta3 = -80;

%------LOAD DATA FROM FILE------
if shouldRecalculateAll == false
    if shouldRecalculateArm == true
        %don't load saved arm lengths and theta values if
        %we are recalculating the arm
       load armDataValues.mat -regexp ^(?!(baseL|L1|L2|L3|theta1|theta2|theta3)$).
    else
        load('armDataValues.mat')
    end
    fprintf('Data has been loaded\n')
end


%------ARM CALCULATIONS------
if shouldRecalculateArm == true || shouldRecalculateAll == true
    fprintf('Calculating Arm\n')
    %trig to find out how much of each segment is in x&y directions
    seg1X = L1*cosd(theta1);
    seg1Y = L1*sind(theta1);
    seg2X = L2*cosd(theta1+theta2);
    seg2Y = L2*sind(theta1+theta2);
    seg3X = L3*cosd(theta1+theta2+theta3);
    seg3Y = L3*sind(theta1+theta2+theta3);

    %coordinates of each joint's location
    b1 = [0 0];
    j1 = [0 baseL];
    j2 = [(j1(1)+seg1X) (j1(2)+seg1Y)];
    j3 = [(j2(1)+seg2X) (j2(2)+seg2Y)];
    %end effector location
    effector = [(j3(1)+seg3X) (j3(2)+seg3Y)];

    %joints which will be marked with a circle in the figure
    markJointsX = [j1(1) j2(1) j3(1)];
    markJointsY = [j1(2) j2(2) j3(2)];
end


%------CALCULATE ALL POTENTIAL END EFFECTOR LOCATIONS------
if shouldRecalculateAll == true
    fprintf('Calculating Work Envelope\n')
    locX1 = [];
    locY1 = [];
    locX2 = [];
    locY2 = [];
    locX3 = [];
    locY3 = [];
    a1 = [];
    a2 = [];
    a3 = [];
    angles = [0:3:180];

    %calculations
    for ang1 = angles
        %all locations of joint2
        locX1 = [locX1 L1*cosd(ang1)];
        locY1 = [locY1 (L1*sind(ang1)+baseL)];
        for ang2 = angles-(90-ang1) %adjust for orientation of segment1
            %all locations of end effector
            locX2 = [locX2 (L2*cosd(ang2)+locX1(end))];
            locY2 = [locY2 (L2*sind(ang2)+locY1(end))];
            for ang3 = angles-(90-ang2) %adjust for orientation of segment2
                locX3 = [locX3 (L3*cosd(ang3)+locX2(end))];
                locY3 = [locY3 (L3*sind(ang3)+locY2(end))];
                a1 = [a1 ang1]; %angle of servo at joint1
                a2 = [a2 (ang2+(90-ang1))]; %local angle of servo at joint2
                a3 = [a3 (ang3+(90-ang2))]; %local angle of servo at joint3
            end
        end
    end
    locXY1 = [locX1;locY1];
    locXY2 = [locX2;locY2];
    locXY3 = [locX3;locY3;a1;a2;a3]; %matrix with effector location and servo angles   
end


%---CREATE A FIGURE WHICH WILL HAVE THE ARM CONFIGURATION PLOT ON IT----
figure('Name','Forward Kinematics','NumberTitle','off',...
    'Position',[10 10 1200 600]);
movegui(gcf,'center') %center the figure on screen

yline(0,'--k') %draws the ground
xlim([-50 50]) %x axis bounds
ylim([-30 55]) %y axis bounds
hold on
plot([b1(1) j1(1)],[b1(2) j1(2)],'LineWidth',2) %fixed "base" segment
plot([j1(1) j2(1)],[j1(2) j2(2)],'LineWidth',2) %first segment
plot([j2(1) j3(1)],[j2(2) j3(2)],'LineWidth',2) %second segment
plot([j3(1) effector(1)],[j3(2) effector(2)],'LineWidth',2)%third segment

%draw marks on the joints and end effector
scatter(markJointsX,markJointsY,50,'k','Filled')
scatter(effector(1),effector(2),80,'g','Filled','s')

%marks all locations that can be reached
%scatter(locXY1(1,:),locXY1(2,:),1,[0 1 0],'Filled')%joint2
%scatter(locXY2(1,:),locXY2(2,:),1,[1 0 0],'Filled')%joint3
scatter(locXY3(1,:),locXY3(2,:),1,[0 0 1],'Filled')%end effector

%title, axis, and label info
title('Orientation of Robotic Arm');
xlabel('X Position (cm)','fontsize',14);
ylabel('Y Position (cm)','fontsize',14);


%------SAVE DATA TO FILE------
if shouldRecalculateAll == true || shouldRecalculateArm == true || ~isfile('armDataValues.mat')
    if ~isfile('testVals.mat') %print if no save file exists
        fprintf('No save file exists! Creating new file...\n');
    end
    save armDataValues.mat -regexp ^(?!(shouldRecalculateArm|shouldRecalculateAll)$).
    fprintf('Data has been saved\n')
end

fprintf('The program has finished executing\n')