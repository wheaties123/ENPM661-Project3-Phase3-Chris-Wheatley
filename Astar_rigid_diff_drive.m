% Chris Wheatley
% ENPM661 Spring 2020
% Project #3 Phase #3

% This script will use the A* algorithm to simulate a rigid robot exploring 
%   an action space, starting from a user-specified
%   location all the way until the goal node is explored.  
%   Then the optimal path will be drawn. The robot is assigned differential
%   drive constraints (i.e. non-halonomic constraints).

close all; 
clear all;

% Solicit input node configuration from user
fprintf('\n');
prompt = 'Enter x,y START node location in meters and starting orientation in deg. (e.g. [1,1,30]): ';
start_node = input(prompt);
fprintf('\n');

% Solicit goal node configuration from user
fprintf('\n');
prompt1 = 'Enter x,y GOAL node location in meters (e.g. [10,9]): ';
goal_node = input(prompt1);
fprintf('\n');

% Calulated Paramaters From Datasheet
r=(354/2)/1000;
L=2*r;
wheelLength=pi*(76/1000);
wheelRad=(76/2)/1000;

% Solicit obstacle clearance from user
fprintf('\n');
prompt3 = 'Enter obstacle clearance (meters): ';
c = input(prompt3);
fprintf('\n');

% Solicit wheel RPMs from user
fprintf('\n');
prompt3 = 'Enter wheel RPMs [rpm1, rpm2]: ';
rpms = input(prompt3);
fprintf('\n');

xmin=-5; ymin=-5;
xmax=5; ymax=5;
fig=figure; hold on; axis equal;
% Define black border around action movement area
line([xmin xmax xmax xmin xmin],[ymin ymin ymax ymax ymin],'Color','black','LineWidth',2);
xlabel('X Coordinates (meters)'); ylabel('Y Coordinates (meters)');
title('A* Rigid (Non-Holonomic) (Pink Circle = Start; Pink Triangle = Goal; Yellow Circle = "Close Enough to Goal" Region)');

xl = [xmin xmax]; 
yl = [ymin ymax];

% Define obstacle space
% Left-most square
h1=line([-4.75 -4.75 -3.25 -3.25 -4.75],[-.75 .75 .75 -.75 -.75],'Color','black');

% Right-most square
h2=line([4.75 4.75 3.25 3.25 4.75],[-.75 .75 .75 -.75 -.75],'Color','black');

% Top-left square
h3=line([-2.75 -2.75 -1.25 -1.25 -2.75],[2.25 3.75 3.75 2.25 2.25],'Color','black');

 t=-pi:0.01:pi;

% Bottom-Left Circle 
xunit = cos(t) - 2;
yunit = sin(t) - 3;
h4=plot(xunit, yunit,'Color','black');

% Bottom-Right Circle 
xunit1 = cos(t) + 2;
yunit1 = sin(t) - 3;
h5=plot(xunit1, yunit1,'Color','black');

% Middle Circle 
xunit2 = cos(t);
yunit2 = sin(t);
h6=plot(xunit2, yunit2,'Color','black');

% Top-Right Circle 
xunit3 = cos(t) + 2;
yunit3 = sin(t) + 3;
h7=plot(xunit3, yunit3,'Color','black');

Obstacles=[h1 h2 h3 h4 h5 h6 h7];
%Obstacles=[h6];

startInObstacle = obstacleCheckRigid(Obstacles,start_node,r,c);
goalInObstacle = obstacleCheckRigid(Obstacles,goal_node,r,c);

if or(startInObstacle==1,or(or(start_node(1)>xmax,start_node(1)<xmin),or(start_node(2)>ymax,start_node(2)<ymin)))
    outside_obs_start=1;
    while or(outside_obs_start==1,or(or(start_node(1)>xmax,start_node(1)<xmin),or(start_node(2)>ymax,start_node(2)<ymin)))
        % Display message if start node falls outside of action space 
        fprintf('\n');
        disp('INAVLID START NODE LOCATION!');
        fprintf('\n');
        prompt = 'Enter new x,y STARTING node location (e.g. [0,0,30]), relative to bottom left corner of action space: ';
        start_node = input(prompt);
        outside_obs_start = obstacleCheckRigid(Obstacles,start_node,r,c);
    end
end

if or(goalInObstacle==1,or(or(goal_node(1)>xmax,goal_node(1)<xmin),or(goal_node(2)>ymax,goal_node(2)<ymin)))
    outside_obs_goal=1;
    while or(outside_obs_goal==1,or(or(goal_node(1)>xmax,goal_node(1)<xmin),or(goal_node(2)>ymax,goal_node(2)<ymin)))
        % Display message if goal node falls outside of action space 
        fprintf('\n');
        disp('INAVLID GOAL NODE LOCATION!');
        fprintf('\n');
        prompt = 'Enter new x,y GOAL node location (e.g. [10,9]), relative to bottom left corner of action space: ';
        goal_node = input(prompt);
        outside_obs_goal = obstacleCheckRigid(Obstacles,goal_node,r,c);
    end
end

% Start program run timer
tic

% Plot yellow circle around goal node, representing distance
% threshold/margin
th = 0:pi/50:2*pi;
x_circle = .5 * cos(th) + goal_node(1);
y_circle = .5 * sin(th) + goal_node(2);
plot(x_circle, y_circle);
fill(x_circle, y_circle, 'y');
drawnow

% Plot start and end point
plot(start_node(1), start_node(2),'mo','MarkerFaceColor','m');
plot(goal_node(1), goal_node(2),'m^','MarkerFaceColor','m');
uistack(fig,'top');

start_node(3)=start_node(3)*(pi/180);
% Initialize start node info
Nodes(1).x=start_node(1);
Nodes(1).y=start_node(2);
Nodes(1).Explored=0;
Nodes(1).ParentID=0;
Nodes(1).Theta=wrapTo2Pi(start_node(3));
Nodes(1).ID=1;
Nodes(1).Cost2Come=0;
Nodes(1).Cost2Go=sqrt((abs(start_node(1)-goal_node(1))^2)+(abs(start_node(2)-goal_node(2))^2));
Nodes(1).TotalCost=Nodes(1).Cost2Go+Nodes(1).Cost2Come;
Nodes(1).interpsX=0;
Nodes(1).interpsY=0;
Nodes(1).LeftRPM=0;
Nodes(1).RightRPM=0;

% Convert angle to 0-2*pi scale
start_node(3)=wrapTo2Pi(start_node(3));

% Toggle that states goal node has not been explored
goal_node_explored=0;

% Initialize node ID counter and Parent ID counter
i=1;
ParentIdx=1;

rpm_vals{1}=[0,rpms(1)];
rpm_vals{2}=[rpms(1),0];
rpm_vals{3}=[rpms(1),rpms(1)];
rpm_vals{4}=[0,rpms(2)];
rpm_vals{5}=[rpms(2),0];
rpm_vals{6}=[rpms(2),rpms(2)];
rpm_vals{7}=[rpms(1),rpms(2)];
rpm_vals{8}=[rpms(2),rpms(1)];
n=length(rpm_vals);

Xs=[]; Ys=[];
xx=start_node(1); yy=start_node(2);
startAngle=start_node(3);
firstIteration=1;
T=[];

% While goal node HAS NOT BEEN EXPLORED, perform every possible action and
%   record info in running structure
while goal_node_explored==0
    cost_ref=1000000000;
    if firstIteration==1
        firstIteration=0;
        firstIteration1=1;
    else      
        % Sort unexplored nodes by lowest cost2come+cost2go
        T = struct2table(Nodes); % convert the struct array to a table
        sortedT = sortrows(T, 'TotalCost'); % sort the table by 'TotalCost'
        toDelete = sortedT.Explored == 1;
        sortedT(toDelete,:) = [];
        sortedS = table2struct(sortedT); % change it back to struct array if necessary
        % Extract first node in the sorted queue
        if isempty(sortedS)==1
            fprintf('\n\n PROGRAM TERMINATED! NO FURTHER ACTIONS POSSIBLE! (interference w/obstacles or map boundary)')
            return 
        else
        end
        xx=sortedS(1).x; yy=sortedS(1).y; 
        ParentIdx=sortedS(1).ID; 
        startAngle=sortedS(1).Theta;
        isExplored=sortedS(1).Explored;
    end
    for k=1:1:length(rpm_vals)
        % Generate new node
        if goal_node_explored==0
        rpmLeft=rpm_vals{k}(1);
        rpmRight=rpm_vals{k}(2);
        [newX,newY,newTheta,interpX,interpY,runningDist] = changeAngle(xx,yy,startAngle,rpmLeft,rpmRight,wheelRad,L);
        
        if firstIteration1==1
            isExplored=0;
            firstIteration1=0;
        else
        end
        
        % Check if new node is in any of the obstacles
        outsideObstaclesANDBorder=obstacleCheckRigid(Obstacles,[newX,newY],r,c);
        
        checker=0; checker1=0; checker2=0; checker3=0; 
        for iiii=1:1:length(Obstacles)
            h=Obstacles(iiii);
            In = inpolygon(interpX,interpY,h.XData+r+c,h.YData+r+c);
            In1= inpolygon(interpX,interpY,h.XData+r+c,h.YData-r-c);
            In2= inpolygon(interpX,interpY,h.XData-r-c,h.YData+r+c);
            In3= inpolygon(interpX,interpY,h.XData-r-c,h.YData-r-c);
            checker=checker+sum(In);
            checker1=checker1+sum(In1);
            checker2=checker2+sum(In2);
            checker3=checker3+sum(In3);
        end
        if or(or(checker>=1,checker1>=1),or(checker2>=1,checker3>=1))==1
            interpolatedSegmentsInBorder=1;
        else
            interpolatedSegmentsInBorder=0;
        end
       
        Anglee=wrapTo2Pi(newTheta);
                    
        % Check if new node is within 0.1 distance of any other node
        if isempty(Xs)==1
            num=0;
        else
            Xstmp=Xs-newX; Ystmp=Ys-newY;
            num=sum(and(abs(Xstmp)<=.1,abs(Ystmp)<=.1));
        end
        
        if and(and(newX>=xmin,newY>=ymin),num==0)                
            closeEnough=sqrt((goal_node(1)-newX)^2+(goal_node(2)-newY)^2)<0.5;

            if and(and(and(outsideObstaclesANDBorder==0,interpolatedSegmentsInBorder==0),or(closeEnough==1,and(and(newX>=xmin,newX<=xmax),and(newY>=ymin,newY<=ymax))))==1,num==0)==1                
                cost2come=Nodes(ParentIdx).Cost2Come+runningDist;
                cost2go=sqrt((abs(newX-goal_node(1))^2)+(abs(newY-goal_node(2))^2));
                sumCost=cost2come+cost2go;

                if or(abs(cost2go)<=0.5,closeEnough==1)==1
                    goal_node_explored=1;
                else
                end

                i=i+1;
                drawnow
                xxx = interpX;
                yyy = interpY;
                % Cubic spline data interpolation
                tt = 1:numel(xxx);
                xy = [xxx;yyy];
                pp = spline(tt,xy);
                tInterp = linspace(1,numel(xxx));
                xyInterp = ppval(pp, tInterp);
                plot(xyInterp(1,:),xyInterp(2,:),'b-');
                Nodes(i).x=newX;
                Nodes(i).y=newY;
                Nodes(i).ParentID=ParentIdx;
                Nodes(i).ID=i;
                Nodes(i).Theta=Anglee;
                Nodes(i).Cost2Come=cost2come;
                Nodes(i).Cost2Go=cost2go;
                Nodes(i).TotalCost=sumCost;
                Nodes(i).Explored=0;
                Nodes(i).interpsX=interpX;
                Nodes(i).interpsY=interpY;
                Nodes(i).LeftRPM=rpmLeft;
                Nodes(i).RightRPM=rpmRight;
                Xs=[Xs newX]; Ys=[Ys newY];
            else
            end
        else
        end
        else
        end
    end
    
    Anglee=wrapTo2Pi(startAngle);
    
    % Mark node as "Explored"
    Nodes(ParentIdx).Explored=1;
end

% Backtrack to find optimal path, and plot it with a red line
backTrackingFinished=0;
k=0;

T = struct2table(Nodes); % convert the struct array to a table
toDelete = T.Explored == 0;
T(toDelete,:) = [];

node_idx=Nodes(end).ID;

while backTrackingFinished==0
    k=k+1;
    xxx = Nodes(node_idx).interpsX;
    yyy = Nodes(node_idx).interpsY;
    if or(and(xxx==0,yyy==0)==1,node_idx==1)
    else
        % Cubic spline data interpolation
        tt = 1:numel(xxx);
        xy = [xxx;yyy];
        pp = spline(tt,xy);
        tInterp = linspace(1,numel(xxx));
        xyInterp = ppval(pp, tInterp);
        plot(xyInterp(1,:),xyInterp(2,:),'g-','LineWidth',2);
    end
                    
    xVals(k)=Nodes(node_idx).x;
    yVals(k)=Nodes(node_idx).y;
    plot(Nodes(node_idx).x,Nodes(node_idx).y,'go','MarkerFaceColor','green','MarkerSize',5);
    nodeNum(k)=k;
    nodeIndex(k)=node_idx;
    leftWheelVel(k)=Nodes(node_idx).LeftRPM;
    rightWheelVel(k)=Nodes(node_idx).RightRPM;
    orientation(k)=Nodes(node_idx).Theta;
    node_idx=Nodes(node_idx).ParentID;
    
    if and(xVals(k)==start_node(1),yVals(k)==start_node(2))
        backTrackingFinished=1;
    else
    end
end

drawnow
plot(start_node(1), start_node(2),'mo','MarkerFaceColor','m');
plot(goal_node(1), goal_node(2),'m^','MarkerFaceColor','m');
drawnow
uistack(fig,'top');

X_Values=flip(xVals);
Y_Values=flip(yVals);
Left_Wheel_RPMs=flip(leftWheelVel);
Right_Wheel_RPMs=flip(rightWheelVel);
Orientation=flip(orientation);

x_val_input="x_vals={";
y_val_input="y_vals={";
left_vels_input="left_vels={";
right_vels_input="right_vels={";
orientation_input="orientations={";
for p=1:1:length(X_Values)
    x_val_input=strcat(x_val_input,string(X_Values(p)));
    y_val_input=strcat(y_val_input,string(Y_Values(p)));
    left_vels_input=strcat(left_vels_input,string(Left_Wheel_RPMs(p)));
    right_vels_input=strcat(right_vels_input,string(Right_Wheel_RPMs(p)));
    orientation_input=strcat(orientation_input,string(Orientation(p)));
    if p==length(X_Values)
        x_val_input=strcat(x_val_input,"};");
        y_val_input=strcat(y_val_input,"};");
        left_vels_input=strcat(left_vels_input,"};");
        right_vels_input=strcat(right_vels_input,"};");
        orientation_input=strcat(orientation_input,"};");
    else
        x_val_input=strcat(x_val_input,",");
        y_val_input=strcat(y_val_input,",");
        left_vels_input=strcat(left_vels_input,",");
        right_vels_input=strcat(right_vels_input,",");
        orientation_input=strcat(orientation_input,",");
    end
end
disp(x_val_input)
disp(y_val_input)
disp(left_vels_input)
disp(right_vels_input)
disp(orientation_input)

% End program run timer
toc
