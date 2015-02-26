function launch()

% Copyright (C) 2013 Georgia Tech Research Corporation
% see the LICENSE file included with this software

clear java;
clear classes;
close all

if (isdeployed)
    [path, folder, ~] = fileparts(ctfroot);
    root_path = fullfile(path, folder);
else
    root_path = fileparts(mfilename('fullpath'));
end
addpath(genpath(root_path));

javaaddpath(fullfile(root_path, 'java'));

app = simiam.ui.AppWindow(root_path, 'launcher');


   
app.load_ui();
%loop
for i = 1:2
    %swap out .xml map for initial conditions 
    settings_file = strcat('settings', num2str(i), '.xml');
    
    %run the program
    app.ui_button_start([],[], settings_file); %this loads simulator & does inital conditions
    for z=1:10
        app.ui_button_zoom_out([],[]);
    end
    app.simulator_.stop();
    
    %% do something to change variables. You'll have access to pretty much
    %  anything you want since it's all been created 
    
    % set clocky's percent randomness
    app.simulator_.world.robots.elementAt(1).supervisor.set_percent_random(.1);
    
    % set clockys sensor gains
    gains = [1 1 1 1 1];
    app.simulator_.world.robots.elementAt(1).supervisor.controllers{5}.set_sensor_gains(gains);
    
    
%     app.simulator_.world.robots.elementAt(1).robot.dynamics
%     app.simulator_.world.robots.elementAt(2).pose
    
    %% re-start simulation
    app.simulator_.start();
    
    %detect collision or game ender
    pause(1);
    
    %save variables
    clockyFinalx(i) = app.simulator_.world.robots.elementAt(1).pose.x;
    clockyFinaly(i) = app.simulator_.world.robots.elementAt(1).pose.y;
    humanFinalx(i) = app.simulator_.world.robots.elementAt(2).pose.x;
    humanFinaly(i) = app.simulator_.world.robots.elementAt(2).pose.y;
    finalTime(i) = 0.05*get(app.simulator_.clock, 'TasksExecuted');
    
    %go to 'home'
    app.ui_button_home([],[]);
end
%% export to workspace
putvar(clockyFinalx);
putvar(clockyFinaly);
putvar(humanFinalx);
putvar(humanFinaly);
putvar(finalTime);

%% plot

close all
figure(1)
scatter(clockyFinalx, clockyFinaly)
hold on
scatter(humanFinalx, humanFinaly)
legend('clocky', 'human')

end
