clc, clear all, close all

%%%%%%%%%%% case 1 %%%%%%%%%%%%%%%%%%%
% Set up the initial positions of the red ship and obstacle
init_position = [0, 20];    %[0, 20]
goal_position = [0, -10];   %[0, -10]

obstacle_position = [-10, 5];  %[-10, 5]


% Set up the velocity vectors for the red ship and obstacle
V0 = 3;
red_ship_velocity = V0;
obstacle_velocity = [2 , 0];


%initial position of red ship
red_ship_position = init_position;

%calculate initial heading angle
heading0 = atan2(goal_position(2) - init_position(2), goal_position(1) - init_position(1));
heading = heading0;



% % %%%%%%%%%%%%% case 2 %%%%%%%%%%%%%%%%%%%%%%%
% Set up the initial positions of the red ship and obstacle
%init_position = [0, -10];
%goal_position = [0, 20];
 
%obstacle_position = [-10, 5];


% Set up the velocity vectors for the red ship and obstacle
%V0 = 3;
%red_ship_velocity = V0;
%obstacle_velocity = [2 , 0];

 
%initial position of red ship
%red_ship_position = init_position;
 
%calculate initial heading angle
%heading0 = atan2(goal_position(2) - init_position(2), goal_position(1) - init_position(1));
%heading = heading0;


% % %%%%%%%%%%%%% case 3 %%%%%%%%%%%%%%%%%%%%%%%
% % % Set up the initial positions of the red ship and obstacle
%init_position = [-10, 10];
%goal_position = [11, -11];
% % 
%obstacle_position = [-6.5, -7.5];
% % 
% % 
% % % Set up the velocity vectors for the red ship and obstacle
%V0 = 3;
%red_ship_velocity = V0;
%obstacle_velocity = [1.4 , 1.4];
% % 
% % 
% % %initial position of red ship
%red_ship_position = init_position;
% % 
% % %calculate initial heading angle
%heading0 = atan2(goal_position(2) - init_position(2), goal_position(1) - init_position(1));
%heading = heading0;



% Set up the time step and the duration of the simulation
dt = 0.1;  % 0.1

% Set up the path array to store the positions of the red ship during movement
path = [red_ship_position(1), red_ship_position(2)];

cir_angle = 0;    %angle of circle path for red ship
cir_flag = 0;     %Status on whether the reship is on a circular path
obstacle_pass = 0;%Status on whether the obstacle passes or not
reft_pass = 0 ;   %status on whether the red ship move to left
frames = [];

% Loop over until red ship reaches to goal point
while norm(red_ship_position - goal_position)>0.4
    
    heading_angle = heading * 180/pi

    % Update the positions of the red ship and obstacle based on their velocities
    red_ship_position(1) = red_ship_position(1) + red_ship_velocity * cos(heading) * dt;
    red_ship_position(2) = red_ship_position(2) + red_ship_velocity * sin(heading) * dt;  

    obstacle_position = obstacle_position + obstacle_velocity .* dt;


    % Determine the distance between the red ship and the obstacle
    distance = norm(red_ship_position- obstacle_position);   
    
    
    %% form path to avoid the obstacle
    if distance < 8 && cir_flag ==0 && obstacle_pass ==0   %if distance < 8 && cir_flag ==0 && obstacle_pass ==0
         cir_flag =1;
       
         %determine the obtable on left or right of path 
         d = (obstacle_position(1) - init_position(1))*(goal_position(2)- init_position(2)) -...
             (obstacle_position(2) - init_position(2))*(goal_position(1)- init_position(1));
         
         if d >= 0
             reft_pass = 0;
         else
             reft_pass = 1;
         end
      
         
    end
    
    %Update the heading angle of the red ship to avoid the obstacle
    if cir_flag ==1
        
        if reft_pass == 0
                        
             %calculate the disired the heading angle
             if cir_angle > (pi)
                 
                 d_heading = heading0;                   %disired heading angle
             else
                 
                 d_heading = heading0 + cir_angle -pi/2 ;%disired heading angle     
             end
            
            %caculate the heading angle 
            if d_heading >heading
                heading = heading +0.055;
            else
                heading = heading -0.053;       
            end

        else
             %calculate the disired the heading angle
             if cir_angle > (pi)
                 
                 d_heading = heading0;                   %disired heading angle
             else
                 
                 d_heading = heading0 - cir_angle + pi/2 ;%disired heading angle    
             end
            
            %caculate the heading angle 
            if d_heading >heading
                heading = heading + 0.055;
            else
                heading = heading - 0.053;
            end

        end
        
        %update angle of circle path
        cir_angle = cir_angle + dt;
                
        if cir_angle>pi+pi/1.2           %if cir_angle>pi+pi/2
            cir_flag = 0;
            obstacle_pass = 1;
        end
        
    else
        red_ship_velocity = V0;
        heading = heading0;   
        
    end
     

    % Store the position of the red ship in the path array
    path = [path; [red_ship_position(1),red_ship_position(2)]];

    % Plot the positions of the red ship and obstacle
    plot(path(:,1), path(:,2), 'r--')
    hold on
    plot(red_ship_position(1), red_ship_position(2), 'ro', 'MarkerSize', 10)
    plot(obstacle_position(1), obstacle_position(2), 'bo', 'MarkerSize', 10)
    xlim([-15, 25])
    ylim([-15, 25])
    pause(0.1)
    hold off
  drawnow;
    frame = getframe(gcf);
    frames = [frames, frame];

end

% Save the frames as a GIF
filename = 'different_angle_of_approach.gif';
for t = 1:length(frames)
    % Convert the current frame to indexed format
    [ind, map] = rgb2ind(frames(t).cdata, 256);
    % Write the indexed image to the GIF file
    if t == 1
        % If this is the first frame, create the file
        imwrite(ind, map, filename, 'gif', 'LoopCount', Inf, 'DelayTime', 0.1);
    else
        % If this is not the first frame, append to the file
        imwrite(ind, map, filename, 'gif', 'WriteMode', 'append', 'DelayTime', 0.1);
    end
end



