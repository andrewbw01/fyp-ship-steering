%
clc, clear all, close all

%%%%%%%%%%% case 1 %%%%%%%%%%%%%%%%%%%
% Set up the initial positions of the red ship and green ship
init_position1 = [0, 20];
goal_position1 = [0, -10];

init_position2 = [0, -10];
goal_position2 = [0, 20];




% Set up the velocity vectors for the red ship and obstacle
V0 = 3;
red_ship_velocity = V0;
green_ship_velocity = V0;


%initial position of red ship and greenship
red_ship_position = init_position1;
green_ship_position = init_position2;


%calculate initial heading angles
heading10 = atan2(goal_position1(2) - init_position1(2), goal_position1(1) - init_position1(1));
heading1 = heading10;

heading20 = atan2(goal_position2(2) - init_position2(2), goal_position2(1) - init_position2(1));
heading2 = heading20;


% Set up the time step and the duration of the simulation
dt = 0.1;

% Set up the path array to store the positions of the red ship and green ship during movement
path1 = [red_ship_position(1), red_ship_position(2)];
path2 = [green_ship_position(1), green_ship_position(2)];

cir_angle1 = 0;    %angle of circle path for red ship
cir_flag1 = 0;     %Status on whether the reship is on a circular path
obstacle_pass1 = 0;%Status on whether the obstacle passes or not
reft_pass1 = 0 ;   %status on whether the red ship move to left

cir_angle2 = 0;    %angle of circle path for red ship
cir_flag2 = 0;     %Status on whether the reship is on a circular path
obstacle_pass2 = 0;%Status on whether the obstacle passes or not
reft_pass2 = 0 ;   %status on whether the red ship move to left

% Loop over until red ship reaches to goal point
while norm(red_ship_position - goal_position1)>0.4
    
    % Update the positions of the red ship and green ship based on their velocities
    red_ship_position(1) = red_ship_position(1) + red_ship_velocity * cos(heading1) * dt;
    red_ship_position(2) = red_ship_position(2) + red_ship_velocity * sin(heading1) * dt;  

    green_ship_position(1) = green_ship_position(1) + red_ship_velocity * cos(heading2) * dt;
    green_ship_position(2) = green_ship_position(2) + red_ship_velocity * sin(heading2) * dt;    


    % Determine the distance between the red ship and green ship
    distance = norm(red_ship_position- green_ship_position);   
   
    
    %form path to avoid the obstacle
    if distance < 8 && cir_flag1 ==0 && obstacle_pass1 ==0
         cir_flag1 =1;
         
         
         %determine the green ship on left or right of path 
         d = (green_ship_position(1) - init_position1(1))*(goal_position1(2)- init_position1(2)) -...
             (green_ship_position(2) - init_position1(2))*(goal_position1(1)- init_position1(1));
         
         if d > 0.01
             reft_pass1 = 0;
         else
             reft_pass1 = 1;
         end
      
         
    end
    
    %Update the heading angle of the red ship to avoid the obstacle
    if cir_flag1 ==1
        
        if reft_pass1 == 0
                        
             %calculate the disired the heading angle
             if cir_angle1 > (pi)
                 
                 d_heading1 = heading10;                   %disired heading angle
             else
                 
                 d_heading1 = heading10 + cir_angle -pi/2 ;%disired heading angle
             end
            
            %caculate the heading angle 
            if d_heading1 >heading1
                heading1 = heading1 +0.055;
            else
                heading1 = heading1 -0.053;
            end

        else
             %calculate the disired the heading angle
             if cir_angle1 > (pi)
                 
                 d_heading1 = heading10;                   %disired heading angle
             else
                 
                 d_heading1 = heading10 - cir_angle1 + pi/2 ;%disired heading angle
             end
            
            %caculate the heading angle 
            if d_heading1 >heading1
                heading1 = heading1 + 0.055;
            else
                heading1 = heading1 - 0.057;
            end

        end
        
        %update angle of circle path
        cir_angle1 = cir_angle1 + dt;
                
        if cir_angle1>pi+pi/2
            cir_flag1 = 0;
            obstacle_pass1 = 1;
        end
        
    else
        red_ship_velocity = V0;
        heading1 = heading10;   
        
    end
     
    
     %form path of green ship to avoid the obstacle
    if distance < 8 && cir_flag2 ==0 && obstacle_pass2 ==0
         cir_flag2 =1;
         
       
         %determine the green ship on left or right of path 
         d = (green_ship_position(1) - init_position2(1))*(goal_position2(2)- init_position2(2)) -...
             (green_ship_position(2) - init_position2(2))*(goal_position2(1)- init_position2(1));
         
         if d > 0.01
             reft_pass2 = 0;
         else
             reft_pass2 = 1;
         end
      
         
    end
    
    %Update the heading angle of the green ship to avoid the obstacle
    if cir_flag2 == 1
        
        if reft_pass2 == 0
                        
             %calculate the disired the heading angle
             if cir_angle2 > (pi)
                 
                 d_heading2 = heading20;                   %disired heading angle
             else
                 
                 d_heading2 = heading20 + cir_angle2 -pi/2 ;%disired heading angle
             end
            
            %caculate the heading angle 
            if d_heading2 >heading2
                heading2 = heading2 +0.055;
            else
                heading2 = heading2 -0.053;
            end

        else
             %calculate the disired the heading angle
             if cir_angle2 > (pi)
                 
                 d_heading2 = heading20;                   %disired heading angle
             else
                 
                 d_heading2 = heading20 - cir_angle2 + pi/2 ;%disired heading angle
             end
            
            %caculate the heading angle 
            if d_heading2 >heading2
                heading2 = heading2 + 0.055;
            else
                heading2 = heading2 - 0.057;
            end

        end
        
        %update angle of circle path
        cir_angle2 = cir_angle2 + dt;
                
        if cir_angle2>pi+pi/2
            cir_flag2 = 0;
            obstacle_pass2 = 1;
        end
        
    else
        green_ship_velocity = V0;
        heading2 = heading20;   
        
    end
    
    

    % Store the position of the red ship in the path array
    path1 = [path1; [red_ship_position(1),red_ship_position(2)]];
    
    % Store the position of the green ship in the path array
    path2 = [path2; [green_ship_position(1),green_ship_position(2)]];

    % Plot the positions of the red ship and green ship
    plot(path1(:,1), path1(:,2), 'r--')
    hold on
    
    plot(path2(:,1), path2(:,2), 'g--')
    plot(red_ship_position(1), red_ship_position(2), 'ro', 'MarkerSize', 10)
    plot(green_ship_position(1), green_ship_position(2), 'go', 'MarkerSize', 10)
    xlim([-15, 25])
    ylim([-15, 25])
    pause(0.1)
    hold off
end


