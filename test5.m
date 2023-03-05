% first simulation that re dship moves out and then corrects path by
% returning to path. The movement it takes is not yet ideal, it is one arc
% followed by straight line, then an arc in opposite direction. 

clc, clear all, close all

% Set up the initial positions of the red ship and obstacle
init_position = [0, 20,];
goal_position = [0,-10];
red_ship_position = init_position;

direct = 0;

heading0 = (pi/2)*3; %intial heading angle;
heading = heading0;

obstacle_position = [-10, 5];

% Set up the velocity vectors for the red ship and obstacle
red_ship_velocity = 3;
obstacle_velocity = [2 , 0];

% Set up the time step and the duration of the simulation
dt = 0.1;
t_end = 15;

% Set up the path array to store the positions of the red ship during movement
path = [red_ship_position(1), red_ship_position(2)];

% Loop over until red ship reaches to goal point
while norm(red_ship_position - goal_position)>0.1
    
    % Update the positions of the red ship and obstacle based on their velocities
    red_ship_position(1) = red_ship_position(1) + red_ship_velocity * cos(heading) * dt;
    red_ship_position(2) = red_ship_position(2) + red_ship_velocity * sin(heading) * dt;
    
    obstacle_position = obstacle_position + obstacle_velocity .* dt;

    % Check if the red ship has crossed the path of the obstacle
    if red_ship_position(2) < obstacle_position(2) + 5 && ...
       red_ship_position(2) > obstacle_position(2) && ...
       red_ship_position(1) < obstacle_position(1) + 5 && ...
       red_ship_position(1) > obstacle_position(1)

        % Determine the distance between the red ship and the obstacle
        distance = norm([red_ship_position(1), red_ship_position(2)] - obstacle_position);

        % If the red ship is too close to the obstacle, move in a semicircle around the obstacle
        if distance < 5

            % Calculate the angle between the red ship and the obstacle
            angle = atan2(red_ship_position(2) - obstacle_position(2), red_ship_position(1) - obstacle_position(1));
            
            % Determine the position of the center of the arc
            center = obstacle_position + [0, 5];
            
            % Calculate the radius of the arc
            radius = 2;

            % Calculate the new velocity vector for the red ship
            if angle >= 0 && angle < pi
                % Move to the left of the obstacle
                red_ship_velocity_v = [(red_ship_position(1)-center(1)), (red_ship_position(2)-center(2))] .* [-1, 1] / radius .* 2;
                
                % calculate heading angle
                heading = atan(red_ship_velocity_v(2)/red_ship_velocity_v(1)); 
                if red_ship_velocity_v(2)<0 &&...
                   red_ship_velocity_v(1)<0 
                    heading = pi + heading;
                end
                
                %calculate the magnitude of velocity
                red_ship_velocity = sqrt(red_ship_velocity_v(1)^2 + red_ship_velocity_v(2)^2);
            else
                % Move to the right of the obstacle
                red_ship_velocity_v = [(red_ship_position(1)-center(1)), (red_ship_position(2)-center(2))] .* [1, -1] / radius .* 2;
                
                % calculate heading angle
                heading = atan(red_ship_velocity_v(2)/red_ship_velocity_v(1)); 
                if red_ship_velocity_v(2)<0 &&...
                   red_ship_velocity_v(1)<0 
                    heading = pi + heading;
                end
               
                
                %calculate the magnitude of velocity
                red_ship_velocity = sqrt(red_ship_velocity_v(1)^2 + red_ship_velocity_v(2)^2);                
                
            end

        % If the red ship is not too close to the obstacle, move towards it
        else
            red_ship_velocity = 3;
            heading = heading0;
            
           
        end

    % If the red ship has not crossed the path of the obstacle, move towards it
    else
        red_ship_velocity = 3;
        heading = heading0;
        
        %calcaulate the angle between the lines(red ship - the obstacle, initial point- goal point)
        v_1 = [init_position(1),init_position(2)] - [goal_position(1), goal_position(2)];
        v_2 = [red_ship_position(1), red_ship_position(2)] - [obstacle_position(1),obstacle_position(2)];
        Theta = acos(dot(v_1, v_2)/(norm(v_1)*norm(v_2)));
        
        if Theta>pi/2 %if ship passes the obstacle
            if direct ==0
                center = [red_ship_position(1)+3, red_ship_position(2)+3];
                direct = 1;
            end
            
            % Move to the right of the obstacle
            red_ship_velocity_v = [(red_ship_position(1)-center(1)), (red_ship_position(2)-center(2))] .* [1, -1] / radius .* 2;

            % calculate heading angle
            heading = atan(red_ship_velocity_v(2)/red_ship_velocity_v(1)); 
            if red_ship_velocity_v(2)<0 &&...
               red_ship_velocity_v(1)<0 
                heading = pi + heading;
            end

            %calculate the magnitude of velocity
            red_ship_velocity = sqrt(red_ship_velocity_v(1)^2 + red_ship_velocity_v(2)^2);
        end
        
        %distance from red ship to 
        %http://mathworld.wolfram.com/Point-LineDistance2-Dimensional.html
        
        x1 = init_position(1);
        y1 = init_position(2);
        
        x2 = goal_position(1);
        y2 = goal_position(2);
        
        x3 = red_ship_position(1);
        y3 = red_ship_position(2);
        
        
        % Find the numerator for  point-to-line distance formula.
        numerator = abs((x2 - x1) * (y1 - y3) - (x1 - x3) * (y2 - y1));

        % Find the denominator for  point-to-line distance formula.
        denominator = sqrt((x2 - x1) ^ 2 + (y2 - y1) ^ 2);

        % Compute the distance.
        distance = numerator ./ denominator;
        
        
        if distance < 0.1
            red_ship_velocity = 3;
            heading = heading0;
            
        end
            
        

    end

    % Store the position of the red ship in the path array
    path = [path; [red_ship_position(1),red_ship_position(2)]];

    % Plot the positions of the red ship and obstacle
    plot(path(:,1), path(:,2), 'r--')
    hold on
    plot(red_ship_position(1), red_ship_position(2), 'ro', 'MarkerSize', 10)
    plot(obstacle_position(1), obstacle_position(2), 'bo', 'MarkerSize', 10)
    xlim([-15, 15])
    ylim([-15, 25])
    pause(0.1)
    hold off
end

% After the obstacle has moved, continue moving the red ship towards its final position
while red_ship_position(2) > 0
    % Determine the distance between the red ship and the final position
    distance = norm(red_ship_position - [0, 0]);

    % If the red ship is too far from the final position, move towards it
    if distance > 0.5
        red_ship_velocity = [-1, -3]
    else
        red_ship_velocity = [0, -3];
    end

    % Update the position of the red ship based on its velocity
    red_ship_position = red_ship_position + red_ship_velocity .* dt;
    path = [path; red_ship_position];

    % Plot the positions of the red ship and obstacle
    plot(path(:,1), path(:,2), 'r--')
    hold on
    plot(red_ship_position(1), red_ship_position(2), 'ro', 'MarkerSize', 10)
    plot(obstacle_position(1), obstacle_position(2), 'bo', 'MarkerSize', 10)
    xlim([-15, 15])
    ylim([-5, 25])
    pause(0.1)
    hold off
end

