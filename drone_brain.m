function [altitude_cmd, roll, pitch, yaw, kill_flag] = drone_brain(front_lazer, back_lazer, left_lazer, right_lazer, z_sensor, sim_time)
    %#codegen (Used for SIMULINK embedded C coder)
    
    persistent flight_mode;
    persistent hover_start;
    
    % --- INITIALIZATION ---
    if isempty(flight_mode)
        flight_mode = 1; 
        hover_start = 0;      
    end
    
    target_z = 0.3; % Target hover height in meters
    kill_flag = 0;
    
    altitude_cmd = 0; roll = 0; pitch = 0; yaw = 0;
    
    % --- FLIGHT LOGIC ---
    switch flight_mode
        case 1 % LIFTOFF
            [altitude_cmd, roll, pitch, yaw] = drone_liftoff_local(target_z, z_sensor, sim_time);
            if z_sensor >= (target_z - 0.05)
                flight_mode = 2;
                hover_start = sim_time;
            end
            
        case 2 % HOVER & AVOIDANCE
            [altitude_cmd, roll, pitch, yaw] = drone_hover_local(front_lazer, back_lazer, left_lazer, right_lazer, target_z);
            if (sim_time - hover_start) > 3.0 % <-- Hover time in seconds
                flight_mode = 3;
            end
            
        case 3 % ROAMING
            [altitude_cmd, roll, pitch, yaw] = drone_roam(front_lazer, back_lazer, left_lazer, right_lazer, target_z);
            if (sim_time - hover_start) > 60 % <-- Roam time in seconds
                flight_mode = 4;
            end
            
        case 4 % LANDING
            [altitude_cmd, roll, pitch, yaw, kill_flag] = drone_land_local(z_sensor, sim_time);
            
        otherwise
            altitude_cmd = 0; roll = 0; pitch = 0; yaw = 0; kill_flag = 1;

    end
end

% ==========================
% --- HELPER FUNCTIONS ---
% ==========================

% Liftoff
function [altitude_cmd, roll, pitch, yaw] = drone_liftoff_local(target_z, current_z_reading, current_time)
    persistent start_z;
    persistent start_time;
    
    if isempty(start_z)
        start_z = current_z_reading;
        start_time = current_time;
    end
    
    roll = 0.0; pitch = 0.0; yaw = 0.0;
    climb_rate = 0.2; 
    
    elapsed = current_time - start_time;
    calculated_z = start_z + (climb_rate * elapsed);
    
    altitude_cmd = min(calculated_z, target_z); 
end

% Hover
function [altitude_cmd, roll, pitch, yaw] = drone_hover_local(front_lazer, back_lazer, left_lazer, right_lazer, target_z)
    roll = 0.0; pitch = 0.0; yaw = 0.0;
    altitude_cmd = target_z;
    margin = 0.3; 
    avoid_speed_ms = 0.2; % Speed in m/s to push away from walls
    
    % AVOIDANCE Logic: Move AWAY from detected obstacles
    if front_lazer < margin, pitch = -avoid_speed_ms; end
    if back_lazer < margin, pitch = avoid_speed_ms; end
    if left_lazer < margin, roll = -avoid_speed_ms; end
    if right_lazer < margin, roll = avoid_speed_ms; end
end

% Roaming
function [altitude_cmd, roll, pitch, yaw] = drone_roam(front_lazer, back_lazer, left_lazer, right_lazer, target_z)
    persistent move_dir;    
    
    % 1. Sanitize out-of-range sensor data
    if front_lazer <= 0 || isnan(front_lazer), front_lazer = 9.99; end
    if back_lazer  <= 0 || isnan(back_lazer),  back_lazer  = 9.99; end
    if left_lazer  <= 0 || isnan(left_lazer),  left_lazer  = 9.99; end
    if right_lazer <= 0 || isnan(right_lazer), right_lazer = 9.99; end

    if isempty(move_dir)
        % Lock onto the most open path to start
        distances = [front_lazer, right_lazer, back_lazer, left_lazer];
        [~, max_idx] = max(distances);
        move_dir = max_idx; % 1=Front, 2=Right, 3=Back, 4=Left
    end
    
    roll = 0.0; pitch = 0.0; yaw = 0.0;
    altitude_cmd = target_z;
    
    roam_speed = 0.35;    % Primary travel speed in m/s
    wall_distance = 0.6; % Safety margin
    
    switch move_dir
        case 1 % Currently Moving Forward
            if front_lazer > wall_distance
                move_dir = 1;
            else
                % Path Blocked Logic
                if left_lazer > wall_distance
                    move_dir = 4; % Try Left
                elseif right_lazer > wall_distance
                    move_dir = 2; % Try Right
                else
                    move_dir = 3; % Reverse as last resort
                end
            end
            
        case 2 % Currently Moving Right
            if right_lazer > wall_distance
                move_dir = 2;
            else
                if front_lazer > wall_distance
                    move_dir = 1;
                elseif back_lazer > wall_distance
                    move_dir = 3;
                else
                    move_dir = 4;
                end
            end
            
        case 3 % Currently Moving Backward
            if back_lazer > wall_distance
                move_dir = 3;
            else
                if right_lazer > wall_distance
                    move_dir = 2;
                elseif left_lazer > wall_distance
                    move_dir = 4;
                else
                    move_dir = 1;
                end
            end
            
        case 4 % Currently Moving Left
            if left_lazer > wall_distance
                move_dir = 4;
            else
                if back_lazer > wall_distance
                    move_dir = 3;
                elseif front_lazer > wall_distance
                    move_dir = 1;
                else
                    move_dir = 2;
                end
            end
    end
    
    % 3. Apply the movement command directly in m/s
    if move_dir == 1      % Move Forward
        pitch = roam_speed;
    elseif move_dir == 2  % Move Right
        roll = -roam_speed;
    elseif move_dir == 3  % Move Backward
        pitch = -roam_speed;
    elseif move_dir == 4  % Move Left
        roll = roam_speed;
    end
end

% Landing
function [altitude_cmd, roll, pitch, yaw, kill_flag] = drone_land_local(z_actual, current_time)
    persistent land_start_alt;
    persistent land_start_time;
    
    if isempty(land_start_alt)
        land_start_alt = z_actual;
        land_start_time = current_time;
    end
    
    roll = 0.0; pitch = 0.0; yaw = 0.0; kill_flag = 0;
    descent_rate = 0.2;
    
    theoretical_duration = land_start_alt / descent_rate;
    elapsed_landing = current_time - land_start_time;
    
    if (elapsed_landing >= theoretical_duration) || (z_actual < 0.03)
        kill_flag = 1; 
        altitude_cmd = 0.0;
    else
        altitude_cmd = max(0, land_start_alt - (descent_rate * elapsed_landing));
    end
end