% ==========================================
% Crazyflie Command Hub & Live Mapper
% ==========================================
clear drone_brain; % Clear memory

disp('Building the Python buffer...');
fid = fopen('cf_telemetry.py', 'w');
fprintf(fid, 'class SensorBuffer:\n');
fprintf(fid, '    def __init__(self):\n');
fprintf(fid, '        self.front = 2.0\n');
fprintf(fid, '        self.back = 2.0\n');
fprintf(fid, '        self.left = 2.0\n');
fprintf(fid, '        self.right = 2.0\n');
fprintf(fid, '        self.z = 0.0\n');
fprintf(fid, '        self.x = 0.0\n');
fprintf(fid, '        self.y = 0.0\n');
fprintf(fid, '        self.vx = 0.0\n');
fprintf(fid, '        self.vy = 0.0\n\n');
fprintf(fid, '    def callback(self, timestamp, data, logconf):\n');
fprintf(fid, '        if "range.front" in data: self.front = float(data["range.front"]) / 1000.0\n');
fprintf(fid, '        if "range.back" in data: self.back = float(data["range.back"]) / 1000.0\n');
fprintf(fid, '        if "range.left" in data: self.left = float(data["range.left"]) / 1000.0\n');
fprintf(fid, '        if "range.right" in data: self.right = float(data["range.right"]) / 1000.0\n');
fprintf(fid, '        if "stateEstimate.z" in data: self.z = float(data["stateEstimate.z"])\n');
fprintf(fid, '        if "stateEstimate.x" in data: self.x = float(data["stateEstimate.x"])\n');
fprintf(fid, '        if "stateEstimate.y" in data: self.y = float(data["stateEstimate.y"])\n');
fprintf(fid, '        if "stateEstimate.vx" in data: self.vx = float(data["stateEstimate.vx"])\n');
fprintf(fid, '        if "stateEstimate.vy" in data: self.vy = float(data["stateEstimate.vy"])\n');
fclose(fid);

disp('Initializing CRTP drivers...');
py.cflib.crtp.init_drivers();
current_dir = py.str(pwd);
sys_path = py.sys.path;
if count(sys_path, current_dir) == 0
    insert(sys_path, int32(0), current_dir);
end
cf_mod = py.importlib.import_module('cf_telemetry');
py.importlib.reload(cf_mod); 
sensor_buffer = cf_mod.SensorBuffer();

% 2. --- INITIALIZE COMMAND HUB UI ---
disp('Spooling up UI Dashboard...');
fig = figure('Name', 'Command Hub', 'Color', 'w', 'Position', [100, 100, 950, 500]);

% Color Palette Definitions for UI
blue_header = [0.0, 0.15, 0.45]; % Deep Navy Blue
blue_text   = [0.0, 0.25, 0.60]; % Bright Blue
bg_color    = 'w';               % Clean White Background

% Setup Map Axis
ax = axes('Position', [0.05, 0.1, 0.55, 0.8], 'Color', 'k', ...
          'XColor', [0.2 0.2 0.2], 'YColor', [0.2 0.2 0.2]);
hold on; grid on; axis equal;
axis([-3 3 -3 3]); % 6x6 meter tracking area
title('Live 2D Room Mapping', 'Color', blue_header, 'FontSize', 14, 'FontWeight', 'bold');
xlabel('X Position (m)', 'Color', 'k', 'FontWeight', 'bold'); 
ylabel('Y Position (m)', 'Color', 'k', 'FontWeight', 'bold');

% Setup Map Plot Objects
wall_map = scatter(nan, nan, 10, [0.8 0.2 0.2], 'filled', 'MarkerFaceAlpha', 0.4); % Persistent Room Outline
path_line = plot(nan, nan, 'b-', 'LineWidth', 1.5); % The quadcopter's path history
wall_scatter = scatter(nan, nan, 30, 'r', 'filled'); % Bright current wall hits
drone_dot = plot(nan, nan, 'bo', 'MarkerFaceColor', 'c', 'MarkerSize', 10);

% Setup Dashboard Text Controls
uicontrol('Style', 'text', 'String', 'TELEMETRY DASHBOARD', 'Position', [620, 430, 300, 30], ...
    'FontSize', 14, 'FontWeight', 'bold', 'BackgroundColor', bg_color, 'ForegroundColor', blue_header);

txt_mode = uicontrol('Style', 'text', 'String', 'MODE: WAITING', 'Position', [620, 390, 300, 30], ...
    'FontSize', 13, 'FontWeight', 'bold', 'BackgroundColor', bg_color, 'ForegroundColor', [0.85 0.2 0.0], 'HorizontalAlignment', 'left');

txt_pos = uicontrol('Style', 'text', 'String', 'Pos: X: 0.00 | Y: 0.00 | Z: 0.00', 'Position', [620, 350, 300, 30], ...
    'FontSize', 11, 'FontWeight', 'bold', 'BackgroundColor', bg_color, 'ForegroundColor', blue_text, 'HorizontalAlignment', 'left');

txt_vel = uicontrol('Style', 'text', 'String', 'Vel: VX: 0.00 | VY: 0.00', 'Position', [620, 310, 300, 30], ...
    'FontSize', 11, 'FontWeight', 'bold', 'BackgroundColor', bg_color, 'ForegroundColor', blue_text, 'HorizontalAlignment', 'left');

txt_lasers = uicontrol('Style', 'text', 'String', 'F: 0.00 | B: 0.00 | L: 0.00 | R: 0.00', 'Position', [620, 270, 300, 30], ...
    'FontSize', 11, 'FontWeight', 'bold', 'BackgroundColor', bg_color, 'ForegroundColor', blue_text, 'HorizontalAlignment', 'left');

txt_cmds = uicontrol('Style', 'text', 'String', 'Cmds (m/s): Pitch: 0.00 | Roll: 0.00', 'Position', [620, 230, 300, 30], ...
    'FontSize', 11, 'FontWeight', 'bold', 'BackgroundColor', bg_color, 'ForegroundColor', blue_text, 'HorizontalAlignment', 'left');

txt_time = uicontrol('Style', 'text', 'String', 'Flight Time: 0.0s', 'Position', [620, 190, 300, 30], ...
    'FontSize', 11, 'FontWeight', 'bold', 'BackgroundColor', bg_color, 'ForegroundColor', blue_text, 'HorizontalAlignment', 'left');

% Map Memory Arrays
path_x = []; path_y = [];
map_x = []; map_y = []; % Persistent Array for Room Outline
state_names = {'1: LIFTOFF', '2: HOVER', '3: ROAMING', '4: LANDING'};

% 3. Connect to Quadcopter
uri = 'radio://0/80/2M';
disp('Opening radio link...');
sc = py.cflib.crazyflie.syncCrazyflie.SyncCrazyflie(uri);
sc.open_link();
cf = sc.cf; 
kill_switch = onCleanup(@() emergency_land(cf, sc));

try
    % SPLIT LOGGING INTO TWO BLOCKS (Max payload size is ~26 bytes per block)
    logconf1 = py.cflib.crazyflie.log.LogConfig(name='Ranges', period_in_ms=int32(20));
    logconf1.add_variable('range.front', 'uint16_t');
    logconf1.add_variable('range.back', 'uint16_t');
    logconf1.add_variable('range.left', 'uint16_t');
    logconf1.add_variable('range.right', 'uint16_t');
    logconf1.add_variable('stateEstimate.z', 'float');
    
    logconf2 = py.cflib.crazyflie.log.LogConfig(name='States', period_in_ms=int32(20));
    logconf2.add_variable('stateEstimate.x', 'float'); 
    logconf2.add_variable('stateEstimate.y', 'float'); 
    logconf2.add_variable('stateEstimate.vx', 'float'); 
    logconf2.add_variable('stateEstimate.vy', 'float'); 
    
    cf.log.add_config(logconf1);
    cf.log.add_config(logconf2);
    
    cb_handle = py.getattr(sensor_buffer, 'callback');
    logconf1.data_received_cb.add_callback(cb_handle);
    logconf2.data_received_cb.add_callback(cb_handle);
    
    logconf1.start();
    logconf2.start();
    
    disp('*** FLIGHT LOOP ACTIVE - PRESS CTRL+C TO KILL ***');
    pause(1); 
    cf.commander.send_setpoint(0.0, 0.0, 0.0, int32(0));
    
    kill_flag = 0;
    tic; 
    
    while kill_flag == 0
        sim_time = toc;
        
        % Read buffer
        f = double(sensor_buffer.front);
        b = double(sensor_buffer.back);
        l = double(sensor_buffer.left);
        r = double(sensor_buffer.right);
        z = double(sensor_buffer.z);
        x = double(sensor_buffer.x);
        y = double(sensor_buffer.y);
        vx = double(sensor_buffer.vx);
        vy = double(sensor_buffer.vy);
        
        % Run the Brain
        [alt_cmd, roll_cmd, pitch_cmd, yaw_cmd, kill_flag] = ...
            drone_brain(f, b, l, r, z, sim_time);
            
        % Issue command
        if kill_flag == 1
            cf.commander.send_stop_setpoint();
            break;
        else
            cf.commander.send_hover_setpoint(pitch_cmd, roll_cmd, yaw_cmd, alt_cmd);
        end
        
        % --- ESTIMATE STATE FOR UI ---
        if sim_time < 5.0
            current_state = 1; % Liftoff
        elseif sim_time < 8.0 
            current_state = 2; % Hover
        elseif sim_time < 38.0
            current_state = 3; % Roaming
        else
            current_state = 4; % Landing
        end
        
        % --- UPDATE MAP MEMORY ---
        path_x(end+1) = x; 
        path_y(end+1) = y;
        
        % Calculate CURRENT Wall Points AND Append to Permanent Map
        curr_map_x = []; curr_map_y = [];
        if f < 2.0
            curr_map_x(end+1) = x + f; curr_map_y(end+1) = y; 
            map_x(end+1) = x + f; map_y(end+1) = y;
        end
        if b < 2.0
            curr_map_x(end+1) = x - b; curr_map_y(end+1) = y; 
            map_x(end+1) = x - b; map_y(end+1) = y;
        end
        if l < 2.0
            curr_map_x(end+1) = x; curr_map_y(end+1) = y + l; 
            map_x(end+1) = x; map_y(end+1) = y + l;
        end
        if r < 2.0
            curr_map_x(end+1) = x; curr_map_y(end+1) = y - r; 
            map_x(end+1) = x; map_y(end+1) = y - r;
        end
        
        % --- UPDATE UI ---
        set(path_line, 'XData', path_x, 'YData', path_y);
        set(wall_map, 'XData', map_x, 'YData', map_y); % Plot the permanent room outline
        set(drone_dot, 'XData', x, 'YData', y);
        set(wall_scatter, 'XData', curr_map_x, 'YData', curr_map_y);
        
        if current_state >= 1 && current_state <= 4
            set(txt_mode, 'String', ['MODE: ', state_names{current_state}]);
        end
        set(txt_pos, 'String', sprintf('Pos: X: %0.2f | Y: %0.2f | Z: %0.2f', x, y, z));
        set(txt_vel, 'String', sprintf('Vel (m/s): VX: %0.2f | VY: %0.2f', vx, vy));
        set(txt_lasers, 'String', sprintf('F: %0.2f | B: %0.2f | L: %0.2f | R: %0.2f', f, b, l, r));
        set(txt_cmds, 'String', sprintf('Cmds (m/s): Pitch: %0.2f | Roll: %0.2f', pitch_cmd, roll_cmd));
        set(txt_time, 'String', sprintf('Flight Time: %0.1fs', sim_time));
        
        drawnow limitrate; 
        pause(0.02); 
    end
    
catch ME
    disp('System Fault / Error detected:');
    disp(ME.message);
end

function emergency_land(cf, sc)
    disp('Executing hardware shutdown sequence...');
    try cf.commander.send_stop_setpoint(); catch, end
    try sc.close_link(); catch, end
    disp('Quadcopter safely disconnected.');
end