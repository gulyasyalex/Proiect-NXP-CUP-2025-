recordTerminator = sprintf('\r\n');
fieldTerminator = ';';
% good bad
% 17 - 3
% 22 - 0
enable_car_engine = 0.0;
enable_car_steering_wheel = 0.0;
enable_emergency_brake = 0.0;
enable_pixy_vector_approximation = 0.0;             
enable_distance_sensor1 = 1.0;
enable_distance_sensor2 = 1.0;
enable_distance_sensor3 = 1.0;
enable_remote_start_stop = 0.0

lane_width_vector_unit_real = 53.0;
black_color_treshold = 0.2;
car_length_cm = 17.5;
lookahead_min_distance_cm = 22.0;                       % 22
lookahead_max_distance_cm = 50.0;                       % 40
min_speed = 97.0;
<<<<<<< HEAD
max_speed = 128.0;                                      % 
car_speed_ki = -0.01;
=======
max_speed = 135.0;                                      % 
car_speed_ki = -0.02;
>>>>>>> 6876ebeacb7986081a9488929a22d38a9beb4415
car_speed_kd = -0.2;
car_speed_ki_min_max_impact = 5.0;
emergency_break_distance_cm = 75.0;                     % 75
emergency_brake_min_speed = 94.0;
emergency_brake_distance_from_obstacle_cm = 9.0;       % 14
emergency_brake_enable_delay = 10.0;
steering_wheel_angle_offset = 0.0;
min_axis_angle_vector = 15.0;
max_speed_after_emergency_brake_delay = 110;
finish_line_angle_tolerance = 15.0;

values = [lane_width_vector_unit_real lookahead_min_distance_cm...
    lookahead_max_distance_cm emergency_break_distance_cm...
    min_speed max_speed black_color_treshold car_length_cm...
    enable_car_engine...
    enable_car_steering_wheel...
    emergency_brake_min_speed...
    emergency_brake_distance_from_obstacle_cm...
    enable_emergency_brake...
    enable_pixy_vector_approximation...
    enable_distance_sensor1...
    enable_distance_sensor2...
    emergency_brake_enable_delay...
    steering_wheel_angle_offset...
    enable_distance_sensor3...
    min_axis_angle_vector...
    max_speed_after_emergency_brake_delay...
    enable_remote_start_stop...
    car_speed_ki...
    car_speed_kd...
    car_speed_ki_min_max_impact...
    finish_line_angle_tolerance...
    enable_finish_line];

outputString = '';

for i = 1:length(values)
    outputString = [outputString sprintf('%.2f%c', values(i), fieldTerminator)];
end

if ~isempty(outputString)
    outputString = outputString(1:end-1);
end

outputString = [outputString recordTerminator];

server.write(outputString)


