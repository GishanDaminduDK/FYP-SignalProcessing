% Define file paths
file_paths = { ...
    'D:\drone_swarms_stationary\four_drones\environment3_20.csv', ...
    'D:\drone_swarms_stationary\four_drones\rcplane11_20.csv', ...
    'D:\drone_swarms_stationary\four_drones\rcplane11_10.csv', ...
    'D:\drone_swarms_stationary\four_drones\rcplane17_20.csv', ...
    'D:\drone_swarms_stationary\four_drones\rcplane18_20.csv'...
};

% Initialize five separate arrays
radar_data_1 = [];
radar_data_2 = [];
radar_data_3 = [];
radar_data_4 = [];
radar_data_5 = [];

% Loop through each file and store data in corresponding array
for i = 1:length(file_paths)
    % Read the CSV file
    data_combined = readmatrix(file_paths{i});
    
    % Determine sizes
    num_real_cols = size(data_combined, 2) / 2; % Split columns between real and imaginary parts
    rows = 256 * 128; % Define rows if applicable
    cols = 16 * 12;   % Define cols if applicable

    % Separate real and imaginary parts
    real_part = data_combined(:, 1:num_real_cols);
    imag_part = data_combined(:, num_real_cols+1:end);

    % Reconstruct the 2D complex array
    data_reshaped = complex(real_part, imag_part);

    % Reshape back to the original 4D array
    radar_data_reconstructed = reshape(data_reshaped, [256, 128, 16, 12]);

    % Store the reconstructed data in the appropriate array
    switch i
        case 1
            radar_data_1 = radar_data_reconstructed;
        case 2
            radar_data_2 = radar_data_reconstructed;
        case 3
            radar_data_3 = radar_data_reconstructed;
        case 4
            radar_data_4 = radar_data_reconstructed;
        case 5
            radar_data_5 = radar_data_reconstructed;
    end

    % Display the file name and data size
%     disp(['Processed file: ', file_paths{i}]);
%     disp(size(radar_data_reconstructed));
end
disp("Test");
% disp(size(radar_data_1));
% disp(radar_data_2(1,1,1,1));
% disp(radar_data_1(1,1,1,1));
% radar_data=radar_data_2-radar_data_1;
% disp(radar_data(1,1,1,1));


radar_data_new2=radar_data_5+radar_data_2+radar_data_3-radar_data_4-4*radar_data_1;
disp(radar_data_new2(1,1,1,1));
disp(size(radar_data_new2));
data = radar_data_new2; % Example 4D complex array

% Reshape the 4D array into a 2D array (256*128 rows, 16*12 columns)
data_reshaped = reshape(data, [256*128, 16*12]);

% Separate real and imaginary parts
real_part = real(data_reshaped);
imag_part = imag(data_reshaped);

% Combine real and imaginary parts into a single 2D array
data_combined = [real_part, imag_part];

% Write the 2D array to a CSV file
writematrix(data_combined, 'D:\drone_swarms_stationary\four_drones\drone_swarm4.csv');
disp('4D array saved as 2D to complex_data_2D.csv');






























































