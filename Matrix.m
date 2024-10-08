% Define the chirp_ADC_matrix
chirp_ADC_matrix = [
    [ [1, 2], [5, 6], [9, 10] ],   % ADC samples for chirp 1 across 3 loops
    [ [3, 4], [7, 8], [11, 12] ],  % ADC samples for chirp 2 across 3 loops
    [ [13, 14], [17, 18], [21, 22] ],  % ADC samples for chirp 3 across 3 loops
    [ [15, 16], [19, 20], [23, 24] ]   % ADC samples for chirp 4 across 3 loops
];


chirp_ADC_matrix = squeeze(chirp_ADC_matrix);
disp(size(chirp_ADC_matrix))
numberofADCsamples = 2;
% Number of loops
numLoops = size(chirp_ADC_matrix,numberofADCsamples);

% Initialize a cell array to store the data for each loop
loopDataArray = cell(1, numLoops);

% Loop through each loop and extract the ADC values
for loopIdx = 1:numLoops
    % Extract all chirps data for the current loop
    extracted_data = squeeze(chirp_ADC_matrix(:, loopIdx, :))';
    
    % Store the extracted data in the cell array
    loopDataArray{loopIdx} = extracted_data;
    
    % Display the extracted data for the current loop
    disp(['Extracted Data for Loop ', num2str(loopIdx), ':']);
    disp(extracted_data);
end

% Now, loopDataArray contains separate arrays for each loop's ADC values
disp('All Loops Data:');
disp(loopDataArray{1});


