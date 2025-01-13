function [out1] = myspecgramnew(data1, window, nfft, shift)
    % Check if inputs are valid
    if nargin < 4
        error('Not enough input arguments.');
    end
    disp(size(data1));

    % Calculate the number of segments based on the length of data, window size, and shift
    N = floor((length(data1) - window) / shift) + 1;
    disp(length(data1))

    % Initialize the output matrix to zeros with the appropriate size
    out1 = zeros(nfft, N); % Preallocate with zeros for efficiency and to avoid undefined errors

    % Ensure that the number of segments is valid
    if N < 1
        error('Insufficient data length for the given window and shift sizes.');
    end

    % Loop to compute the FFT of each windowed segment
    for i = 1:N
        % Define the segment start and end indices
        startIdx = (i - 1) * shift + 1;
        endIdx = startIdx + window - 1;

        % Extract the data segment, apply a Hanning window, and compute the FFT
        segment = data1(startIdx:endIdx).' .* hann(window);
        tmp = fft(segment, nfft);

        % Assign the FFT result to the corresponding column in the output
        out1(:, i) = tmp;
    end
end
