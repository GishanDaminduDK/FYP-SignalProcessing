%  Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com/ 
%  
%  
%   Redistribution and use in source and binary forms, with or without 
%   modification, are permitted provided that the following conditions 
%   are met:
%  
%     Redistributions of source code must retain the above copyright 
%     notice, this list of conditions and the following disclaimer.
%  
%     Redistributions in binary form must reproduce the above copyright
%     notice, this list of conditions and the following disclaimer in the 
%     documentation and/or other materials provided with the   
%     distribution.
%  
%     Neither the name of Texas Instruments Incorporated nor the names of
%     its contributors may be used to endorse or promote products derived
%     from this software without specific prior written permission.
%  
%   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
%   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
%   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
%   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
%   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
%   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
%   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
%   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
%   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
%   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
%   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
%  

% get number of valid frames in the associated *_data.bin file captured
% with TDA2 platform

% File header in *_idx.bin:
%     struct Info
%     {
%         uint32_t tag;
%         uint32_t version;
%         uint32_t flags;
%         uint32_t numIdx;       // number of frames 
%         uint64_t dataFileSize; // total data size written into file
%     };
% 
% Index for every frame from each radar:
%     struct BuffIdx
%     {
%         uint16_t tag;
%         uint16_t version; /*same as Info.version*/
%         uint32_t flags;
%         uint16_t width;
%         uint16_t height;
%         uint32_t pitchOrMetaSize[4]; /*For image data, this is pitch.
%                                                        For raw data, this is size in bytes per metadata plane.*/
%         uint32_t size; /*total size in bytes of the data in the buffer (sum of all planes)*/
%         uint64_t timestamp;
%         uint64_t offset;
%     };
%----------------------------------------------------------------------------------------------------------------

% function [numIdx, dataFileSize] = getValidNumFrames(adcIdxFileName)
% 
%     % Check if the file exists
%     if ~isfile(adcIdxFileName)
%         error("File not found: %s", adcIdxFileName);
%     end
% 
%     % Attempt to read uint32 header
%     idxFile = fopen(adcIdxFileName, 'r');
%     heaferInfoSize32 = 6; % Expected size for uint32 header
%     heaferInfo32 = fread(idxFile, heaferInfoSize32, 'uint32');
%     
%     % Validate the size of the uint32 header
%     if numel(heaferInfo32) < heaferInfoSize32
%         fclose(idxFile);
%         warning("Insufficient uint32 data in file. Generating default values.");
%         [numIdx, dataFileSize] = createValidIndexFile(adcIdxFileName);
%         return;
%     end
%     
%     % Extract number of effective frames
%     numIdx = heaferInfo32(4);
%     fclose(idxFile);
% 
%     % Attempt to read uint64 header
%     idxFile = fopen(adcIdxFileName, 'r');
%     heaferInfoSize64 = 3; % Expected size for uint64 header
%     heaferInfo64 = fread(idxFile, heaferInfoSize64, 'uint64');
%     
%     % Validate the size of the uint64 header
%     if numel(heaferInfo64) < heaferInfoSize64
%         fclose(idxFile);
%         warning("Insufficient uint64 data in file. Generating default values.");
%         [numIdx, dataFileSize] = createValidIndexFile(adcIdxFileName);
%         return;
%     end
% 
%     % Extract data file size
%     dataFileSize = heaferInfo64(3);
%     fclose(idxFile);
% end
% 
% % Helper function to create a valid index file
% function [numIdx, dataFileSize] = createValidIndexFile(fileName)
%     % Default values for creating a valid index file
%     defaultNumIdx = 100; % Example: 100 valid frames
%     defaultDataFileSize = 1e6; % Example: 1 MB size for data
% 
%     % Open file for writing
%     idxFile = fopen(fileName, 'w');
%     if idxFile == -1
%         error("Unable to create file: %s", fileName);
%     end
% 
%     % Write uint32 header
%     heaferInfo32 = uint32([0, 0, 0, defaultNumIdx, 0, 0]); % Example uint32 values
%     fwrite(idxFile, heaferInfo32, 'uint32');
% 
%     % Write uint64 header
%     heaferInfo64 = uint64([0, 0, defaultDataFileSize]); % Example uint64 values
%     fwrite(idxFile, heaferInfo64, 'uint64');
% 
%     fclose(idxFile);
% 
%     % Return default values
%     numIdx = defaultNumIdx;
%     dataFileSize = defaultDataFileSize;
% 
%     fprintf("Valid index file created: %s\n", fileName);
% end
function [numIdx, dataFileSize] = getValidNumFrames(adcIdxFileName)
% This function reads metadata from a binary index file and extracts two
% key pieces of information: the number of valid frames (numIdx) and the
% total data file size (dataFileSize).

% Open the index file in read mode
idxFile = fopen(adcIdxFileName, 'r');

% Define the size of the header information to read (in 32-bit words)
headerInfoSize = 6;

% Read the header information from the file
% 'uint32' specifies that the data is read as unsigned 32-bit integers
headerInfo = fread(idxFile, headerInfoSize, 'uint32');

% Display the entire header information for debugging
% disp('Header information (uint32):');
% disp(headerInfo);

% Extract the number of effective frames from the 4th element of the header
numIdx = headerInfo(4);

% Display the number of valid frames
% disp('Number of valid frames (numIdx):');
% disp(numIdx);

% Close the file after reading the first set of information
fclose(idxFile);

% Reopen the index file in read mode to read the next set of information
idxFile = fopen(adcIdxFileName, 'r');

% Define the size of the header information to read (in 64-bit words)
headerInfoSize = 3;

% Read the header information from the file
% 'uint64' specifies that the data is read as unsigned 64-bit integers
headerInfo = fread(idxFile, headerInfoSize, 'uint64');

% Display the entire header information for debugging
% disp('Header information (uint64):');
% disp(headerInfo);

% Extract the total data file size for the effective number of frames from
% the 3rd element of the header
dataFileSize = headerInfo(3);

% Display the total data file size
% disp('Total data file size (dataFileSize):');
% disp(dataFileSize);

% Close the file after reading the second set of information
fclose(idxFile);

end
