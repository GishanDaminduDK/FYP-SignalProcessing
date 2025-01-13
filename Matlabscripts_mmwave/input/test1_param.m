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


ADVANCED_FRAME_CONFIG = 0; 
dataPlatform = 'TDA2'; 
%pass the chirp parameters associated with test data 
numADCSample = 2.560000e+02; 
adcSampleRate = 8.000000e+06; %Hz/s 
startFreqConst = 7.700000e+10; %Hz 
chirpSlope = 7.898600e+13; %Hz/s 
chirpIdleTime = 5.000000e-06; %s 
adcStartTimeConst = 6.000000e-06; %s 
chirpRampEndTime = 4.000000e-05; %s 
framePeriodicty = 1.000000e-01; 
NumDevices = 4; 
framePeriodicty = 1.000000e-01; 
frameCount = 1.280000e+02; %s 
numChirpsInLoop = 1.200000e+01; %s 
nchirp_loops = 128; 
numTxAnt = 12; 
TxToEnable = [12  11  10   9   8   7   6   5   4   3   2   1];
numRxToEnable = 16; 
centerFreq = 7.826378e+01; 
