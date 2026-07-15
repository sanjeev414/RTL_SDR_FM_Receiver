%% FM Band Scanner
% This program scans for a particular frequency band as per the user input
% and checks for the presence of transmission for each frequency in the
% band as per the parameters defined.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


clc;
clear;
close all;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Parameters


disp("For searching of Stations............");
start_freq          = input("Enter the Start Frequency in MHz.........")*1e6;           % sweep start frequency
stop_freq           = input("Enter the Stop Frequency in MHz.........")*1e6;            % sweep stop frequency
rtlsdr_id           = '0';                                                              % RTL-SDR stick ID
rtlsdr_fs           = 2.8e6;                                                            % RTL-SDR sampling rate in Hz
rtlsdr_gain         = 40;                                                               % RTL-SDR tuner gain in dB
rtlsdr_frmlen       = 4096;                                                             % RTL-SDR output data frame size
rtlsdr_datatype     = 'single';                                                         % RTL-SDR output data type
rtlsdr_ppm          = -2;                                                               % RTL-SDR tuner parts per million correction
disp('  ');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


nfrmhold            = 20;                                                               % number of frames to receive
fft_hold            = 'max';                                                            % hold function "max" or "avg"
nfft                = 4096;                                                             % number of points in FFTs (2^something)
dec_factor          =16;                                                                % output plot downsample
overlap             = 0.5;                                                              % FFT overlap to counter rolloff
nfrmdump            = 100;                                                              % number of frames to dump after retuning (to clear buffer)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Calculation of Tuning Frequencies


rtlsdr_tunerfreq  = start_freq:rtlsdr_fs*overlap:stop_freq;                             % range of tuner frequency in Hz


if( max(rtlsdr_tunerfreq) < stop_freq )                                                 % check the whole range is covered, if not, add an extra tuner freq
    rtlsdr_tunerfreq(length(rtlsdr_tunerfreq)+1) = max(rtlsdr_tunerfreq)+rtlsdr_fs*overlap;
end

nretunes = length(rtlsdr_tunerfreq);                                                    % calculate number of retunes required
freq_bin_width = (rtlsdr_fs/nfft);                                                      % create xaxis
freq_axis = (rtlsdr_tunerfreq(1)-rtlsdr_fs/2*overlap  :  freq_bin_width*dec_factor  :  (rtlsdr_tunerfreq(end)+rtlsdr_fs/2*overlap)-freq_bin_width)/1e6;

% CALCULATIONS (others)

rtlsdr_data_fft = zeros(1,nfft);                                                        % fullsize matrix to hold calculated fft [1 x nfft]
fft_reorder = zeros(length(nfrmhold),nfft*overlap);                                     % matrix with overlap compensation to hold re-ordered ffts [navg x nfft*overlap]
fft_dec = zeros(nretunes,nfft*overlap/dec_factor);                                      % matrix with overlap compensation to hold all ffts  [ntune x nfft*overlap/data_decimate]

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% System Object Function Determination


obj_rtlsdr = comm.SDRRTLReceiver(...
    rtlsdr_id,...
    'CenterFrequency',      rtlsdr_tunerfreq(1),...
    'EnableTunerAGC', 		false,...
    'TunerGain', 			rtlsdr_gain,...
    'SampleRate',           rtlsdr_fs, ...
    'SamplesPerFrame', 		rtlsdr_frmlen,...
    'OutputDataType', 		rtlsdr_datatype ,...
    'FrequencyCorrection', 	rtlsdr_ppm );


% FIR decimator

obj_decmtr = dsp.FIRDecimator(...
    'DecimationFactor',     dec_factor,...
    'Numerator',            fir1(300,1/dec_factor));


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  Pre-Check and Simulation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% check if RTL-SDR is active

if ~isempty(sdrinfo(obj_rtlsdr.RadioAddress))
else
    error(['RTL-SDR failure. Please check connection to ',...
        'MATLAB using the "sdrinfo" command.']);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% for each of the tuner values

for ntune = 1:1:nretunes

    % tune RTL-SDR to new centre frequency

    obj_rtlsdr.CenterFrequency = rtlsdr_tunerfreq(ntune);
    disp("rtlsdr_fc:   " +rtlsdr_tunerfreq(ntune)+ "MHz.." );
    disp("  ");

    % dump frames to clear software buffer

    for frm = 1:1:nfrmdump
        % fetch a frame from the rtlsdr stick

        rtlsdr_data = step(obj_rtlsdr);
    end


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% Reception and Processing of Data from RTL-SDR


    % loop for nfrmhold frames

    for frm = 1:1:nfrmhold

        % fetch a frame from the rtlsdr stick

        rtlsdr_data = step(obj_rtlsdr);

        % remove DC component

        rtlsdr_data = rtlsdr_data - mean(rtlsdr_data);

        % find fft [ +ve , -ve ]

        rtlsdr_data_fft = abs(fft(rtlsdr_data,nfft))';

        % rearrange fft [ -ve , +ve ] and keep only overlap data

        fft_reorder(frm,( 1 : (overlap*nfft/2) ))      = rtlsdr_data_fft( (overlap*nfft/2)+(nfft/2)+1 : end );   % -ve
        fft_reorder(frm,( (overlap*nfft/2)+1 : end ))  = rtlsdr_data_fft( 1 : (overlap*nfft/2) );                % +ve

    end


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


    % process the fft data down to [1 x nfft*overlap/data_decimate] from [nfrmhold x nfft*overlap/data_decimate]

    fft_reorder_proc = max(fft_reorder);

    % decimate data to smooth and store in spectrum matrix

    fft_dec(ntune,:) = step(obj_decmtr,fft_reorder_proc')';

end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Data Manipulation and Representation


% REORDER INTO ONE MATRIX

fft_masterreshape = reshape(fft_dec',1,ntune*nfft*overlap/dec_factor);

% PLOT DATA

y_data = fft_masterreshape;
y_data_dbm = 10*log10((fft_masterreshape.^2)/50);
figure(1);title("Spectrum of Band of Signal");subplot(2,1,1);plot(freq_axis,y_data_dbm);
xlabel(" Frequency (MHz) ");ylabel(" Power Ratio (dBm) ");

subplot(2,1,2);plot(freq_axis,y_data);
xlabel(" Frequency (MHz) ");ylabel("Relative Power (Watts)");

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Determination of Channel Frequency and tuning.

% Find the Peak that has the maximum power and its corresponding index

[pwr,idx] = max(y_data_dbm);

% Check the corresponding Frequency for the maximum power by its index

top_freq = freq_axis(idx);
rtlsdr_fc = round(top_freq,2)*1e6;

disp("The Frequency with highest transmission power is:  "+ rtlsdr_fc + "MHz...");
disp(" "); 
%% 


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
