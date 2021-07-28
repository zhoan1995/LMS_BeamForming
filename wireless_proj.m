clear ; clc; close all
%% defining a geometry
pars.fc=1e9; %carrier freq
pars.c=physconst('LightSpeed');
pars.lambda=pars.c/pars.fc; %carrier wavelengh
pars.SNR=20; 
pars.n_signals = 4;


%define the geometry of the problem (xyz coordinates)
Geometry.BSPos=[0,0,25];
Geometry.V1PosStart=[70,-100,1.5]; %start position for vehicle 1
Geometry.V1PosEnd=[70,100,1.5]; %end position for vehicle 1(movement in y)
Geometry.V2PosStart=[200,-50,1.5]; %start position for vehicle 2
Geometry.V2PosEnd=[10,-50,1.5]; %end position for vehicle 2(movement in x)
Geometry.I1Pos=[10,-210,1.5]; %position of first interf
Geometry.I2Pos=[-150,100,1.5]; %position of 2nd interf

%Calculating the DoA for I#1 at start

Geometry.DistI1Start=sqrt(sum((Geometry.I1Pos(1,1:2)- Geometry.BSPos(1,1:2)).^2));

Geometry.AOAI1Start=atan2(Geometry.BSPos(1,2)-Geometry.I1Pos(1,2), Geometry.BSPos(1,1)-Geometry.I1Pos(1,1))*180/pi;
Geometry.ZOAI1Start=atan2(Geometry.DistI1Start, Geometry.BSPos(1,3)-Geometry.I1Pos(1,3))*180/pi;
Geometry.DOAVIStart=[Geometry.AOAI1Start Geometry.ZOAI1Start];

%Calculating the DoA for V#2 at start
Geometry.DistI2Start=sqrt(sum((Geometry.I2Pos(1,1:2)-...
    Geometry.BSPos(1,1:2)).^2));

Geometry.AOAI2Start=atan2(Geometry.BSPos(1,2)-Geometry.I2Pos(1,2),...
    Geometry.BSPos(1,1)-Geometry.I2Pos(1,1))*180/pi;
Geometry.ZOAI2Start=atan2(Geometry.DistI2Start,...
    Geometry.BSPos(1,3)-Geometry.I2Pos(1,3))*180/pi;
Geometry.DOAI2Start=[Geometry.AOAI2Start Geometry.ZOAI2Start];





%%extract distances
%calculate distances as sqrt((x1-x2)^2+(y1-y2)^2)
Geometry.T1=sqrt(sum((Geometry.V1PosEnd(1,1:2)-...
    Geometry.V1PosStart(1,1:2)).^2));
Geometry.T2=sqrt(sum((Geometry.V2PosEnd(1,1:2)-...
    Geometry.V2PosStart(1,1:2)).^2));

Geometry.DistV1Start=sqrt(sum((Geometry.V1PosStart(1,1:2)-...
    Geometry.BSPos(1,1:2)).^2));
Geometry.DistV2Start=sqrt(sum((Geometry.V2PosStart(1,1:2)-...
    Geometry.BSPos(1,1:2)).^2));

%calculate the DoA for V#1 at start
%DoA=[AoA ZoA]
Geometry.AOAV1Start=atan2(Geometry.BSPos(1,2)-Geometry.V1PosStart(1,2),...
    Geometry.BSPos(1,1)-Geometry.V1PosStart(1,1))*180/pi;
Geometry.ZOAV1Start=atan2(Geometry.DistV1Start,...
    Geometry.BSPos(1,3)-Geometry.V1PosStart(1,3))*180/pi;
Geometry.DOAV1Start=[Geometry.AOAV1Start Geometry.ZOAV1Start];

%Calculating the DoA for V#2 at start
Geometry.AOAV2Start=atan2(Geometry.BSPos(1,2)-Geometry.V2PosStart(1,2),...
    Geometry.BSPos(1,1)-Geometry.V2PosStart(1,1))*180/pi;
Geometry.ZOAV2Start=atan2(Geometry.DistV2Start,...
    Geometry.BSPos(1,3)-Geometry.V2PosStart(1,3))*180/pi;
Geometry.DOAV2Start=[Geometry.AOAV2Start Geometry.ZOAV2Start];


%defining a 4*4 antenna array can be done with phased.URA
Geometry.Nant=4; %number of antenna
Geometry.BSarray=phased.URA('Size',[4 4],...
    'ElementSpacing',[pars.lambda/2 pars.lambda/2],'ArrayNormal','x');
%to get position of elements of the array simply cal
Geometry.BSAntennaPos=getElementPosition(Geometry.BSarray);

%creating conformal antenna array
Geometry.confarray=phased.ConformalArray('ElementPosition',Geometry.BSAntennaPos);


%% channel model
%Simulation Parameters
sp=qd_simulation_parameters;
sp.center_frequency=pars.fc;
%Creating a layout object with the simulation parameters
l=qd_layout(sp);                      % Create new layout
%Setting scenario parameters:
l.set_scenario('QuaDRiGa_UD2D_LOS');
txArr=qd_arrayant('omni');
rxArr=qd_arrayant('omni');
rxArr.no_elements=16;
rxArr.element_position=Geometry.BSAntennaPos;

%Tx settings
    l.no_tx = 4;                             % We want 4 Tx
    l.tx_position =[Geometry.V1PosStart', Geometry.V2PosStart', ...
    Geometry.I1Pos', Geometry.I2Pos'];        % Tx position based on the Vi and Ii.
    l.tx_array = txArr;                          
% Rx settings
    l.no_rx = 1;                              % 1 Receiver
    l.rx_array=rxArr;
    l.rx_position=Geometry.BSPos';            %put the BSPos in RX pos
    
% %%track1 configuration:
%  tx_track1=qd_track('linear', Geometry.T1, pi/2);
%  tx_track1.name='trackV1';
%  tx_track1.initial_position=Geometry.V1PosStart';
%  %track2 configuration:
%  tx_track2=qd_track('linear', Geometry.T2, pi);
%  tx_track2.name='trackV2';
%  tx_track2.initial_position=Geometry.V2PosStart';
%  %Attaching the tracks to the layout
%  l.tx_track(1,1)=copy(tx_track1);
%  l.tx_track(1,2)=copy(tx_track2);
%  l.rx_position=Geometry.BSPos';

%l.visualize(); %visualizing the scenario
l.set_pairing; %Evaluate all links
chan=l.get_channels; %Generate input for the channel_builder


%% OFDM1 configuration:
ofdmMod1 = comm.OFDMModulator('FFTLength', 12, ...
    'NumGuardBandCarriers', [1;1], ...
    'InsertDCNull', false, ...
    'CyclicPrefixLength', 0, ...
    'Windowing', false, ...
    'NumSymbols', 140, ...
    'NumTransmitAntennas', 1, ...
    'PilotInputPort', true, ...
    'PilotCarrierIndices', 10);
M1 = 4; 	 % Modulation order
% input bit source:
in1 = randi([0 1], 9*140*2, 1);
dataInput1 = qammod(in1, M1, 'gray', 'InputType', 'bit', 'UnitAveragePower', true);
ofdmInfo1 = info(ofdmMod1);
ofdmSize1 = ofdmInfo1.DataInputSize;
dataInput1 = reshape(dataInput1, ofdmSize1);
% waveform generation:
pilotInput1 = ones(1, 140, 1);
waveform1 = ofdmMod1(dataInput1, pilotInput1);
Fs1 = 180000;                                 % sample rate of waveform
%build the demodulator from the modulator
ofdmDemod1 = comm.OFDMDemodulator(ofdmMod1);
Geometry.Ts=1/Fs1;
%% OFDM2 configuration:
ofdmMod2 = comm.OFDMModulator('FFTLength', 12, ...
    'NumGuardBandCarriers', [1;1], ...
    'InsertDCNull', false, ...
    'CyclicPrefixLength', 0, ...
    'Windowing', false, ...
    'NumSymbols', 140, ...
    'NumTransmitAntennas', 1, ...
    'PilotInputPort', true, ...
    'PilotCarrierIndices', 3);
M2 = 4; 	 % Modulation order
% input bit source:
in2 = randi([0 1], 9*140*2, 1);
dataInput2 = qammod(in2, M2, 'gray', 'InputType', 'bit', 'UnitAveragePower', true);
ofdmInfo2 = info(ofdmMod2);
ofdmSize2 = ofdmInfo2.DataInputSize;
dataInput2 = reshape(dataInput2, ofdmSize2);
% waveform generation:
pilotInput2 = ones(1, 140, 1);
waveform2 = ofdmMod2(dataInput2, pilotInput2);
Fs2 = 180000;                                     % sample rate of waveform
%build the demodulator from the modulator
ofdmDemod2 = comm.OFDMDemodulator(ofdmMod2);
%% Generate OFDM Waveform
% OFDM configuration:
ofdmMod = comm.OFDMModulator('FFTLength', 12, ...
    'NumGuardBandCarriers', [1;1], ...
    'InsertDCNull', false, ...
    'CyclicPrefixLength', [0], ...
    'Windowing', false, ...
    'NumSymbols', 140, ...
    'NumTransmitAntennas', 1, ...
    'PilotInputPort', true, ...
    'PilotCarrierIndices', [6]);

M = 4; 	 % Modulation order
% input bit source:
in = randi([0 1], 9*140*2, 1);

dataInput = qammod(in, M, 'gray', 'InputType', 'bit', 'UnitAveragePower', true);
ofdmInfo = info(ofdmMod);
ofdmSize3 = ofdmInfo.DataInputSize;
dataInput = reshape(dataInput, ofdmSize3);

% waveform generation:
pilotInput = ones(1, 140, 1);
waveform_I1 = ofdmMod(dataInput, pilotInput);
ofdmMod = comm.OFDMModulator('FFTLength', 12, ...
    'NumGuardBandCarriers', [1;1], ...
    'InsertDCNull', false, ...
    'CyclicPrefixLength', [0], ...
    'Windowing', false, ...
    'NumSymbols', 140, ...
    'NumTransmitAntennas', 1, ...
    'PilotInputPort', true, ...
    'PilotCarrierIndices', [11]);

M = 4; 	 % Modulation order
% input bit source:
in = randi([0 1], 9*140*2, 1);

dataInput = qammod(in, M, 'gray', 'InputType', 'bit', 'UnitAveragePower', true);
ofdmInfo = info(ofdmMod);
ofdmSize4 = ofdmInfo.DataInputSize;
dataInput = reshape(dataInput, ofdmSize4);

% waveform generation:
pilotInput = ones(1, 140, 1);
waveform_I2 = ofdmMod(dataInput, pilotInput);
% PilotCarrierIndices 6 & 12


%% channels output
%vehicle 1
channel=chan(1);
chTaps=size(channel.delay); 
TsVect=0:Geometry.Ts:Geometry.Ts*(length(waveform1)-1);
chOut1=zeros(chTaps(1), length(waveform1));
   for antenna=1:1:chTaps(1)
        for path=1:1:chTaps(3)
            %Get the x for our interpolation
            %We put minus in the delay because we want to do the convolution
            %inX=TsVect-channel.delay(antenna, 1, path, snap);
            inX=TsVect-channel.delay(antenna, 1, path, 1);
            %Interpolation the waveform
            inY=interp1(TsVect, waveform1.', inX, 'pchip');
            %Get the output by multiplting by the coefficient and adding all
            %contributions up!
            chOut1(antenna,:)=inY*channel.coeff(antenna, 1,path ,1)+chOut1(antenna,:);
        
        end
    end


%vehicle 2
channel = chan(2);
chTaps=size(channel.delay); 
TsVect=0:Geometry.Ts:Geometry.Ts*(length(waveform2)-1);
chOut2=zeros(chTaps(1), length(waveform2));
   for antenna=1:1:chTaps(1)
        for path=1:1:chTaps(3)
            %Get the x for our interpolation
            %We put minus in the delay because we want to do the convolution
            %inX=TsVect-channel.delay(antenna, 1, path, snap);
            inX=TsVect-channel.delay(antenna, 1, path, 1);
            %Interpolation the waveform
            inY=interp1(TsVect, waveform2.', inX, 'pchip');
            %Get the output by multiplting by the coefficient and adding all
            %contributions up!
            chOut2(antenna,:)=inY*channel.coeff(antenna, 1,path ,1)+chOut2(antenna,:);
        
        end
    end
%% channels output Interfr
% I1
channel=chan(3);
chTaps=size(channel.delay); 
TsVect=0:Geometry.Ts:Geometry.Ts*(length(waveform_I1)-1);
chOutI1=zeros(chTaps(1), length(waveform_I1));
   for antenna=1:1:chTaps(1)
        for path=1:1:chTaps(3)
            %Get the x for our interpolation
            %We put minus in the delay because we want to do the convolution
            %inX=TsVect-channel.delay(antenna, 1, path, snap);
            inX=TsVect-channel.delay(antenna, 1, path, 1);
            %Interpolation the waveform
            inY=interp1(TsVect, waveform_I1.', inX, 'pchip');
            %Get the output by multiplting by the coefficient and adding all
            %contributions up!
            chOutI1(antenna,:)=inY*channel.coeff(antenna, 1,path ,1)+chOutI1(antenna,:);
        
        end
   end
    
   % I2
channel=chan(4);
chTaps=size(channel.delay); 
TsVect=0:Geometry.Ts:Geometry.Ts*(length(waveform_I2)-1);
chOutI2=zeros(chTaps(1), length(waveform_I2));
   for antenna=1:1:chTaps(1)
        for path=1:1:chTaps(3)
            %Get the x for our interpolation
            %We put minus in the delay because we want to do the convolution
            %inX=TsVect-channel.delay(antenna, 1, path, snap);
            inX=TsVect-channel.delay(antenna, 1, path, 1);
            %Interpolation the waveform
            inY=interp1(TsVect, waveform_I2.', inX, 'pchip');
            %Get the output by multiplting by the coefficient and adding all
            %contributions up!
            chOutI2(antenna,:)=inY*channel.coeff(antenna, 1,path ,1)+chOutI2(antenna,:);
        
        end
    end



%% Get the total channel output
chOut = chOut1 + chOut2 + chOutI1 + chOutI2;
chOut = awgn(chOut,pars.SNR,'measured')* 1e3;


%% AoA
%doas=[chan1 AoA,EoA; chan2 AoA,EoA]
doas = [  [chan(1).par.AoA_cb(1),chan(1).par.EoA_cb(1)];...
          [chan(2).par.AoA_cb(1),chan(2).par.EoA_cb(1)];...
          [chan(3).par.AoA_cb(1),chan(3).par.EoA_cb(1)];...
          [chan(4).par.AoA_cb(1),chan(4).par.EoA_cb(1)]]';
%% LMS BF
n_iter=100; % number of iteration for LMS alg

[out_arr_v1 ,w_v1]=LMSBF(Geometry, pars, doas, chOut, waveform1,n_iter);
[out_arr_v2, w_v2]=LMSBF(Geometry, pars, doas, chOut, waveform2,n_iter);

%% ofdm demodulations

%before BF
out_v1=ofdmDemod1(transpose(chOut(1,:)));
out_v2=ofdmDemod2(transpose(chOut(1,:)));
%after BF
BFV1_out = ofdmDemod1(out_arr_v1);
BFV2_out = ofdmDemod2(out_arr_v2);


%% Scattergraph
% without beamforming
figure;
xv1 = real(out_v1);
xv1 = reshape(xv1,[9*140,1]);
yv1 = imag(out_v1);
yv1 = reshape(yv1,[9*140,1]);
scatter(xv1,yv1);
title('Vehicle1: W/o Beamforming');

figure;
xv2 = real(out_v2);
xv2 = reshape(xv2,[9*140,1]);
yv2 = imag(out_v2);
yv2 = reshape(yv2,[9*140,1]);
scatter(xv2,yv2);
title('Vehicle2: W/o Beamforming');

% with beamforming
figure;
xbeamv1 = real(BFV1_out);
xbeamv1 = reshape(xbeamv1,[9*140,1]);
ybeamv1 = imag(BFV1_out);
ybeamv1 = reshape(ybeamv1,[9*140,1]);
scatter(xbeamv1,ybeamv1);
title('Vehicle1:with LMS Beamforming');

figure;
xbeamv2 = real(BFV2_out);
xbeamv2 = reshape(xbeamv2,[9*140,1]);
ybeamv2 = imag(BFV2_out);
ybeamv2 = reshape(ybeamv2,[9*140,1]);
scatter(xbeamv2,ybeamv2);
title('Vehicle2:with LMS Beamforming');

%% BER Calculation

% BER for Vehicle 1 w/o BF
dataOutv1 = qamdemod(out_v1, 4,'OutputType', 'bit', 'UnitAveragePower', true);
dataOutv1 = dataOutv1(:);
[Errorsv1, BERv1] = biterr(in1,dataOutv1);
% BER for Vehicle 1 with BF
dataOutv1_beam = qamdemod(BFV1_out, 4,'OutputType', 'bit', 'UnitAveragePower', true);
dataOutv1_beam = dataOutv1_beam(:);
[Errorsv1_beam, BERv1_beam] = biterr(in1,dataOutv1_beam);

% BER for Vehicle 2 w/o BF
dataOutv2 = qamdemod(out_v2, 4,'OutputType', 'bit', 'UnitAveragePower', true);
dataOutv2 = dataOutv2(:);
[Errorsv2, BERv2] = biterr(in2,dataOutv2);
% BER BER for Vehicle 2 with BF
dataOutv2_beam = qamdemod(BFV2_out, 4,'OutputType', 'bit', 'UnitAveragePower', true);
dataOutv2_beam = dataOutv2_beam(:);
[Errorsv2_beam, BERv2_beam] = biterr(in2,dataOutv2_beam);

BER_v1=[BERv1,BERv1_beam];
BER_v2=[BERv2,BERv2_beam];


formatSpec = "BER v1: \n Without BF = %G,\n With    BF = %d, ";
str = sprintf(formatSpec,BER_v1(1),BER_v1(2))


formatSpec = "BER v2: \n Without BF = %G,\n With    BF = %d, ";
str = sprintf(formatSpec,BER_v2(1),BER_v2(2))


