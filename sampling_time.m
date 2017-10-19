% Computer project ME 554 
% By: Krishna Varadarajan
% choosing sampling time period, h or Ts
%--------------------------------------------

clear all;
clc;

J1 = 0.0024; J2 = 0.0019; J3 = 0.0019;
k1 = 2.8; k2 = 2.8;
c1 = 0.007; c2 = 0.001; c3 = 0.001;

% continous time state matrices
A = [0 1 0 0 0 0;
    -k1/J1 -c1/J1 k1/J1 0 0 0;
    0 0 0 1 0 0;
    k1/J2 0 -(k1+k2)/J2 -c2/J2 k2/J2 0;
    0 0 0 0 0 1;
    0 0 k2/J3 0 -k2/J3 -c3/J3];

B = [0 1/J1 0 0 0 0]';

C = [1 0 0 0 0 0]; % chosen randomly, doesn't affect our objective

D = 0; % can be chosen randomly, doesn't affect our objective

sys = ss(A,B,C,D); % create continous-time state space system
sysd = c2d(sys,0.01); % discretize with a sampling period
[a,b,c,d] = ssdata(sysd); % extract the D-T system matrices

DetC = zeros(10,1); % initialize a matrix to store determinants of C
k = 1; % initialize counting variable k

% calculate determinant of controllability matrix for a range of sampling
% period
for h = 0.001:0.001:0.01,
    sysd =  d2d(sysd,h);
    [a,b,c,d] = ssdata(sysd); 
    ContMat = [b a*b a*a*b a*a*a*b a*a*a*a*b a*a*a*a*a*b]; % controllability matrix
    DetC(k,1) = det(ContMat);
    k=k+1;   
      
end

% plot the detrminant of CTRLB matrix versus sampling period
DetC
k = 1:1:10;
plot(k*0.01,DetC(k));
xlim([0.01,0.1])
xlabel('Sampling periods, Ts or h','Interpreter','latex');
ylabel('Det(C)','Interpreter','latex');
title('Computer Project 1 - ME 554: Choosing a sampling time - By: Krishna Varadarajan','Interpreter','latex');
