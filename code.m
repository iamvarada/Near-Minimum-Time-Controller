% Computer project ME 554 
% By: Krishna Varadarajan
% plotting the control input and states - sample time already chosen =
% Sampling time  = 0.03s
%--------------------------------------------

clear all;
clc;

h = 0.03; % sampling period in secs. - CHOSEN FROM |C| VS h PLOT

% define variables
J1 = 0.0024; J2 = 0.0019; J3 = 0.0019;
k1 = 2.8; k2 = 2.8;
c1 = 0.007; c2 = 0.001; c3 = 0.001;

thetaf = 30*pi/180; % final angle - given

% continous time state matrices
A = [0 1 0 0 0 0;
    -k1/J1 -c1/J1 k1/J1 0 0 0;
    0 0 0 1 0 0;
    k1/J2 0 -(k1+k2)/J2 -c2/J2 k2/J2 0;
    0 0 0 0 0 1;
    0 0 k2/J3 0 -k2/J3 -c3/J3];

B = [0 1/J1 0 0 0 0]';

C = [1 0 0 0 0 0];

D = 0;

sys = ss(A,B,C,D); % Continous-time ss model
sysd = c2d(sys,h); % converting to discrete-time model

% Discrete-time ss matrices
[Phi,Gamma,c,d] = ssdata(sysd);

P1 = Phi\Gamma;
P2 = (Phi*Phi)\Gamma;
P3 = (Phi*Phi*Phi)\Gamma;
P4 = (Phi*Phi*Phi*Phi)\Gamma;
P5 = (Phi*Phi*Phi*Phi*Phi)\Gamma;
P6 = (Phi*Phi*Phi*Phi*Phi*Phi)\Gamma;

P = [P1 P2 P3 P4 P5 P6];
Pinv = inv(P);


% gain matrix is the first row of P matrix 
L = Pinv(1,:); 

% input control torque vector for pure deadbeat
TDb = zeros(10,1);
Tm = zeros(10,1);

% state matrix
x = zeros(6,10);

%initial state matrix; (k==1 => t == 0)
x(:,1) = [0;0;0;0;0;0]; % initial states are given as 0

% final state matrix
xf = [thetaf,0,thetaf,0,thetaf,0]'; 

% output matrix
y = zeros(10,1);

% define output state vector not required since output not asked
C = [];

% Calculating the response, and plot sequence by shifting the origin
% Step 1: find Tmax of deadbeat

for k=1:10
    x(1:6,k+1) = (Phi-Gamma*L)*(x(1:6,k)-xf)+xf; % (phi-gamma*L)*delx
%     y(k,1) = C*x(1:4,k); %output response
    
    TDb(k,1) = -L*(x(1:6,k)-xf); % input sequence = -L*x(k)
end

Tmax = 0.05*max(abs(TDb)); % max T for minimum-time control a/c to question

% Calculating the response, and plot sequence by shifting the origin
% Step 2: find maximum time taken by the states to be driven to final state

for k=1:30
    Tm(k,1) = -L*(x(1:6,k)-xf); % input sequence = -L*x(k)
    
    % restrict control input for minimum time control
    if abs(Tm(k,1)) > Tmax
        Tm(k,1) = sign(Tm(k,1))*Tmax;
    else
        Tm(k,1) = Tm(k,1);
    end
    
    % now calculate the states
     x(1:6,k+1) = Phi*(x(1:6,k)-xf)+xf + Gamma*Tm(k,1);

end

k =1:1:30;

subplot(3,1,1)
plot((k-1)*0.03,x([1,3,5],k))
xlabel('Time in seconds','Interpreter','latex','FontSize',16)
ylabel('Angle in radians','Interpreter','latex','FontSize',16)
title('Angle of the disks (rad) Vs Time (s)')
legend({'$\theta_1$','$\theta_2$','$\theta_3$'},'Interpreter','latex')


subplot(3,1,2)
plot((k-1)*0.03,x([2,4,6],k))
xlabel('Time in seconds','Interpreter','latex','FontSize',16)
ylabel('Angular velocity in radians/s','Interpreter','latex','FontSize',16)
title('Speed of the disks (rad/s) Vs Time (s)')
legend({'$\dot{\theta}_1$','$\dot{\theta}_2$','$\dot{\theta}_3$'},'Interpreter','latex')

subplot(3,1,3)
plot((k-1)*0.03,Tm(k,1))
xlabel('Time in seconds','Interpreter','latex','FontSize',16)
ylabel('Control Input Torque (N-m)','Interpreter','latex','FontSize',16)
title('Control Input Torque (N-m) Vs Time (s)')
legend({'T(t)'},'Interpreter','latex')

% overall title
suptitle('States response, and control input for minimum time control, By: Krishna Varadarajan')

%---------End of code-------------