%% Quarter Car Suspension Hinf control Ok

clear;
close all;
clc;


%% I- Quarter Car Suspension Model

% definir model nominal 

ms = 300;    % kg
mus = 60;     % kg
bs = 1000;   % N/m/s
ks = 16000 ; % N/m
kt = 190000; % N/m



A = [ 0 1 0 0; [-ks -bs ks bs]/ms;  0 0 0 1; [ks bs -ks-kt -bs]/mus];
B= [0 0; 0 1e3/ms;0 0; [kt -1e3]/mus];
C =[1 0 0 0;1 0 -1 0;[-ks -bs ks bs]/ms]; 
D =[0 0;0 0;0 1e3/ms];

qcar=ss(A,B,C,D);

qcar.StateName = {'body_travel';'body_vel';'wheel_travels';'wheel_vel'};
qcar.InputName = {'r';'fs'};
qcar.OutputName = {'body_travel';'suspension_travel';'body_acceleration'};

G11=qcar('body_travel','r');%比如说这里是bodytravel/f---xs/r
G21=qcar('suspension_travel','r');%(xs-xus)/r
G31=qcar('body_acceleration','r');%xs_dotdot/r

G12=qcar('body_travel','fs');%xs/fs
G22=qcar('suspension_travel','fs');%(xs-xus)/fs 
G32=qcar('body_acceleration','fs');%xs_dotdot/fs

% definition des ponderation 
Wroad = ss(0.07); 
Wd2 = ss(0.01);  
Wd3 = ss(0.5); 

Wact_bis = 0.75*tf([1 70],[1 700]);  
Wact = 0.8*tf([1 50],[1 500]);


% % HandlingTarget
G_HT = 0.04 * tf([1/8 1],[1/80 1]);
[bode_mag, bode_freq] = bode(G_HT);
[bode_mag_dB, idx] = max(bode_mag);
cutoff_freq = bode_freq(idx);
fprintf('cutoff_freq of G_HT %.2f Hz\n', cutoff_freq);
% % ComfortTarget 
G_CT = 0.4 * tf([1/0.45 1],[1/150 1]); 
[bode_mag, bode_freq] = bode(G_CT);
[bode_mag_dB, idx] = max(bode_mag);
cutoff_freq = bode_freq(idx);
fprintf('cutoff_freq of G_CT %.2f Hz\n', cutoff_freq);

% 5

figure(3)
subplot(211)
bodemag (G21*Wroad,'b');
hold on;
bodemag (G_HT,'r--');

subplot(212)
bodemag (G31*Wroad,'b');
hold on;
bodemag (G_CT,'r--');


%% Mu synthesis

beta = [0.01 0.5 0.99]; % comfort (?=0.01), balanced (?=0.5), and handling (?=0.99).

Wsd_all = beta/G_HT;
Wab_all = (1-beta)/G_CT;

Act= tf(1,[1/60 1]); % actuator model

% modelisation incertitude
% Uncertain Actuator Model 
% P_uncertain = uss(A1,B1,C1,1,'unc',Act_uncertain);
Wunc = makeweight(0.40,15,3);
unc = ultidyn('unc',[1 1],'SampleStateDim',5);
Act_uncertain = Act*(1 + Wunc*unc);

figure
bode(Act_uncertain,'b',Act_uncertain.NominalValue,'r+');
legend('Act_uncertain','Act_uncertain.NominalValue')

Wsd=Wsd_all(1);    
Wab=Wab_all(1);
open_system('QCS_Sim_miu');
io = getlinio('QCS_Sim_miu');
op = operpoint('QCS_Sim_miu');


% confortable mode
% Controller K1 with beta=0.01;
% utiliser ulinearize pour linearise model,ici on utilise pas "linearize"
% parceque linearize prendre pas en compte incertitude

% dans le model simulink, on ajoute la modelisation d'incertitude
sys = ulinearize('QCS_Sim_miu');
% controler deux sortie y1et y2 et puis on les additionne pour obtenir
% commande u
[K1, clp] = musyn(sys,2,1);
K1=tf(K1);

% balanced mode
% Controller K1 with beta=0.01;
Wsd=Wsd_all(2);    
Wab=Wab_all(2);

sys = ulinearize('QCS_Sim_miu');
[K2, clp] = musyn(sys,2,1);
K2=tf(K2);

% handling mode
% Controller K1 with beta=0.01;
Wsd=Wsd_all(3);    
Wab=Wab_all(3);

sys = ulinearize('QCS_Sim_miu');
[K3, clp] = musyn(sys,2,1);
K3=tf(K3);



% Closed loop models with K1 comfort (?=0.01)

K1_1=K1(1); % u/suspension_travel 
K1_2=K1(2); % u/body_acceleration 

Gru=((K1_1*G21)+(K1_2*G31))/(1-(((K1_1*G22)+(K1_2*G32))*Act)); % transfer u/r

Gcl1_11 = tf(G11+(G12*Act*Gru)); % transfer xs/r (body_travel/road disturbance)

Gcl1_21 = tf(G21+(G22*Act*Gru)); % transfer sd/r (suspension_travel/road disturbance)

Gcl1_31 = tf(G31+(G32*Act*Gru)); % transfer ddotxs/r (body_acceleration/road disturbance)




% Closed loop models with K2  balanced (?=0.5)

K2_1=K2(1); % u/suspension_travel 
K2_2=K2(2); % u/body_acceleration 

Gru=((K2_1*G21)+(K2_2*G31))/(1-(((K2_1*G22)+(K2_2*G32))*Act)); % transfer u/r

Gcl2_11 = tf(G11+(G12*Act*Gru)); % transfer xs/r (body_travel/road disturbance)

Gcl2_21 = tf(G21+(G22*Act*Gru)); % transfer sd/r (suspension_travel/road disturbance)

Gcl2_31 = tf(G31+(G32*Act*Gru)); % transfer ddotxs/r (body_acceleration/road disturbance)


% Closed loop models with K3 handling (?=0.99).


K3_1=K3(1); % u/suspension_travel 
K3_2=K3(2); % u/body_acceleration 

Gru=((K3_1*G21)+(K3_2*G31))/(1-(((K3_1*G22)+(K3_2*G32))*Act)); % transfer u/r

Gcl3_11 = tf(G11+(G12*Act*Gru)); % transfer xs/r (body_travel/road disturbance)

Gcl3_21 = tf(G21+(G22*Act*Gru)); % transfer sd/r (suspension_travel/road disturbance)

Gcl3_31 = tf(G31+(G32*Act*Gru)); % transfer ddotxs/r (body_acceleration/road disturbance)

% 10-

figure(4)

bodemag(Gcl1_11,'b');hold on;bodemag(Gcl2_11,'r');hold on;bodemag(Gcl3_11,'g');hold on;bodemag(G11,'k');
legend('confortable','balanced','handling','initial')
title('xs/r')

figure(5)
bodemag(Gcl1_21,'b');hold on;bodemag(Gcl2_21,'r');hold on;bodemag(Gcl3_21,'g');hold on;bodemag(G21,'k');
legend('confortable','balanced','handling','initial')
title('sd/r')

figure(6)
bodemag(Gcl1_31,'b');hold on;bodemag(Gcl2_31,'r');hold on;bodemag(Gcl3_31,'g');hold on;bodemag(G31,'k');
legend('confortable','balanced','handling','initial')
title('ddotxs/r')








