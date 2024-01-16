%% Quarter Car Suspension Hinf control Ok

clear;
close all;
clc;


%% I- Quarter Car Suspension Model

% 1-

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

% U1=r;
% U2=fs;

% Y1=body_travel;
% Y2=suspension_travel;
% Y3=body_acceleration;
% 定义传递函数，之前定义了每个变量的名字这里直接引用
G11=qcar('body_travel','r');%比如说这里是bodytravel/f---xs/r
G21=qcar('suspension_travel','r');%(xs-xus)/r
G31=qcar('body_acceleration','r');%xs_dotdot/r

G12=qcar('body_travel','fs');%xs/fs
G22=qcar('suspension_travel','fs');%(xs-xus)/fs 
G32=qcar('body_acceleration','fs');%xs_dotdot/fs
% 
% % 2-
% % 
% figure(1)
% 
% bodemag(G31,'b');
% hold on;
% bodemag(G32,'--r');
% legend('body acceleration/road disturbance','body acceleration/force fs');
% grid on;
% % 
% % 
% figure(2)
% 
% bodemag(G21,'b');
% hold on;
% bodemag(G22,'--r');
% legend('suspension_travel sd/road disturbance','suspension_travel sd/force fs');
% grid on;
% 

% %  tire-hop= 56.3 rad/s
% %  rattlespace= 23 rad/s
% 
% 
% 
% 
% %% II- Linear Hinf Controller Design
% 
% 3-

Wroad = ss(0.07); %加权函数设置为常数 
Wd2 = ss(0.01);  
Wd3 = ss(0.5);  

Wact_bis = 0.75*tf([1 70],[1 700]);  
Wact = 0.8*tf([1 50],[1 500]);


% figure
% bodemag(Wact)
% % %Wact = (100/13)*tf([1 50],[1 500]);

% 4-
% G21 =sd/r 
% wroad*G21=sd/d1

% G31=xs_dotdot/r
% G31*wroad=xs_dotdot/d1
% 路面干扰d1 经过常数放大器到r形成路面干扰信号输入系统
%看路面干扰信号d1对sd和xsdotdot的敏感度，利用h无穷把这个灵敏度降到最低

% % HandlingTarget,r 到sd的闭环增益 0.04 0.4
G_HT = 0.04 * tf([1/8 1],[1/80 1]);
[bode_mag, bode_freq] = bode(G_HT);
[bode_mag_dB, idx] = max(bode_mag);
cutoff_freq = bode_freq(idx);
fprintf('cutoff_freq of G_HT %.2f Hz\n', cutoff_freq);
% % ComfortTarget r到xs_dotdot闭环增益
G_CT = 0.4 * tf([1/0.45 1],[1/150 1]); 
[bode_mag, bode_freq] = bode(G_CT);
[bode_mag_dB, idx] = max(bode_mag);
cutoff_freq = bode_freq(idx);
fprintf('cutoff_freq of G_CT %.2f Hz\n', cutoff_freq);

% 5-
% handling模式设计GHT来压低sd对路面干扰的敏感度
% 在操作模式下，我们关注底盘间距变化
figure(3)
subplot(211)
bodemag (G21*Wroad,'b');
hold on;
bodemag (G_HT,'r--');

% 舒适模式下我们只关注底盘上下浮动加速度
%comfortable模式 设计GCT 来压低xsdotdot对路面干扰的敏感度
subplot(212)
bodemag (G31*Wroad,'b');
hold on;
bodemag (G_CT,'r--');



% % % % 这部分我的理解是，找到合理GHT 和GCT来近似sd和xsdotdot对路面干扰的敏感度，要求
% 近似的bode图在原始图最大增益的上面，这样我们之后只需要对这个近似的频域信号进行h无穷控制，就基本实现了
% 对原始灵敏度函数的压制，

% sd和xsdotdot对路面干扰信号d1的灵敏度
% 对其进行h无穷控制等价于，对比其更糟的情况进行h无穷控制，这样做是为了更好的进行数学分析，
% 因为原始频谱比较困难用数学描述

% 6-

% 这里面的beta是h无穷最后的gamma，数值越大惩罚越小吗
% 这里beta是权重不是最后的gamma
beta = [0.01 0.5 0.99]; % comfort (?=0.01), balanced (?=0.5), and handling (?=0.99).
% 先通过设计wsd对Ght形状进行限制
Wsd_all = beta/G_HT;
Wab_all = (1-beta)/G_CT;

Act= tf(1,[1/60 1]); % actuator model

% Controller K1 with beta=0.01;

Wsd=Wsd_all(1);    
Wab=Wab_all(1);

[A1,B1,C1,D1]=linmod('QCS_Sim');

P1=ss(A1,B1,C1,D1);
% 为什么是 2 1 2 代表输出e1 和e2 1代表输入d1
% P1中截取2乘以1尺寸做成 h无穷处理的矩阵P，然后设计K使P稳定
% 同时 2 1也是说去前两个输出做两个反馈控制，合并到一个u上面
% 这里xs没有接线所以前两个动态是sd 和xsdotdot，2 --》1的意思是从前两个动态负反馈控制
% 而hinfsyn是最小化前两个动态通过加权后的e1 和e2 对第一个输入也就是d1的灵敏度
[K1,Scl1,gam1] = hinfsyn(P1,2,1);

K1=tf(K1);

% Controller K2 with beta=0.5;

Wsd=Wsd_all(2);
Wab=Wab_all(2);

% 实际上增广模型 就是FPK笔记那个e y-》d u是观测矩阵，动态矩阵A2和B2要包含所有动态，
% d1 d2 d3 u再加上qcar的四个动态

[A2,B2,C2,D2]=linmod('QCS_Sim');

P2=ss(A2,B2,C2,D2);

[K2,Scl2,gam2] = hinfsyn(P2,2,1);

K2=tf(K2);

% Controller K3 with beta=0.99;

Wsd=Wsd_all(3);
Wab=Wab_all(3);


[A3,B3,C3,D3]=linmod('QCS_Sim');

P3=ss(A3,B3,C3,D3);

[K3,Scl3,gam3] = hinfsyn(P3,2,1);

K3=tf(K3);
% 输出gam1 gam2 gam3
% gamma
% 0.9410
% 0.6724
% 0.8877



% 9-
% Closed loop models with K1 comfort (?=0.01)
% 这里没毛病，设计K1和K2来控制e1和e2对d1的灵敏度，然后输出K1和K2一共三组    
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






% Road disturbance

Fs=1e3;
Ts=1/Fs;

t = 0:Ts:1;
roaddist = zeros(size(t));
nn=round(length(roaddist)/8);
dd=round(length(t));
roaddist(250:250+nn) = 0.025*(1-cos(8*pi*t(250:250+nn)));

sd_comfort=lsim(Gcl1_21,roaddist,t);
sd_balanced=lsim(Gcl2_21,roaddist,t);
sd_handling=lsim(Gcl3_21,roaddist,t);
sd_no_control=lsim(G21,roaddist,t);

xsddot_comfort=lsim(Gcl1_31,roaddist,t);
xsddot_balanced=lsim(Gcl2_31,roaddist,t);
xsddot_handling=lsim(Gcl3_31,roaddist,t);
xsddot_no_control=lsim(G31,roaddist,t);

figure(7)
plot(t,roaddist,'k'); 
hold on;
plot(t,sd_comfort,'b'); 
hold on;
plot(t,sd_balanced,'r'); 
hold on;
plot(t,sd_handling,'g'); 
legend ('road disturbance', 'suspension travel comfort', 'suspension travel balanced', 'suspension travel handling');


figure(8)
plot(t,roaddist,'k'); 
hold on;
plot(t,xsddot_comfort,'g'); 
hold on;
plot(t,xsddot_balanced,'b'); 
hold on;
plot(t,xsddot_handling,'r'); 
legend ('road disturbance', 'body acceleration comfort', 'body acceleration balanced', 'body acceleration handling');





%% Mu synthesis

% Uncertain Actuator Model
Wunc = makeweight(0.40,15,3);
unc = ultidyn('unc',[1 1],'SampleStateDim',5);

Act_uncertain = Act*(1 + Wunc*unc);

beta = [0.01 0.5 0.99]; % comfort (?=0.01), balanced (?=0.5), and handling (?=0.99).
% 先通过设计wsd对Ght形状进行限制
Wsd_all = beta/G_HT;
Wab_all = (1-beta)/G_CT;

Act= tf(1,[1/60 1]); % actuator model

% Controller K1 with beta=0.01;

Wsd=Wsd_all(1);    
Wab=Wab_all(1);

[A1,B1,C1,D1]=linmod('QCS_Sim_miu');

P1=ss(A1,B1,C1,D1);
% 为什么是 2 1 2 代表输出e1 和e2 1代表输入d1
% P1中截取2乘以1尺寸做成 h无穷处理的矩阵P，然后设计K使P稳定
% 同时 2 1也是说去前两个输出做两个反馈控制，合并到一个u上面
% 这里xs没有接线所以前两个动态是sd 和xsdotdot，2 --》1的意思是从前两个动态负反馈控制
% 而hinfsyn是最小化前两个动态通过加权后的e1 和e2 对第一个输入也就是d1的灵敏度
[Kdk1,CLdk,gdk] = dksyn(P1,2,1);

K1=tf(Kdk1);

figure(9)
bode(Act_uncertain,'b',Act_uncertain.NominalValue,'r+');


% Select=1; %comfort
% 
% sim('QCS_Closed_Loop_Sim');
% 
% sd_simC=sd_sim;
% xsddot_simC=xsddot_sim;
% 
% Select=2; %balanced
% 
% sim('QCS_Closed_Loop_Sim');
% 
% sd_simB=sd_sim;
% xsddot_simB=xsddot_sim;
% 
% Select=3; %handling
% 
% sim('QCS_Closed_Loop_Sim');
% 
% sd_simH=sd_sim;
% xsddot_simH=xsddot_sim;
% 
% 
% figure(111)
% 
% plot(sd_simC,'b')
% hold on;
% plot(sd_simB,'k')
% hold on;
% plot(sd_simH,'r')
% 
% 
% figure(112)
% 
% plot(xsddot_simC,'b')
% hold on;
% plot(xsddot_simB,'k')
% hold on;
% plot(xsddot_simH,'r')



