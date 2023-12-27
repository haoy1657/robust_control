%exo7
clear;
clear all;
clc;
%partie1
p=tf('p');
Gn=0.8425/(((5.699e-8)*p^2)+((4.774e-6)*p)+1);
G1=0.8/(((5.55e-8)*p^2)+((3.53e-6)*p)+1);
G2=0.9133/(((5.82e-8)*p^2)+((4.01e-6)*p)+1);
% damp(Gn);
% dcgain(Gn);
% Kn=0.8425;
% wn=-4.19e+03;
% xin=1e-02;
% 
% damp(G1);
% dcgain(G1);
% K1=0.8;
% w1=4.24e+03;
% xi1=7.49e-03;
% 
% damp(G2);
% dcgain(G2);
% K2=0.9133;
% w2=4.15e+03;
% xi2=8.31e-03;


damp(Gn);
dcgain(Gn);

K=0.8425;
wn=4.19e+03 ;
xi=1.00e-02;

G_2nd=(K*wn^2)/((p^2)+(2*xi*wn*p)+wn^2);

figure(1)
step(Gn,'k');
hold on;
step(G_2nd,'.-r');

figure(2);
step(Gn,'k');
hold on;
step(G1,'b');
hold on;
step(G2,'r');

figure(3);
bode(Gn,'k');
hold on;
bode(G1,'b');
hold on;
bode(G2,'r');
%partie2
figure(4);
sigma((G1-Gn)/Gn);
hold on
sigma((G2-Gn)/Gn);
hold on;

% choix du filtre W3
w1=1000;
w2=2200;

W3=tf([1/w1 1],[1])*tf([1/w1 1],[1])*tf([1/w1 1],[1])*tf([1],[1/w2 1])*tf([1],[1/w2 1])*tf([1],[1/w2 1])*0.23;

sigma(W3);figure(4);
sigma((G1-Gn)/Gn);
hold on
sigma((G2-Gn)/Gn);
hold on;

% choix du filtre W3
w1=1000;
w2=2200;
W3=tf([1/w1 1],[1])*tf([1/w1 1],[1])*tf([1/w1 1],[1])*tf([1],[1/w2 1])*tf([1],[1/w2 1])*tf([1],[1/w2 1])*0.23;

sigma(W3);
%静态误差取0.5即可
% choix du filtre W2
W2=tf([1],[1.2]);

% choix du filtre W1 par la méthode du modèle
% on veut en BF un ksi=0.7, K=1, et Tr=0.1s
wn=30;
ksi=0.7;
K=0.995;
Td=tf([K],[1/wn^2 2*ksi/wn 1]);
Sd=1-Td;
W1=1/Sd;


[A,B,C,D]=linmod('TD_ex7');

Psys=ltisys(A,B,C,D);

[gopt, K]=hinfric(Psys,[1,1]);

[Ak,Bk,Ck,Dk]=unpck(K);

K=tf(ss(Ak,Bk,Ck,Dk));
%然后程序很自己寻找最佳值gamma
%缺少lumod





