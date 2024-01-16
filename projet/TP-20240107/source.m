%车辆悬浮系统控制
clear all;
m1=1.5e3;m2=1.0e4;
k1=5.0e6;k2=5.0e5;
b1=1.7e3;b2=50e3;
A=[0 0 1 0;0 0 0 1;-(k1+k2)/m1 k2/m1 -(b1+b2)/m1 b2/m1;k2/m2 -k2/m2 b2/m2 -b2/m2];
B=[b1/m1 0;0 0;k1/m1-(b1*(b1+b2))/(m1*m1) -1/m1;(b1*b2)/(m1*m2) 1/m2];
C1=[1 0 0 0;0 0 0 0;k2/m2 -k2/m2 b2/m2 -b2/m2;-1 1 0 0];
D1=[-1 0;0 1;(b1*b2)/(m1*m2) 1/m2;0 0];
C2=[k2/m2 -k2/m2 b2/m2 -b2/m2; -1 1 0 0];
D2=[(b1*b2)/(m1*m2) 1/m2;0 0];
sysG=ltisys(A,B,[C1;C2],[D1;D2]);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%    双子端模型的传递函数  %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
syswq0=ltisys('tf',0.01,[0.4 1]);%考虑权重
syswz1=ltisys('tf',200,1);
syswz2=ltisys('tf',0.1,1);
syswz3=ltisys('tf',[3.18e-2 0.4],[3.16e-4 3.14e-2 1]);
syswz4=ltisys('tf',100,1);
syswz5=ltisys('tf',1,1);
syswz=sdiag(syswz1,syswz2,syswz3,syswz4,syswz5,syswz5);
syswq=sdiag(syswq0,syswz5);
sys=smult(syswq,sysG,syswz);
[gopt,K]=hinflmi(sys,[2 1]);
[Ak,Bk,Ck,Dk]=ltiss(K);%相应的闭环系统最优Hinf性能指标＝0.5628，