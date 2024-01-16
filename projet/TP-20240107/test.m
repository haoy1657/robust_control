clear;
close all;
clc;

% 基础模型
A1 = 0.0256;	% Area of tank 1 (hunits^2)
A2 = 0.0477;	% Area of tank 2 (hunits^2)
h2 = 0.241;	    % Height of tank 2, fixed by overflow (hunits)
fb = 3.28e-5;   % Bias stream flow (hunits^3/sec)
fs = 0.00028;	% Flow scaling (hunits^3/sec/funit)
th = 1.0;	    % Hot water supply temp (tunits)
tc = 0.0;	    % Cold water supply temp (tunits)
tb = tc;	    % Cold bias stream temp (tunits)
alpha = 4876;   % Constant for flow/height relation (hunits/funits)
beta = 0.59;    % Constant for flow/height relation (hunits)

h1ss = 0.75;                            % Water level for tank 1
t1ss = 0.75;                            % Temperature of tank 1
f1ss = (h1ss+beta)/alpha;               % Flow tank 1 -> tank 2
fss = [th,tc;1,1]\[t1ss*f1ss;f1ss];
fhss = fss(1);                          % Hot flow
fcss = fss(2);                          % Cold flow
t2ss = (f1ss*t1ss + fb*tb)/(f1ss + fb); % Temperature of tank 2 

A = [ -1/(A1*alpha),          0;
      (beta*t1ss)/(A1*h1ss),  -(h1ss+beta)/(alpha*A1*h1ss)];

B = fs*[ 1/(A1*alpha),   1/(A1*alpha);
         th/A1,          tc/A1];

C = [ alpha,             0;
      -alpha*t1ss/h1ss,  1/h1ss];

D = zeros(2,2);
tank1nom = ss(A,B,C,D,'InputName',{'fh','fc'},'OutputName',{'h1','t1'});

clf
step(tank1nom), title('Step responses of Tank 1')


inputs = {'t1cmd', 'tdiffcmd', 't1noise', 't2noise', 'fhc', 'fcc'};
outputs = {'y_Wt1perf', 'y_Wt2perf', 'y_Whact', 'y_Wcact', ...
             'y_Whrate', 'y_Wcrate', 'y_Wt1cmd', 'y_t1diffcmd', ...
                                           'y_t1Fn', 'y_t2Fn'};

hot_act.InputName = 'fhc'; hot_act.OutputName = {'fh' 'fh_rate'};
cold_act.InputName = 'fcc'; cold_act.OutputName = {'fc' 'fc_rate'};

tank1and2u.InputName = {'fh','fc'};
tank1and2u.OutputName = {'t1','t2'};

t1F.InputName = 't1'; t1F.OutputName = 'y_t1F';
t2F.InputName = 't2'; t2F.OutputName = 'y_t2F';

Wt1cmd.InputName = 't1cmd'; Wt1cmd.OutputName = 'y_Wt1cmd';
Wtdiffcmd.InputName = 'tdiffcmd'; Wtdiffcmd.OutputName = 'y_Wtdiffcmd';

Whact.InputName = 'fh'; Whact.OutputName = 'y_Whact';
Wcact.InputName = 'fc'; Wcact.OutputName = 'y_Wcact';

Whrate.InputName = 'fh_rate'; Whrate.OutputName = 'y_Whrate';
Wcrate.InputName = 'fc_rate'; Wcrate.OutputName = 'y_Wcrate';

Wt1perf.InputName = 'u_Wt1perf'; Wt1perf.OutputName = 'y_Wt1perf';
Wt2perf.InputName = 'u_Wt2perf'; Wt2perf.OutputName = 'y_Wt2perf';

Wt1noise.InputName = 't1noise'; Wt1noise.OutputName = 'y_Wt1noise';
Wt2noise.InputName = 't2noise'; Wt2noise.OutputName = 'y_Wt2noise';

sum1 = sumblk('y_t1diffcmd = y_Wt1cmd + y_Wtdiffcmd');
sum2 = sumblk('y_t1Fn = y_t1F + y_Wt1noise');
sum3 = sumblk('y_t2Fn = y_t2F + y_Wt2noise');
sum4 = sumblk('u_Wt1perf = y_Wt1cmd - t1');
sum5 = sumblk('u_Wt2perf = y_Wtdiffcmd + y_Wt1cmd - t2');

% This produces the uncertain state-space model
P = connect(tank1and2u,hot_act,cold_act,t1F,t2F,Wt1cmd,Wtdiffcmd,Whact, ...
                Wcact,Whrate,Wcrate,Wt1perf,Wt2perf,Wt1noise,Wt2noise, ...
                   sum1,sum2,sum3,sum4,sum5,inputs,outputs);

disp('Weighted open-loop model: ')
P
