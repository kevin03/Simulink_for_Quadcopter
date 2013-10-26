%% Level-2 M file S-Function for Neural Network Adaptive Control.
%   by Zhao Weihua
%   09Jan2011 
%% Modification made by Zhao weihua 15Feb2011
% add a deadzone for the tracking error to avoid the high frequecy
% unmoldeled dynamics
% The length of the deadzone is DialogPrm(12)


%% Modification made by Zhao weihua 19Feb2011
%  normalize the inupts of the NN
% the normalize equation: delta = (d-d_min)/(d_max - d_min);
% d: input signal needs normalization
% delta: signal after normalization
% d_min: minimum  of d
% d_max: maximum  of d
% 
% y_min =  DialogPrm(13)
% y_max =  DialogPrm(14)
% v_min =  DialogPrm(15)
% v_max =  DialogPrm(16)


%% inputs, outputs, DialogParameters of the NN block
% Inputs Signals
% input(1): y_c(n)----scalar --- filtered Commanded signal 
% input(2):  y(n) ----scalar --- plant output
% input(3):  v(n) = y_c^(r) + v_0 - v_ad + v_bar ----scalar --- pseudo control
% input(4): y_ad ---- compensator state

% Output Signals
% output(1): v_ad --- scalar --- adaptive control
% output(2): v_bar--- scalar --- robustifying term
% output(3): V --- (n1 +1)x(n2) --- NN input weighting matrix
% output(4): W --- (n2 +1)x(1) --- NN output weighting vector

% DialogParameters
% DialogPrm(1): n --- system order
% DialogPrm(2): r --- system relative degree
% DialogPrm(3): n2 --- Number of neurons 
% DialogPrm(4): NN_ST --- sample time (delta_t)
% DialogPrm(5): GAMA_V
% DialogPrm(6): gama_V_e
% DialogPrm(7): GAMA_W
% DialogPrm(8): gama_W_e
% DialogPrm(9): alpha
% DialogPrm(10): c_f
% DialogPrm(11): c_y_tilt
% DialogPrm(12): NN_Deadzone
% DialogPrm(13): minimum of y
% DialogPrm(14): maximum of y
% DialogPrm(15): minimum of v
% DialogPrm(16): maximum of v

%% NN structure
%
% input layer (X_bar): length = (2 + 2*n - r)
% actual input n1 = (1 + 2*n -r)
% 
% Input weighting matrix: V --- (n1 +1)x(n2) = (2 + 2*n - r)x(n2)
%                    [ theta_v_1, ... ... ... ,theta_v_n2;
%                        v_1_1,   ... ... ... ,v_1_n2;
%                         ...,   , ... ... ... ,  ....    ;
%                        v_n1_1,  ... ... ... ,v_n1_n2]

% Neuron sigmoidal activation function: NN_sigma = 1/(1 + e^(-a*z))

% X_bar = [b_v(1), y_c(t), y(t), y(t-d), ... y(t - (n-1)d),v(t), v(t-d), ... v(t-(n-r-1)d)]'
% 
% NN_sigma_bar = [b_w; NN_sigma(1);...; NN_sigma(n1)];

% Input weighting vector: W --- (n2 +1)x(1)
%                    [theta_w;w_1;...;w_n2];

% v_ad = W'*NN_sigma_bar(V'X_bar);


%% 

function msfcn_Adaptive_NN_HeightOnly(block)

  setup(block);
  
%endfunction

function setup(block)
  %% read parameters from the Dialogbox
  block.NumDialogPrms  = 16;
  n = block.DialogPrm(1).Data; %system order
  r = block.DialogPrm(2).Data; %system relative degree
  n2 = block.DialogPrm(3).Data; %Number of neurons 
  NN_ST = block.DialogPrm(4).Data; %NN sample time
 
%   global n r n2 NN_ST

  
  %% Register number of input and output ports
  block.NumInputPorts  = 4;
  block.NumOutputPorts = 4;

  %% Setup functional port properties to dynamically
  %% inherited.
  block.SetPreCompInpPortInfoToDynamic;
  block.SetPreCompOutPortInfoToDynamic;
 
  %% Input Ports setup
%   input(1): y_c(n)----scalar --- filtered Desired(reference) signal 
  block.InputPort(1).Dimensions        = 1;
  block.InputPort(1).SamplingMode = 'Sample';
  block.InputPort(1).DirectFeedthrough = true;

%   input(2):  y(n) ----scalar --- plant output
  block.InputPort(2).Dimensions        = 1;
  block.InputPort(2).SamplingMode = 'Sample';
  block.InputPort(2).DirectFeedthrough = true;
  
% input(3):  v(n) = y_c^(r) + v_0 - v_ad + v_bar ----scalar --- pseudo
% control
  block.InputPort(3).Dimensions        = 1;
  block.InputPort(3).SamplingMode = 'Sample';
  block.InputPort(3).DirectFeedthrough = true;
  
%   input (4): y_ad --- compensator state
  block.InputPort(4).Dimensions        = 1;
  block.InputPort(4).SamplingMode = 'Sample';
  block.InputPort(4).DirectFeedthrough = false;

  
  %% Output Ports setup
%   output(1): v_ad --- scalar --- adaptive control
  block.OutputPort(1).Dimensions       = 1;
  block.OutputPort(1).SamplingMode = 'Sample';

% output(2): v_bar--- scalar --- robustifying term
  block.OutputPort(2).Dimensions       = 1;
  block.OutputPort(2).SamplingMode = 'Sample';
  
% output(3): V --- (n1 +1)x(n2) --- NN input weighting matrix
  block.OutputPort(3).Dimensions       = [2+2*n-r,n2];
  block.OutputPort(3).SamplingMode = 'Sample';
  
% output(4): W --- (n2 +1)x(1) --- NN output weighting vector
  block.OutputPort(4).Dimensions       = n2 + 1;
  block.OutputPort(4).SamplingMode = 'Sample';
  
  %% Set block sample time to inherited
  block.SampleTimes = [NN_ST, 0];
  
  %% Set the block simStateComliance to default (i.e., same as a built-in block)
  block.SimStateCompliance = 'DefaultSimState';

  %% Register methods
  block.RegBlockMethod('PostPropagationSetup',    @DoPostPropSetup);
  block.RegBlockMethod('InitializeConditions',    @InitConditions);  
  block.RegBlockMethod('Outputs',                 @Output);  
%   block.RegBlockMethod('Update',                  @Update);  
  
%endfunction

function DoPostPropSetup(block)
 %% Setup Dwork
  n = block.DialogPrm(1).Data;
  r = block.DialogPrm(2).Data;
  n2 = block.DialogPrm(3).Data;
%   NN_ST = block.DialogPrm(4).Data;
%   global n r n2
%% Total Dworks needed in NN block:
  block.NumDworks = 3+n2;
  
%  (1) Dwork(1):  input(X_bar, eta) = (2 + 2*n -r);
  block.Dwork(1).Name = 'NN_input_eta'; 
  block.Dwork(1).Dimensions      = 2 + 2*n -r;
  block.Dwork(1).DatatypeID      = 0;
  block.Dwork(1).Complexity      = 'Real';
  block.Dwork(1).UsedAsDiscState = true;
  
%  (2) From Dwork(2) to Dwork(1+n2) are cols of Weighting matrix V and
%  all have length: (n1 + 1), i.e., 
%  (2 + 2*n - r)
%  Dwork(2)~ Dwork(1+n2)
  for i = 2:(1+n2)
        block.Dwork(i).Name = strcat('V_',int2str(i)); 
        block.Dwork(i).Dimensions      = 2 + 2*n -r;
        block.Dwork(i).DatatypeID      = 0;
        block.Dwork(i).Complexity      = 'Real';
        block.Dwork(i).UsedAsDiscState = true;
  end
  
 
% (3) Dwork(2+n2)is the Weighting vector W:(n2+1),
  block.Dwork(2+n2).Name = 'W'; 
  block.Dwork(2+n2).Dimensions      = n2+1;
  block.Dwork(2+n2).DatatypeID      = 0;
  block.Dwork(2+n2).Complexity      = 'Real';
  block.Dwork(2+n2).UsedAsDiscState = true;
  
% (4) Dwork(3+n2) is the output of the filter: scalar
  block.Dwork(3+n2).Name = 'z_f'; 
  block.Dwork(3+n2).Dimensions      = 1;
  block.Dwork(3+n2).DatatypeID      = 0;
  block.Dwork(3+n2).Complexity      = 'Real';
  block.Dwork(3+n2).UsedAsDiscState = true;

%endfunction

function InitConditions(block)

  %% Initialize Dwork
  n = block.DialogPrm(1).Data;
  r = block.DialogPrm(2).Data;
  n2 = block.DialogPrm(3).Data;
%   NN_ST = block.DialogPrm(4).Data;
%   global n r n2 n1

%  (1) initial Dwork(1):  input(eta) = (2 + 2*n -r);
  block.Dwork(1).Data = [1;zeros(1 + 2*n -r,1)]; % the first element is 
%   b_v that allows for the threshold theta_v to be includeed in V
  
  
%  (2) initial Dwork(2) to Dwork(1+n2)are cols of Weighting matrix V and
%      all have length: (n1 + 1), i.e., (2 + 2*n - r)
%      Dwork(2)~ Dwork(1+n2)
  for i = 2:(1+n2)
      block.Dwork(i).Data = [0;zeros(1+2*n-r,1)];% set theta_v = 0.1
  end

%  (3) initial Dwork(2+n2)is the Weighting vector W:(n2+1),
  block.Dwork(2+n2).Data = [0;zeros(n2,1)]; %the first element is 
  % theta_w
  
%  (4) Dwork(3+n2) is the output of the filter (z_f): scalar  
  block.Dwork(3+n2).Data = 0;
%endfunction

function Output(block)
  n = block.DialogPrm(1).Data; %system order
  r = block.DialogPrm(2).Data; %system relative degree
  n2 = block.DialogPrm(3).Data; %Number of neurons 
  NN_ST = block.DialogPrm(4).Data; %NN sample time
  GAMA_V = block.DialogPrm(5).Data; %Input weight V learning rate
  gama_V = block.DialogPrm(6).Data; %e-modification factor
  GAMA_W = block.DialogPrm(7).Data; %Output weight W learning rate
  gama_W = block.DialogPrm(8).Data; %e-modification factor
  alpha = block.DialogPrm(9).Data; %filter parameter
  c_f = block.DialogPrm(10).Data;% filter error combination coeficients
  c_y_tilt =  block.DialogPrm(11).Data;% tracking error coeficients
  NN_Deadzone = block.DialogPrm(12).Data; %dead zone length
  y_min = block.DialogPrm(13).Data; %minimum of y 
  y_max = block.DialogPrm(14).Data;%maximum of y 
  v_min = block.DialogPrm(15).Data; %minimum of y 
  v_max = block.DialogPrm(16).Data;%maximum of y 
% global n r n2 NN_ST n1 eta V GAMA_V gama_V Z
% global SIGMA SIGMA_DOT a W GAMA_W gama_W z_f z alpha c_f c_y_tilt
  

%% construction of input: X_bar (eta)
eta = block.Dwork(1).Data;
temp_y_window = eta(3:1+n);%store  y: n-1 window
temp_v_window = eta(3+n:1+2*n-r);%store  v: n-r-1 window
eta(2) = block.InputPort(1).Data; %set y_c(n)
eta(3) = block.InputPort(2).Data; %set y(n)
eta(3+n) = block.InputPort(3).Data; % set v(n)
% shitf y_window and v_window
eta(4:2+n) = temp_y_window;
eta(4+n:2+2*n-r) = temp_v_window;

% normalize the inputs
% delta = (d-d_min)/(d_max - d_min);
eta(2:2+n) = (eta(2:2+n)-y_min)/(y_max - y_min); %normalize y_c and y
eta(3+n:2+2*n-r) = (eta(3+n:2+2*n-r)-v_min)/(v_max - v_min); %normalize v

% update Dwork(1)
block.Dwork(1).Data = eta;

%% construction of V, W
% construc old, i.e., V(n) 
V = [];
for i = 2:(1+n2)
    V = [V,block.Dwork(i).Data]; % 
end

% construct W
W = block.Dwork(2+n2).Data;
%% constructioni of neurons:SIGMA, SIGMA_DOT
Z = V'*eta;
% SIGMA
SIGMA = [1;zeros(n2,1)];% the first element is b_w allows for the threshold
% theta_w to be included in the W
SIGMA_DOT_temp = zeros(n2,1);%construct diagnal of SIGMA_DOT
% activation potential
% global a
% a = linspace(0.1,1,n2);
a = linspace(0.5,1,n2);
for i = 2:(1+n2)
    SIGMA(i) = 1/(1+exp(-a(i-1)*Z(i-1)));
    SIGMA_DOT_temp(i-1) = a(i-1)*exp(-a(i-1)*Z(i-1))/...
        (1+exp(-a(i-1)*Z(i-1)))^2;
end
% SIGMA_DOT
SIGMA_DOT = diag(SIGMA_DOT_temp);
SIGMA_DOT = [zeros(1,n2);SIGMA_DOT];%

%%  Construct v_ad 
v_ad = W'*SIGMA;

%% construct z_f, v_bar
z_f = block.Dwork(3+n2).Data;
%combination of the tracking error, compensator states and filter states
y_tilt = block.InputPort(1).Data - block.InputPort(2).Data;

% z = c_f*z_f + c_y_tilt*y_tilt + 0.1*block.InputPort(3).Data;
z = c_f*z_f + c_y_tilt*y_tilt + 1*block.InputPort(4).Data;
% construct robustifying term: v_bar
v_bar = - 0.1*(10+ norm(V,'fro')+norm(W,'fro'))*sign(z)*abs(y_tilt);
% v_bar = -(0.1+0.1*(norm(V,'fro')+norm(W,'fro')+1))*z;
%% update z_f and Dwork(3+n2)
% using filter: T(s) = 1/(s+alpha)
% z_f = (v_ad - z_f*(alpha/2-1/NN_ST))/(1/NN_ST+alpha/2);

% using filter: T(s) = 1/(alpha*s+1)
if abs(y_tilt) > NN_Deadzone;
    z_f = (v_ad - z_f*(0.5-alpha/NN_ST))/(alpha/NN_ST+0.5);
end

% update Dwork(3+n2)
block.Dwork(3+n2).Data = z_f;
%% update V and W
% W = (-GAMA_W*(SIGMA-SIGMA_DOT*V_temp'*eta)*z - W*(GAMA_W*gama_W*abs(z)/2-1/NN_ST))/...
%     (1/NN_ST + GAMA_W*gama_W*abs(z)/2);
if (abs(y_tilt) > NN_Deadzone) ;%&& (abs(v_ad) < v_max);
    W_temp = W;
    W = (-GAMA_W*(SIGMA-SIGMA_DOT*V'*eta)*z - W*(GAMA_W*gama_W*abs(z)/2-1/NN_ST))/...
        (1/NN_ST + GAMA_W*gama_W*abs(z)/2);
    V_temp = V;%V(n)
    V = (-GAMA_V*eta*z*W'*SIGMA_DOT - V*(GAMA_V*gama_V*abs(z)/2-1/NN_ST))/...
        (1/NN_ST + GAMA_V*gama_V*abs(z)/2);
end

% update Dwork(2) to Dwork(1+n2) from V
for i = 2:(1+n2)
    block.Dwork(i).Data = V(:,i-1);
end
% update Dwork(2+n2) from W
block.Dwork(2+n2).Data = W;

%% output
% (1): v_ad --- scalar --- adaptive control
block.OutputPort(1).Data = v_ad;
% output(2): v_bar--- scalar --- robustifying term
block.OutputPort(2).Data = v_bar;
% output(3): V --- (n1 +1)x(n2) --- NN input weighting matrix
block.OutputPort(3).Data = V;
% output(4): W --- (n2 +1)x(1) --- NN output weighting vector
block.OutputPort(4).Data = W;

%endfunction


