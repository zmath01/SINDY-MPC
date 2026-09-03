% Automatic flight control of F8
% System identification: SINDYc
clear all, close all, clc
figpath = '../FIGURES/F8/'; mkdir(figpath)
datapath = '../DATA/F8/'; mkdir(datapath)
addpath('../utils');

SystemModel = 'F8';
Nvar = 3;

%% Generate Data
InputSignalType = 'sine3';
ONLY_TRAINING_LENGTH = 1;
ENSEMBLE_DATA = 0;
getTrainingData
u = u';

%% SINDYc
ModelName = 'SINDYc';
polyorder = 3;
usesine = 0;
lambda_vec = [.0001,.01,0.01];
eps = 0;
trainSINDYc

%% Compare with true model parameters
Xi0 = zeros(size(Xi));
Xi0([2,4,5,6,8,10,16,18,19,25,35],1) = [-0.877,1,-0.215,0.47,-0.088,-0.019,3.846,-1,0.28,0.47,0.63];
Xi0([4],2) = 1;
Xi0([2,4,5,6,16,19,25,35],3) = [-4.208,-0.396,-20.967,-0.47,-3.564,6.265,46,61.4];
Xi-Xi0

%% Prediction over training phase
if any(strcmp(InputSignalType,{'sine2', 'chirp','prbs', 'sphs'}))
    [tSINDYc,xSINDYc] = ode45(@(t,x)sparseGalerkinControl(t,x,forcing(x,t),Xi(:,1:Nvar),polyorder,usesine),tspan,x0,options);
else
    p.ahat = Xi(:,1:Nvar);
    p.polyorder = polyorder;
    p.usesine = usesine;
    p.dt = dt;
    [Ntrain,Ns] = size(x);
    xSINDYc = zeros(Ns,Ntrain);
    xSINDYc(:,1) = x0';
    for ct = 1:Ntrain-1
        xSINDYc(:,ct+1) = rk4u(@sparseGalerkinControl_Discrete,xSINDYc(:,ct),u(ct),dt,1,[],p);
    end
    xSINDYc = xSINDYc';
end

%% Show validation
clear ph
figure, box on,
ccolors = get(gca,'colororder');
ccolors_valid = [ccolors(1,:)-[0 0.2 0.2]; ccolors(2,:)-[0.1 0.2 0.09]; ccolors(3,:)-[0.1 0.2 0.09]];
for i = 1:Nvar
    ph(i) = plot(tspan,x(:,i),'-','Color',ccolors(i,:),'LineWidth',1); hold on
end
for i = 1:Nvar
    ph(Nvar+i) = plot(tspan,xSINDYc(:,i),'--','Color',ccolors_valid(i,:),'LineWidth',2);
end
ylim([-0.8 0.9])
xlabel('Time'); ylabel('xi')
legend(ph([1,4]),'True',ModelName)
legend(ph([1,2,3]),'angle of attack','pitch angle','pitch rate')
set(gca,'LineWidth',1,'FontSize',14)
set(gcf,'Position',[100 100 300 200]); set(gcf,'PaperPositionMode','auto')
print('-depsc2','-loose','-cmyk',[figpath,'EX_',SystemModel,'_SI_',ModelName,'_',InputSignalType,'.eps']);

%% Held-out prediction
% Keep the state allocation one element longer than the number of intervals.
tspanV = 100:dt:200;
xA = xv;
tA = tv;
if any(strcmp(InputSignalType,{'sine2','chirp','prbs','sphs'}))
    [tB,xB] = ode45(@(t,x)sparseGalerkinControl(t,x,forcing(x,t),Xi(:,1:Nvar),polyorder,usesine),tspanV,x(end,:),options);
    xB = xB(2:end,:);
    tB = tB(2:end);
else
    [Nval,Ns] = size(xA);
    xB = zeros(Ns,Nval+1);
    xB(:,1) = x(end,:)';
    for ct = 1:Nval
        xB(:,ct+1) = rk4u(@sparseGalerkinControl_Discrete,xB(:,ct),uv(ct),dt,1,[],p);
    end
    xB = xB';
    tB = tspanV(1:size(xB,1));
end

%% Show training and prediction
u = u';
VIZ_SI_Validation

%% Save Data
Model.name = 'SINDYc';
Model.polyorder = polyorder;
Model.usesine = usesine;
Model.Xi = Xi;
Model.dt = dt;
save(fullfile(datapath,['EX_',SystemModel,'_SI_',ModelName,'_',InputSignalType,'.mat']),'Model')
