clear all
clc
%%NOTE: models from linearization are in continous time (no dt is needed)
display('Yak54-40')
KT=input ('trim speed [knots]: ');

%aircraft data
[S,Cbar,b,...
 weight,g,IxxB,IyyB,IzzB,IxzB,...
 CL1,CD1,Cm1,...
 CD0_bar,...
 CD0,CDu,CDa,CDq,CDadot,CDde,...
 Cyb,Cyp,Cyr,Cyda,Cydr,...
 CL0,CLa,CLq,CLadot,CLu,CLde,...
 Clb,Clp,Clr,Clda,Cldr,...
 Cm0,Cma,Cmq,Cmadot,Cmu,Cmde,...
 Cnb,Cnp,Cnr,Cnda,Cndr,...
 PA0,...
 xT2,xT1,xT0,...
 Athrottle,Bthrottle,Cthrottle,Dthrottle,...
 Aelevator,Belevator,Celevator,Delevator,...
 Aaileron,Baileron,Caileron,Daileron,...
 Arudder,Brudder,Crudder,Drudder]...
 =yak40_data(KT);

%trim controls (for 70 knots)
Trim_cmd_Throttle=0.20322;         %[-]
Trim_cmd_Elevator=-1.3828*pi/180;  %[rad]
Trim_cmd_Aileron=0;                %[rad]
Trim_cmd_Rudder=0;                 %[rad]

%%%%%%This should not be here, but is the only way to get the results,
%%%%%%at the end, the true trim values are the ones get from yak40_trim file
%%%%%%The thing is that I can not have trim values before trimming the uav
%%%%%%and this is done after designing the controller.
%This seems to be an iterative process of obtaining the states trim alpha,
%P, Q, R and U, in  a similar way to the control commands. (This is due
%the way the forces and moments are defined [in a trim condition way]):
alphatrim=1.9678*pi/180;       %[rad]
Ptrim=0;                       %[rad/sec]
Qtrim=0;                       %rad/sec]
Rtrim=0;                       %[rad/sec]
Utrim=118.1467;                %[ft/sec]

%States initial conditions
Throttle0_sim=Trim_cmd_Throttle;         %[-]
Elevator0_sim=Trim_cmd_Elevator;         %[rad]
Aileron0_sim=Trim_cmd_Aileron;           %[rad]
Rudder0_sim=Trim_cmd_Rudder;             %[rad]
VT0_sim=118.1492;                        %[ft/sec]=70 knots
alpha0_sim=1.83*pi/180;                  %[rad]
beta0_sim=0;                             %[rad]
phi0_sim=0;                              %[rad]
theta0_sim=1.83*pi/180;                  %[rad]
psi0_sim=0;                              %[rad]
P0_sim=0;                                %[rad/sec]
Q0_sim=0;                                %[rad/sec]
R0_sim=0;                                %[rad/sec]

%%%define if these should come from another file or not!!!!!
rho_sim=0.002295;                        %[slug/ft3]
n0_sim=-6000;                            %[ft]
e0_sim=1900;                             %[ft]
h0_sim=550;                              %[ft]

load ControlSurfaces
load busses