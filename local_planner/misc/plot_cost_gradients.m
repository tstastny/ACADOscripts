% / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /
% PLOT COST GRADIENTS / / / / / / / / / / / / / / / / / / / / / / / / / / /

% choose an objective and state id (see defined indices below)
str_y = 'VN';
str_x = 'XI';

% DONT TOUCH. (unless the states / outputs change)  - - - - - - - - - - - -
IDX_X.N = 1;
IDX_X.E = 2;
IDX_X.D = 3;
IDX_X.V = 4;
IDX_X.GAMMA = 5;
IDX_X.XI = 6;
IDX_X.PHI = 7;
IDX_X.THETA = 8;
IDX_X.NPROP = 9;

IDX_Y.VN = 1;
IDX_Y.VE = 2;
IDX_Y.VD = 3;
IDX_Y.V = 4;
IDX_Y.PHI = 5;
IDX_Y.THETA = 6;
IDX_Y.SOFT_AOA = 7;
IDX_Y.SOFT_H = 8;
IDX_Y.SOFT_R = 9;
IDX_Y.U_T = 10;
IDX_Y.PHI_REF = 11;
IDX_Y.THETA_REF = 12;
% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

% calculate cost gradient
DJ = 2 .* (rec.y(:,:,IDX_Y.(str_y)) - rec.yref(:,:,IDX_Y.(str_y))) .* ...
    rec.dydx(:,:,(IDX_Y.(str_y)-1)*n_X+IDX_X.(str_x));

% name
str_grad = ['dJ(',str_y,')/d(',str_x,')'];

% plot
figure('color','w','name',str_grad);
hold on; grid on; box on;

mesh(rec.time_nmpc, 1:N, DJ);

xlabel('Time [s]');
ylabel('Node');
zlabel(str_grad);
