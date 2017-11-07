% /////////////////////////////////////////////////////////////////////////
% dubins paths list ///////////////////////////////////////////////////////

% pparam1 | type=0  type=1  type=2
% pparam2 | b_n     c_n     c_n
% pparam3 | b_e     c_e     c_e
% pparam4 | b_d     c_d     c_d
% pparam5 | n/a     +/-R    +/-R
% pparam6 | Chi     Chi     n/a
% pparam7 | Gam     Gam     n/a

% /////////////////////////////////////////////////////////////////////////

% LOITER UNLIM

% paths(1).pparam1 = 2;
% paths(1).pparam2 = 0;
% paths(1).pparam3 = 0;
% paths(1).pparam4 = -80;
% paths(1).pparam5 = 50;
% paths(1).pparam6 = 0;
% paths(1).pparam7 = 0;
% 
% paths(2).pparam1 = 2;
% paths(2).pparam2 = 0;
% paths(2).pparam3 = 0;
% paths(2).pparam4 = -80;
% paths(2).pparam5 = 50;
% paths(2).pparam6 = 0;
% paths(2).pparam7 = 0;

% SPIRAL

% paths(1).pparam1 = 1;
% paths(1).pparam2 = 0;
% paths(1).pparam3 = 0;
% paths(1).pparam4 = -30;
% paths(1).pparam5 = 50;
% paths(1).pparam6 = pi/2;
% paths(1).pparam7 = deg2rad(8);
% 
% % paths(2).pparam1 = 1;
% % paths(2).pparam2 = 0;
% % paths(2).pparam3 = 0;
% % paths(2).pparam4 = -50;
% % paths(2).pparam5 = 35;
% % paths(2).pparam6 = pi/2;
% % paths(2).pparam7 = deg2rad(8);
% 
% paths(2).pparam1 = 2;
% paths(2).pparam2 = 0;
% paths(2).pparam3 = 0;
% paths(2).pparam4 = -30;
% paths(2).pparam5 = 50;
% paths(2).pparam6 = 0;
% paths(2).pparam7 = 0;

% LINE

% paths(1).pparam1 = 0;
% paths(1).pparam2 = 500;
% paths(1).pparam3 = 288.6751;
% paths(1).pparam4 = -35.1080;
% paths(1).pparam5 = 0;
% paths(1).pparam6 = deg2rad(30);
% paths(1).pparam7 = deg2rad(3);
% 
% paths(2).pparam1 = 0;
% paths(2).pparam2 = 500;
% paths(2).pparam3 = 288.6751;
% paths(2).pparam4 = -35.1080;
% paths(2).pparam5 = 0;
% paths(2).pparam6 = deg2rad(30);
% paths(2).pparam7 = deg2rad(3);

% paths(1).pparam1 = 0;
% paths(1).pparam2 = cosd(30)*200;
% paths(1).pparam3 = sind(30)*200;
% paths(1).pparam4 = -28.1082;
% paths(1).pparam5 = 0;
% paths(1).pparam6 = deg2rad(30);
% paths(1).pparam7 = deg2rad(8);
% 
% paths(2).pparam1 = 0;
% paths(2).pparam2 = cosd(30)*200+cosd(90)*500;
% paths(2).pparam3 = sind(30)*200+sind(90)*500;
% paths(2).pparam4 = -28.1082+0*34.9634;
% paths(2).pparam5 = 0;
% paths(2).pparam6 = deg2rad(90);
% paths(2).pparam7 = deg2rad(0);


paths(1).pparam1 = 0;
paths(1).pparam2 = cosd(90)*200;
paths(1).pparam3 = sind(90)*200;
paths(1).pparam4 = 0;
paths(1).pparam5 = 0;
paths(1).pparam6 = deg2rad(90);
paths(1).pparam7 = deg2rad(0);

paths(2).pparam1 = 0;
paths(2).pparam2 = cosd(90)*200+cosd(180)*200;
paths(2).pparam3 = sind(90)*200+sind(180)*200;
paths(2).pparam4 = 0;
paths(2).pparam5 = 0;
paths(2).pparam6 = deg2rad(180);
paths(2).pparam7 = deg2rad(0);

