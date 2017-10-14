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

paths(1).pparam1 = 2;
paths(1).pparam2 = 0;
paths(1).pparam3 = 0;
paths(1).pparam4 = -80;
paths(1).pparam5 = 35;
paths(1).pparam6 = 0;
paths(1).pparam7 = 0;

paths(2).pparam1 = 2;
paths(2).pparam2 = 0;
paths(2).pparam3 = 0;
paths(2).pparam4 = -80;
paths(2).pparam5 = 35;
paths(2).pparam6 = 0;
paths(2).pparam7 = 0;

% SPIRAL UP

% paths(1).pparam1 = 1;
% paths(1).pparam2 = 0;
% paths(1).pparam3 = 0;
% paths(1).pparam4 = -50;
% paths(1).pparam5 = 35;
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
% paths(2).pparam4 = -50;
% paths(2).pparam5 = 35;
% paths(2).pparam6 = 0;
% paths(2).pparam7 = 0;

% LINE

% paths(1).pparam1 = 0;
% paths(1).pparam2 = 600;
% paths(1).pparam3 = 300;
% paths(1).pparam4 = 10;
% paths(1).pparam5 = 0;
% paths(1).pparam6 = deg2rad(45);
% paths(1).pparam7 = deg2rad(0);
% 
% paths(2).pparam1 = 0;
% paths(2).pparam2 = 600+600*cos(deg2rad(135));
% paths(2).pparam3 = 300+600*sin(deg2rad(135));
% paths(2).pparam4 = -41.956087166106251+10;
% paths(2).pparam5 = 0;
% paths(2).pparam6 = deg2rad(135);
% paths(2).pparam7 = deg2rad(4);

