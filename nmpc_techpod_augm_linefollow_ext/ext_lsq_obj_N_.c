  t2 = in33-in36;
  t3 = t2*t2;
  t4 = in34-in37;
  t5 = in35-in38;
  t6 = t4*t4;
  t7 = t5*t5;
  t8 = t3+t6+t7;
  t9 = 1.0/t8;
  t10 = 1.0/sqrt(t8);
  t11 = cos(in07);
  t13 = cos(in06);
  t14 = sin(in06);
  t15 = sin(in07);
  t50 = in00*t15;
  t51 = in02*t11*t13;
  t52 = in01*t11*t14;
  t12 = in45-t50+t51+t52;
  t16 = sin(in08);
  t17 = cos(in08);
  t30 = t13*t16;
  t31 = t14*t15*t17;
  t32 = t30-t31;
  t33 = in01*t32;
  t34 = t14*t16;
  t35 = t13*t15*t17;
  t36 = t34+t35;
  t37 = in02*t36;
  t38 = in00*t11*t17;
  t18 = in43-t33+t37+t38;
  t40 = t13*t17;
  t41 = t14*t15*t16;
  t42 = t40+t41;
  t43 = in01*t42;
  t44 = t14*t17;
  t45 = t13*t15*t16;
  t46 = t44-t45;
  t47 = in02*t46;
  t48 = in00*t11*t16;
  t19 = in44+t43-t47+t48;
  t20 = t3+t6;
  t21 = 1.0/sqrt(t20);
  t22 = in09-in33;
  t23 = in10-in34;
  t25 = t2*t21*t22;
  t26 = t4*t21*t23;
  t27 = t25+t26;
  t75 = t2*t21*t27;
  t24 = -in09+in33+t75;
  t70 = t4*t21*t27;
  t28 = -in10+in34+t70;
  t29 = 1.0/3.141592653589793;
  t39 = t18*t18;
  t49 = t19*t19;
  t53 = t39+t49;
  t54 = sqrt(t53);
  t55 = t2*t10*t22;
  t56 = t4*t10*t23;
  t57 = in11-in35;
  t58 = t5*t10*t57;
  t59 = t55+t56+t58;
  t60 = t5*t10*t59;
  t61 = t3*t9;
  t62 = t6*t9;
  t63 = t61+t62;
  t64 = L1_lon*L1_lon;
  t65 = xtrackerr_lon*xtrackerr_lon;
  t66 = t64-t65;
  t67 = sqrt(t66);
  t68 = 1.0/sqrt(t63);
  t69 = -in11+in35+t60;
  t71 = L1_lat*L1_lat;
  t72 = normdd_pp_lat*normdd_pp_lat;
  t73 = t71-t72;
  t74 = sqrt(t73);
  t76 = t12*t12;
  t77 = t39+t49+t76;
  t78 = sqrt(t77);
  t79 = in00*in00;
  t80 = in01*in01;
  t81 = in02*in02;
  t82 = t79+t80+t81;
  A0[0][0] = t68*t69;
  A0[1][0] = in25*in26*t29*t78;
  A0[2][0] = sqrt(t24*t24+t28*t28);
  A0[3][0] = in27*in28*t29*t54;
  A0[4][0] = t12;
  A0[5][0] = t54;
  A0[6][0] = -in11+in35+t60-t5*t10*t67;
  A0[7][0] = sqrt(t63)*t67+t5*t10*t68*t69;
  A0[8][0] = atan2_01-atan2_02;
  A0[9][0] = -in10+in34+t70-t4*t21*t74;
  A0[10][0] = -in09+in33+t75-t2*t21*t74;
  A0[11][0] = t19;
  A0[12][0] = t18;
  A0[13][0] = atan2_03-atan2_04;
  A0[14][0] = in14+in32-sqrt(t82);
  A0[15][0] = -in07+atan((3.141592653589793*t78*(in12+etalon*(in26*in26)*4.0)*(1.0E2/9.81E2))/(in25*in26));
  A0[16][0] = -in06+atan((3.141592653589793*t54*(in13+etalat*(in28*in28)*4.0)*(1.0E2/9.81E2))/(in27*in28));
  A0[17][0] = asin(in01*1.0/sqrt(t82));
  A0[18][0] = in03;
  A0[19][0] = in04;
  A0[20][0] = in05;