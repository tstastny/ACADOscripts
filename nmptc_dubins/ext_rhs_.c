  t3 = cos(in04);
  t9 = cos(in05);
  t10 = in10*t3*t9;
  t2 = in20+t10;
  t7 = sin(in05);
  t8 = in10*t3*t7;
  t4 = in21+t8;
  t5 = sin(in04);
  t11 = in10*t5;
  t6 = in22-t11;
  A0[0][0] = t6*1.0/sqrt(t2*t2+t4*t4+t6*t6);
  A0[1][0] = Td_e;
  A0[2][0] = Td_n;
  A0[3][0] = t4;
  A0[4][0] = t2;
  A0[5][0] = atan2_01-atan2_02;
  A0[6][0] = t2;
  A0[7][0] = t4;
  A0[8][0] = t6;
  A0[9][0] = in06;
  A0[10][0] = in07;
  A0[11][0] = (tan(in03)*(9.81E2/1.0E2))/in10;
  A0[12][0] = -in25*(in06+in23*(in03-in08));
  A0[13][0] = -in26*(in07+in24*(in04-in09));