
f_lines = cell(0);

N = 10;
NX = 20;
NU = 5;
NY = 12;
NYN = 7;
NOD = 21;

size_x = NX*(N+1);
input_x = reshape((input.x)', size_x, 1);
for i = 1:size_x
    f_lines{i} = ['acadoVariables.x[ ',int2str(i-1),' ] = ',num2str(input_x(i)),';'];
end
idx_f = size_x + 1;
f_lines{idx_f} = '';

size_u = NU*N;
input_u = reshape((input.u)', size_u, 1);
for i = 1:size_u
    f_lines{idx_f+i} = ['acadoVariables.u[ ',int2str(i-1),' ] = ',num2str(input_u(i)),';'];
end
idx_f = idx_f + size_u + 1;
f_lines{idx_f} = '';

size_y = NY*N;
input_y = reshape((input.y)', size_y, 1);
for i = 1:size_y
    f_lines{idx_f+i} = ['acadoVariables.y[ ',int2str(i-1),' ] = ',num2str(input_y(i)),';'];
end
idx_f = idx_f + size_y + 1;
f_lines{idx_f} = '';

size_yN = NYN;
input_yN = input.yN;
for i = 1:size_yN
    f_lines{idx_f+i} = ['acadoVariables.yN[ ',int2str(i-1),' ] = ',num2str(input_yN(i)),';'];
end
idx_f = idx_f + size_yN + 1;
f_lines{idx_f} = '';

size_od = NOD*(N+1);
input_od = reshape((input.od)', size_od, 1);
for i = 1:size_od
    f_lines{idx_f+i} = ['acadoVariables.od[ ',int2str(i-1),' ] = ',num2str(input_od(i)),';'];
end
idx_f = idx_f + size_od + 1;
f_lines{idx_f} = '';

size_x0 = NX;
input_x0 = input.x0;
for i = 1:size_x0
    f_lines{idx_f+i} = ['acadoVariables.x0[ ',int2str(i-1),' ] = ',num2str(input_x0(i)),';'];
end
idx_f = idx_f + size_x0 + 1;
f_lines{idx_f} = '';

size_W = NY*NY;
input_W = reshape((input.W)', size_W, 1);
for i = 1:size_W
    f_lines{idx_f+i} = ['acadoVariables.W[ ',int2str(i-1),' ] = ',num2str(input_W(i)),';'];
end
idx_f = idx_f + size_W + 1;
f_lines{idx_f} = '';

size_WN = NYN*NYN;
input_WN = reshape((input.WN)', size_WN, 1);
for i = 1:size_WN
    f_lines{idx_f+i} = ['acadoVariables.WN[ ',int2str(i-1),' ] = ',num2str(input_WN(i)),';'];
end
idx_f = idx_f + size_WN + 1;
f_lines{idx_f} = '';

fid = fopen('test_inputs.c','w');
for k = 1:length(f_lines)
    fprintf(fid,[char(f_lines{k}),'\n']);
end
fclose(fid);
