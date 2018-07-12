rel_n = pos_n - terrain_data[IDX_CENTER_N * LEN_IDX_E + IDX_CENTER_E]; 
rel_n = rel_n + ((rel_n<0.0) ? -1.0 : 1.0) * DIS_OVER_2; 
int delta_idx_n = rel_n * ONE_OVER_DIS; 
int idx_n = IDX_CENTER_N + delta_idx_n; 
if (idx_n >= LEN_IDX_N) { 
idx_n = LEN_IDX_N-1; 
} 
else if (idx_n < 0) { 
idx_n = 0; 
} 
 
float rel_e = pos_e - terrain_data[IDX_CENTER_N * LEN_IDX_E + IDX_CENTER_E]; 
rel_e = rel_e + ((rel_e<0.0f) ? -1.0 : 1.0) * DIS_OVER_2; 
int delta_idx_e = rel_e * ONE_OVER_DIS; 
int idx_e = IDX_CENTER_E + delta_idx_e; 
if (idx_e >= LEN_IDX_E) { 
idx_e = LEN_IDX_E-1; 
} 
else if (idx_e < 0) { 
idx_e = 0; 
} 
 
return terrain_data[idx_n * LEN_IDX_E + idx_e]; 
