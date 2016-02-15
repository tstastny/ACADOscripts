
#include <stdio.h>
#include <string.h>

void rhs_eval( double *in, double *f ) {
    
    f[0] = 1.0*in[0] + 2.0*in[1] + 3.0*in[2] + 4.0*in[3] + 5.0*in[4];
    f[1] = 6.0*in[0] + 7.0*in[1] + 8.0*in[2] + 9.0*in[3] + 10.0*in[4];
    f[2] = 11.0*in[0] + 12.0*in[1] + 13.0*in[2] + 14.0*in[3] + 15.0*in[4];
    
}

int main() {
    
    const double in[5] = {1.0,1.0,1.0,1.0,1.0};
    
    const int NX = 3;
    const int NU = 2;

    double out[NX*(NX+NU)];

    double f_Delta_m[NX];
    double f_Delta_p[NX];
    double in_Delta[NX+NU];
    memcpy(in_Delta, in, sizeof(in));
    const double Delta = 0.00001;
    const double Delta2 = 2.0 * Delta;
    
    int i;
    int j;
    for (i = 0; i < (NX+NU); i=i+1) {
        
        in_Delta[i] = in[i] - Delta;
        rhs_eval( in_Delta, f_Delta_m );
        in_Delta[i] = in[i] + Delta;
        rhs_eval( in_Delta, f_Delta_p );
        in_Delta[i] = in[i];

        for (j = 0; j < NX; j=j+1) {
            out[j*(NX+NU)+i] = (f_Delta_p[j] - f_Delta_m[j]) / Delta2;
        }

    }

    for (i = 0; i < NX*(NX+NU); i=i+1) {
        printf("%lf \n",out[i]);
    }
    
    return 0;
}