#include <stdio.h>
#include <stdlib.h>

int main(int argc, char** argv) {
float one, two;
FILE * fp = fopen("data.txt", "w");
one = 13.45624;
two = 1.3221;
fprintf(fp, "%f %f\n", one, two); 
one = 12.4323;
two = 2.55723;
fprintf(fp, "%f %f\n", one, two); 
one = 9.234572;
two = 4.3257;
fprintf(fp, "%f %f\n", one, two); 
one = 6.457614;
two = 5;
fprintf(fp, "%f %f\n", one, two);

fclose(fp); 

FILE * fp1 = fopen("data.txt", "r");
fscanf(fp1, "%f %f\n", &one, &two);
printf("%f %f\n", one, two);
fscanf(fp1, "%f %f\n", &one, &two);
printf("%f %f\n", one, two);
fscanf(fp1, "%f %f\n", &one, &two);
printf("%f %f\n", one, two);
fscanf(fp1, "%f %f\n", &one, &two);
printf("%f %f\n", one, two);

fclose(fp1);
}
