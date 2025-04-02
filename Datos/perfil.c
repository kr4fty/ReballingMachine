/* Curva perfil de temperatura *************************************************
*
* Temp^
*     |
*     |
*TMax_|                                   __.:.__
*     |                                 /         \
*     |                                /           \
*     |            ___________________/             \
*     |           /                                  \
*     |          /                                    \
*     |         /                                      \
*     |        /                                        \
*Tamb_| ______/                                          \_____________
*     |
*     '------------------------------------------------------------------->
*             |   |                   |  |   |    |      |                  Time
*     0      t0  t1                  t2 t3  tmax  t4     t5
*
*
*   Tiempo  Temperaturas    Duración                    Descripción
*   ======  ============    ========                    ===========
*   t0-t1   1ºC/seg.        Hasta los 150ºC             Precalentamiento
*   t1-t2   0.5ºC/seg.      hasta los 180º, 60 seg Max  Activación del FLUX
*   t2-t3   180º a Tmax     45seg MAX                   Reflow / Extracción
*   t4-t5   -1ºC/seg.       Tmax a Tamb                 Enfriamiento
*
*
* Tmax va a variar de acurdo a la placa (tipo de disipador, espesor, tipo de IC,
* tipo de estaño, etc.). Por lo general integrados con plomo, hasta 183ºC. Para
* ICs sin plomo, hasta unos 218-230ºC
*
*******************************************************************************/


#include <stdio.h>
#include <string.h>

void main(void)
{
    int tiempo;
    char str[50];
    FILE *fp;

    fp = fopen("perfil.csv", "w");
    if(fp == NULL){
        fprintf(stderr, "ERROR");
        return;
    }

    float perfil_temp[]={0, 130, 150, 180, 180, 0};
    float perfil_time[]={0.0, 120.0, 60.0, 15.0, 15.0, 150.0};
    int t1, t2, t3, t4, t5;
    float rampt0t1, rampt1t2, rampt2t3, rampt3t4, rampt4t5;
    float perfilRamp;

    perfil_temp[0] = 26;
    perfil_time[0] = 0;

    t1=perfil_time[0]+perfil_time[1];  // 120
    t2=t1+perfil_time[2];              // 120+60=180
    t3=t2+perfil_time[3];              // 120+60+15=195
    t4=t3+perfil_time[4];              // 120+60+15+15=210
    t5=t4+perfil_time[5];              // 120+60+15+15+150=360

    rampt0t1=((perfil_temp[1]-perfil_temp[0])/perfil_time[1]);
    rampt1t2=((perfil_temp[2]-perfil_temp[1])/perfil_time[2]);
    rampt2t3=((perfil_temp[3]-perfil_temp[2])/perfil_time[3]);
    rampt3t4=((perfil_temp[4]-perfil_temp[3])/perfil_time[4]);
    rampt4t5=((perfil_temp[5]-perfil_temp[4])/perfil_time[5]);

    for(tiempo =0; tiempo < 400; tiempo++){
         if     (tiempo < t1)
            perfilRamp = rampt0t1*tiempo+perfil_temp[0];
        else if(tiempo < t2)
            perfilRamp = rampt1t2*(tiempo-t1)+perfil_temp[1];
        else if(tiempo < t3)
            perfilRamp = rampt2t3*(tiempo-t2)+perfil_temp[2];
        else if(tiempo < t4)
            perfilRamp = rampt3t4*(tiempo-t3)+perfil_temp[3];
        else if(tiempo < t5)
            perfilRamp = rampt4t5*(tiempo-t4)+perfil_temp[4];
        else
            perfilRamp = 0;

        sprintf(str,"%d, %.2f\n", tiempo,perfilRamp);

        fwrite(str , 1 , strlen(str) , fp );
    }

    fclose(fp);
}
