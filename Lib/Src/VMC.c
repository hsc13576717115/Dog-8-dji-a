#include "VMC.h"
#include "math.h"

angle_leg Leg;
coordinate coo;
polarCoordinates_C C_polar;

void get_angle(polarCoordinates_C *c, float F, float Fp)
{
    float arr[2][2];

    float new_F;
    float new_Fp;

    Leg.zeta1 = acos((c->L * c->L + CLF_LNGTH * CLF_LNGTH - THGH_LNGTH * THGH_LNGTH) / (2 * CLF_LNGTH * c->L));
    Leg.zeta3 = c->zeta - Leg.zeta1;

    coo.x_B = THGH_LNGTH * cos(Leg.zeta1);
    coo.y_B = THGH_LNGTH * sin(Leg.zeta1);
    coo.x_C = c->L * cos(c->zeta);
    coo.y_C = c->L * sin(c->zeta);
    coo.x_D = THGH_LNGTH * cos(Leg.zeta3);
    coo.y_D = THGH_LNGTH * sin(Leg.zeta3);

    Leg.zeta2 = asin(coo.y_C - coo.y_B) / CLF_LNGTH;
    Leg.zeta4 = asin(coo.y_C - coo.y_D) / CLF_LNGTH;

    arr[0][0] = (THGH_LNGTH * sin(c->zeta - Leg.zeta3) * sin(Leg.zeta1 - Leg.zeta2) / sin(Leg.zeta3 - Leg.zeta2));
    arr[0][1] = (THGH_LNGTH * sin(c->zeta - Leg.zeta3) * sin(Leg.zeta1 - Leg.zeta2) / c->L * sin(Leg.zeta3 - Leg.zeta2));
    arr[1][0] = (THGH_LNGTH * sin(c->zeta - Leg.zeta2) * sin(Leg.zeta3 - Leg.zeta4) / sin(Leg.zeta3 - Leg.zeta2));
    arr[1][1] = (THGH_LNGTH * sin(c->zeta - Leg.zeta2) * sin(Leg.zeta3 - Leg.zeta4) / c->L * sin(Leg.zeta3 - Leg.zeta2));

    F = new_F = arr[0][0] * F + arr[0][1] * Fp;
    Fp = new_Fp = arr[1][0] * F + arr[1][1] * Fp;
}
