#include "stanford_type.h"
#include "math.h"

#define M_PI (3.14159265f)

#define L(x) (data->leg_length[x - 1])
#define x (data->position[0])
#define y (data->position[1])
#define Alpha(n) (data->angle[n - 1])

uint8_t flag_role=0;

//none
void Stanford_Type_Forward_Kinematics(Stanford_Type_Data_t *data)
{
    float xa, ya, xc, yc, len_ac, A, B, C, theta1;

    xa = L(1) * cosf(Alpha(1));
    ya = L(1) * sinf(Alpha(1));
    xc = L(5) + L(4) * cosf(Alpha(2));
    yc = L(4) * sinf(Alpha(2));
    len_ac = sqrtf((xc - xa) * (xc - xa) + (yc - ya) * (yc - ya));

    A = 2 * L(2) * (xc - xa);
    B = 2 * L(2) * (yc - ya);
    C = L(2) * L(2) + len_ac * len_ac - L(3) * L(3);
    theta1 = 2 * atanf((B + sqrtf(A * A + B * B - C * C)) / (A + C));

    x = xa + (L(2) + L(6)) * cosf(theta1);
    y = ya + (L(2) + L(6)) * sinf(theta1);
}

// FIXME:还没完工，网上提供的公式好像有问题  none
void Stanford_Type_Inverse_Kinematics(Stanford_Type_Data_t *data)
{
    float a, b, c, alpha1[2], xb[2], yb[2], d[2], e[2], f[2], alpha2[4];

    a = 2 * x * L(1);
    b = 2 * y * L(1);
    c = x * x + y * y + L(1) * L(1) - (L(2) + L(6)) * (L(2) + L(6));

    alpha1[0] = 2 * atanf((b + sqrtf(a * a + b * b - c * c)) / (a + c));
    alpha1[1] = 2 * atanf((b - sqrtf(a * a + b * b - c * c)) / (a + c));

    xb[0] = x - L(6) * ((x - L(1) * cosf(alpha1[0])) / (L(2) + L(6)));
    xb[1] = x - L(6) * ((x - L(1) * cosf(alpha1[1])) / (L(2) + L(6)));
    yb[0] = y - L(6) * ((x - L(1) * sinf(alpha1[0])) / (L(2) + L(6)));
    yb[1] = y - L(6) * ((x - L(1) * sinf(alpha1[1])) / (L(2) + L(6)));

    d[0] = 2 * L(4) * (xb[0] - L(5));
    d[1] = 2 * L(4) * (xb[1] - L(5));

    e[0] = 2 * L(4) * yb[0];
    e[1] = 2 * L(4) * yb[1];

    f[0] = (xb[0] - L(5)) * (xb[0] - L(5)) + L(4) * L(4) + yb[0] * yb[0] - L(3) * L(3);
    f[1] = (xb[1] - L(5)) * (xb[1] - L(5)) + L(4) * L(4) + yb[1] * yb[1] - L(3) * L(3);

    alpha2[0] = (2 * atan(e[0] + sqrt(d[0] * d[0] + e[0] * e[0] - f[0] * f[0])) / (d[0] + f[0]));
    alpha2[1] = (2 * atan(e[0] - sqrt(d[0] * d[0] + e[0] * e[0] - f[0] * f[0])) / (d[0] + f[0]));
    alpha2[2] = (2 * atan(e[1] + sqrt(d[1] * d[1] + e[1] * e[1] - f[1] * f[1])) / (d[1] + f[1]));
    alpha2[3] = (2 * atan(e[1] - sqrt(d[1] * d[1] + e[1] * e[1] - f[1] * f[1])) / (d[1] + f[1]));
    //...to be continue
}

//none
void Stanford_Type_Lite_Forward_Kinematics(Stanford_Type_Lite_Data_t *data)
{
    float a1, b1, L;
    a1 = L(1) * cosf((Alpha(1) - Alpha(2)) / 2);
    b1 = L(1) * sinf((Alpha(1) - Alpha(2)) / 2);
    L = sqrtf(L(2) * L(2) - b1 * b1) + a1;
    x = L * sinf((Alpha(1) + Alpha(2)) / 2);
    y = L * cosf((Alpha(1) + Alpha(2)) / 2);
}

//
void Stanford_Type_Lite_Inverse_Kinematics(Stanford_Type_Lite_Data_t *data, int leg)
{
    float l, psi, phi;
    l = sqrtf(x * x + y * y);
		
		if (flag_role) psi = asinf(x / l)*2;
		else if (flag_role==0) psi = asinf(x / l);
      //[-pi/2 ,pi/2]
		//psi = atan2f(y,x);
    phi = acosf((l * l + L(1) * L(1) - L(2) * L(2)) / (2 * l * L(1))); //[0,pi] acos asin返回的都是弧度值

    if (leg == 0)
    {
        Alpha(1) = psi - phi;
        Alpha(2) = psi + phi;
    }
    else
    {
        Alpha(1) = -(psi + phi);
        Alpha(2) = -(psi - phi);
    }
//		Alpha(1) = fmodf(Alpha(1) + M_PI,  M_PI) - M_PI;
//    Alpha(2) = fmodf(Alpha(2) + M_PI,  M_PI) - M_PI;
    //    Alpha(1) = psi + phi;
    //    Alpha(2) = psi - phi;
}

//jump
void Polar_toCartesian(Polar_Coord_Data_t *polar, Cart_Coord_Data_t *cart)
{
    cart->cy = polar->radius * cosf(polar->theta * 3.14159f / 180.0f);
    cart->cx = polar->radius * sinf(polar->theta * 3.14159f / 180.0f);
    return;
}
