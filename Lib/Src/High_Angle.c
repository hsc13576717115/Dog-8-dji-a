#include "High_Angle.h"



/**
 * @brief 站立状态下改电机夹角
 * @param angle夹角大小 [60,360]
 */
void Change_Angle(float angle)
{
	if (angle<60) return;
	if (angle>360) return;
//	angle-=60;
//	angle/=2;
	angle = 200 - angle;
	angle /= 2;
	
//	Standup_LegR_Offset = (-angle) * 3.14159f / 180.0f + LegRinit_onestep + (5) * 3.14159f / 180.0f;
//	Standup_LegL_Offset = (-angle) * 3.14159f / 180.0f + LegLinit_onestep - (5) * 3.14159f / 180.0f; 
	
		Standup_LegR_Offset = (angle) * 3.14159f / 180.0f + LegRinit_twostep;// + (5) * 3.14159f / 180.0f;
		Standup_LegL_Offset = (angle) * 3.14159f / 180.0f + LegLinit_twostep;// - (5) * 3.14159f / 180.0f; 
}

/**
 * @brief 高度转角度
 * @param h输入高度[mm] [140,∞]
 * @return 弧度转角度
 */
double High_To_Angle(float h)
{
		double AB, CE, DE;//无特殊含义 草图上随便写的
    double alpha_rad;
    double sin_alpha2, cos_alpha2;

    // 使用二分法近似求解α
    double low = 0.0;
    double high = PI; // α的范围是0到180度，转换为弧度(PI可以达到360度)
    double mid;
    double tolerance = 1e-6;  // 精度

    while (high - low > tolerance)
		{
        mid = (low + high) / 2.0;
        sin_alpha2 = sin(mid);
        cos_alpha2 = cos(mid);

        // 计算 AB, CE, DE
        AB = 150 * sin_alpha2;
        CE = 150 * cos_alpha2;
        DE = sqrt(Leglength * Leglength - AB * AB);

        // 计算 h
        double calculated_h = DE - CE;

        // 根据 h 的值调整搜索区间
        if (calculated_h > h)
				{
            high = mid;
        }
				else
				{
            low = mid;
        }
    }

    // 最终的中间值 mid 即为 α/2 的值
    alpha_rad = mid;

    // 返回 α（注意要乘以2，因为我们求的是 α/2）
    return 2 * alpha_rad * 180.0 / PI; // 转换为度
}


/*横躺状态*/
/**
 * @brief 定义函数 h(α)
 * @param alpha 角度
 * @return h  高度
 */
double calculate_h(double alpha)
{
    double sin_half_alpha = sin(alpha / 2);
    double cos_half_alpha = cos(alpha / 2);
    double CE = 150 * sin_half_alpha;
    double AE = 150 * cos_half_alpha;
    double DE = sqrt(Leglength * Leglength - CE * CE);
    double AD = DE - AE;
    double cos_CBD = (15.0 / 28.0) * sin_half_alpha;
    double h = cos_CBD * AD;
    return h;
}


/**
 * @brief 定义导数函数 dh/dα(阿尔法)
 * @param alpha 角度
 * @return (h2 - h1) / delta  数值导数
 */
double calculate_derivative(double alpha)
{
    double delta = 1e-6;  // 微小变化量
    double h1 = calculate_h(alpha);
    double h2 = calculate_h(alpha + delta);
    return (h2 - h1) / delta;
}


/**
 * @brief 牛顿迭代法求解 α
 * @param h_target 目标高度[mm]
 * @return alpha * 180 / PI    弧度
 */
double solve_alpha(double h_target)
{
		h_target-=10;
    double alpha = PI/3 ;  // 初始值（60度）
    int iter = 0;

    while (iter < 1000)
		{
        double h = calculate_h(alpha);
        double dh_dalpha = calculate_derivative(alpha);
        double delta_alpha = (h_target - h) / dh_dalpha;  // 更新量
        alpha += delta_alpha;  // 更新 α

        // 检查是否收敛
        if (fabs(delta_alpha) < 1e-6)
				{
            break;
        }

        iter++;
    }

    return alpha * 180 / PI;
}



