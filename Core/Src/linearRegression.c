#include "linearRegression.h"
#include "math.h"

// ============ 1. 线性拟合 ============
void linearFitting2D(DataPoint* points, int size, double* a, double* b) {
    double sum_x = 0, sum_y = 0, sum_x2 = 0, sum_xy = 0;
    int valid_count = 0;
    
    for (int i = 0; i < size; ++i) {
        if (points[i].xDistance > 0 && points[i].yDistance > 0) {
            sum_x += points[i].xDistance;
            sum_y += points[i].yDistance;
            sum_x2 += points[i].xDistance * points[i].xDistance;
            sum_xy += points[i].xDistance * points[i].yDistance;
            valid_count++;
        }
    }
    
    if (valid_count > 1) {
        double denominator = valid_count * sum_x2 - sum_x * sum_x;
        if (fabs(denominator) < 1e-6) {
            *a = 0;
            *b = sum_y / valid_count;
        } else {
            *a = (valid_count * sum_xy - sum_x * sum_y) / denominator;
            *b = (sum_y * sum_x2 - sum_x * sum_xy) / denominator;
        }
    } else {
        *a = 0;
        *b = 0;
    }
}

// ============ 2. 二次拟合 ============
void quadraticFitting2D(DataPoint* points, int size, double* a, double* b, double* c) {
    double Sx = 0, Sx2 = 0, Sx3 = 0, Sx4 = 0;
    double Sy = 0, Sxy = 0, Sx2y = 0;
    int n = 0;
    
    for (int i = 0; i < size; i++) {
        double x = points[i].xDistance;
        double y = points[i].yDistance;
        if (x > 0 && y > 0) {
            n++;
            Sx += x;
            Sx2 += x * x;
            Sx3 += x * x * x;
            Sx4 += x * x * x * x;
            Sy += y;
            Sxy += x * y;
            Sx2y += x * x * y;
        }
    }
    
    if (n < 3) {
        linearFitting2D(points, size, b, c);
        *a = 0;
        return;
    }
    
    // 构建 3x4 增广矩阵
    double A[3][4] = {
        {Sx4, Sx3, Sx2, Sx2y},
        {Sx3, Sx2, Sx,  Sxy },
        {Sx2, Sx,  n,   Sy  }
    };
    
    // 高斯消元法求解
    for (int i = 0; i < 3; i++) {
        // 选主元
        double maxVal = fabs(A[i][i]);
        int maxRow = i;
        for (int k = i + 1; k < 3; k++) {
            if (fabs(A[k][i]) > maxVal) {
                maxVal = fabs(A[k][i]);
                maxRow = k;
            }
        }
        
        if (maxRow != i) {
            for (int j = i; j < 4; j++) {
                double tmp = A[i][j];
                A[i][j] = A[maxRow][j];
                A[maxRow][j] = tmp;
            }
        }
        
        double div = A[i][i];
        if (fabs(div) < 1e-12) continue;
        
        for (int j = i; j < 4; j++) A[i][j] /= div;
        
        for (int k = 0; k < 3; k++) {
            if (k == i) continue;
            double factor = A[k][i];
            for (int j = i; j < 4; j++) 
                A[k][j] -= factor * A[i][j];
        }
    }
    
    *a = A[0][3];
    *b = A[1][3];
    *c = A[2][3];
}

// ============ 3. 三次拟合 ============
void cubicFitting2D(DataPoint* points, int size, double* a, double* b, double* c, double* d) {
    double s[7] = {0};
    double sy[4] = {0};
    int n = 0;
    
    for (int i = 0; i < size; i++) {
        double x = points[i].xDistance;
        double y = points[i].yDistance;
        if (x > 0 && y > 0) {
            n++;
            double x_pow = 1.0;
            for (int k = 0; k <= 6; k++) {
                if (k <= 3) sy[k] += x_pow * y;
                s[k] += x_pow;
                x_pow *= x;
            }
        }
    }
    
    if (n < 4) {
        quadraticFitting2D(points, size, b, c, d);
        *a = 0;
        return;
    }
    
    // 构建 4x5 增广矩阵
    double A[4][5] = {
        {s[6], s[5], s[4], s[3], sy[3]},
        {s[5], s[4], s[3], s[2], sy[2]},
        {s[4], s[3], s[2], s[1], sy[1]},
        {s[3], s[2], s[1], s[0], sy[0]}
    };
    
    // 高斯消元求解
    for (int i = 0; i < 4; i++) {
        double max = fabs(A[i][i]);
        int maxRow = i;
        for (int k = i + 1; k < 4; k++) {
            if (fabs(A[k][i]) > max) {
                max = fabs(A[k][i]);
                maxRow = k;
            }
        }
        
        if (maxRow != i) {
            for (int j = i; j < 5; j++) {
                double tmp = A[i][j];
                A[i][j] = A[maxRow][j];
                A[maxRow][j] = tmp;
            }
        }
        
        double div = A[i][i];
        if (fabs(div) < 1e-12) continue;
        
        for (int j = i; j < 5; j++) A[i][j] /= div;
        
        for (int k = 0; k < 4; k++) {
            if (k == i) continue;
            double factor = A[k][i];
            for (int j = i; j < 5; j++)
                A[k][j] -= factor * A[i][j];
        }
    }
    
    *a = A[0][4];
    *b = A[1][4];
    *c = A[2][4];
    *d = A[3][4];
}

// ============ 4. 计算R²拟合优度 ============
double calculateRSquared(DataPoint* points, int size, double* coeffs, FitType type) {
    double ss_res = 0;
    double ss_tot = 0;
    double mean_y = 0;
    int n = 0;
    
    for (int i = 0; i < size; i++) {
        if (points[i].xDistance > 0 && points[i].yDistance > 0) {
            mean_y += points[i].yDistance;
            n++;
        }
    }
    if (n == 0) return 0;
    mean_y /= n;
    
    for (int i = 0; i < size; i++) {
        double x = points[i].xDistance;
        double y = points[i].yDistance;
        if (x > 0 && y > 0) {
            double y_pred = 0;
            if (type == FIT_LINEAR) {
                y_pred = coeffs[0] * x + coeffs[1];
            } else if (type == FIT_QUADRATIC) {
                y_pred = coeffs[0] * x * x + coeffs[1] * x + coeffs[2];
            } else if (type == FIT_CUBIC) {
                y_pred = coeffs[0] * x * x * x + coeffs[1] * x * x + 
                         coeffs[2] * x + coeffs[3];
            }
            
            ss_res += (y - y_pred) * (y - y_pred);
            ss_tot += (y - mean_y) * (y - mean_y);
        }
    }
    
    return 1.0 - (ss_res / (ss_tot + 1e-10));
}

// ============ 5. 自适应拟合选择 ============
void adaptiveFitting(DataPoint* points, int size, FitResult* result) {
    double r2_linear, r2_quad, r2_cubic;
    double linear_coeffs[2] = {0};
    double quad_coeffs[3] = {0};
    double cubic_coeffs[4] = {0};
    
    // 线性拟合
    linearFitting2D(points, size, &linear_coeffs[0], &linear_coeffs[1]);
    r2_linear = calculateRSquared(points, size, linear_coeffs, FIT_LINEAR);
    
    // 二次拟合
    quadraticFitting2D(points, size, &quad_coeffs[0], &quad_coeffs[1], &quad_coeffs[2]);
    r2_quad = calculateRSquared(points, size, quad_coeffs, FIT_QUADRATIC);
    
    // 三次拟合
    cubicFitting2D(points, size, &cubic_coeffs[0], &cubic_coeffs[1], 
                   &cubic_coeffs[2], &cubic_coeffs[3]);
    r2_cubic = calculateRSquared(points, size, cubic_coeffs, FIT_CUBIC);
    
    // 选择最佳拟合
    if (r2_linear > 0.95) {
        result->type = FIT_LINEAR;
        result->coeffs[0] = 0;
        result->coeffs[1] = 0;
        result->coeffs[2] = linear_coeffs[0];
        result->coeffs[3] = linear_coeffs[1];
        result->r_squared = r2_linear;
    }
    else if (r2_quad > r2_linear + 0.05 && r2_quad > 0.90) {
        result->type = FIT_QUADRATIC;
        result->coeffs[0] = 0;
        result->coeffs[1] = quad_coeffs[0];
        result->coeffs[2] = quad_coeffs[1];
        result->coeffs[3] = quad_coeffs[2];
        result->r_squared = r2_quad;
    }
    else if (r2_cubic > r2_quad + 0.05) {
        result->type = FIT_CUBIC;
        result->coeffs[0] = cubic_coeffs[0];
        result->coeffs[1] = cubic_coeffs[1];
        result->coeffs[2] = cubic_coeffs[2];
        result->coeffs[3] = cubic_coeffs[3];
        result->r_squared = r2_cubic;
    }
    else {
        result->type = FIT_QUADRATIC;
        result->coeffs[0] = 0;
        result->coeffs[1] = quad_coeffs[0];
        result->coeffs[2] = quad_coeffs[1];
        result->coeffs[3] = quad_coeffs[2];
        result->r_squared = r2_quad;
    }
    
    // 计算曲率指标
    if(result->type == FIT_LINEAR) {
    result->curvature = 0;
    } else if(result->type == FIT_QUADRATIC) {
    // 二次项系数，单位 1/cm²
    // 乘以合适的系数使数值便于观察
    result->curvature = fabs(result->coeffs[1]) * 100000;
    } else if(result->type == FIT_CUBIC) {
    // 三次拟合综合曲率
    result->curvature = fabs(result->coeffs[1]) * 100000;
    }
}

// ============ 6. 计算预测转角 ============
float calculateSteeringAngle(FitResult* left_fit, FitResult* right_fit, float front_distance) {
    double x = front_distance;
    
    double y_left = left_fit->coeffs[0] * x * x * x + 
                    left_fit->coeffs[1] * x * x + 
                    left_fit->coeffs[2] * x + 
                    left_fit->coeffs[3];
    
    double y_right = right_fit->coeffs[0] * x * x * x + 
                     right_fit->coeffs[1] * x * x + 
                     right_fit->coeffs[2] * x + 
                     right_fit->coeffs[3];
    
    double dy_left = 3 * left_fit->coeffs[0] * x * x + 
                     2 * left_fit->coeffs[1] * x + 
                     left_fit->coeffs[2];
    
    double dy_right = 3 * right_fit->coeffs[0] * x * x + 
                      2 * right_fit->coeffs[1] * x + 
                      right_fit->coeffs[2];
    
    double avg_slope = (dy_left + dy_right) / 2.0;
    float angle = atan(avg_slope) * 180.0 / PI;
    
    return angle;
}