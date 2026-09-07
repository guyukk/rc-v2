#ifndef _LINEARREGRESSION_
#define _LINEARREGRESSION_

#include "math.h"

// 定义 PI
#ifndef PI
#define PI 3.14159265358979323846
#endif

// 原有定义（如果其他地方用到就保留，否则可以删除）
typedef struct {
    double angle;
    double distance;
} PolarPoint, *PP;

typedef struct{
    double x;
    double y;
} CartesianPoint, *CP;

typedef struct{
    double X;
    double Y;
} Average, *Aver;

typedef struct{
    double a;
    double b;
} EquationParams, *EP;

// ========== 新增：拟合算法定义 ==========
typedef struct {
    double xDistance;
    double yDistance;
} DataPoint;

typedef enum {
    FIT_LINEAR = 1,
    FIT_QUADRATIC = 2,
    FIT_CUBIC = 3
} FitType;

typedef struct {
    FitType type;
    double coeffs[4];
    double curvature;
    double r_squared;
} FitResult;

// 函数声明
void linearFitting2D(DataPoint* points, int size, double* a, double* b);
void quadraticFitting2D(DataPoint* points, int size, double* a, double* b, double* c);
void cubicFitting2D(DataPoint* points, int size, double* a, double* b, double* c, double* d);
double calculateRSquared(DataPoint* points, int size, double* coeffs, FitType type);
void adaptiveFitting(DataPoint* points, int size, FitResult* result);
float calculateSteeringAngle(FitResult* left_fit, FitResult* right_fit, float front_distance);

#endif