#ifndef VMLAB1_RAPIDBODY_H
#define VMLAB1_RAPIDBODY_H
#include <eigen3/Eigen/Dense>

typedef void (*DerivFunc)(double t, double *x, double *xdot);

using namespace Eigen;
static double timeRB = 0;
class RapidBody {
public:
    int stateSize = 18;
    int number_of_steps = 0;
    double mass, innerRadius, outerRadius;
    Matrix3d IBody, IBodyInv;
    Vector3d x;// x(t)
    Matrix3d R;
    Vector3d P, L;
    Matrix3d Iinv;
    Vector3d v, omega;
    Vector3d force, torque;
    double x0, y0, z0, height;
    RapidBody(double x0, double y0, double z0, double mass, double innerRadius, double outerRadius);
    void stateToArray(RapidBody *rb, double *y);
    void arrayToState(RapidBody *rb, double *y);
    Matrix3d star(Vector3d a);
    void computeForceAndTorque(RapidBody *rb, double t);
    void ddtStateToArray(double *xdot, RapidBody *rb);
    void dxdt( double t, double *y, double *ydot);
    void runSimulation(RapidBody *rb, double *x);
    void ode(RapidBody *rb, double *x0, double *xEnd, int len, double t0,
             double t1);
    void printInvariant();
    void sum(double* s1, double s2);
    void sum(double* s1, double *s2);
    void mul(double* m1, double m2);
};


#endif
