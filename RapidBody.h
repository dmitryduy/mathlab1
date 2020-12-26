#ifndef VMLAB1_RAPIDBODY_H
#define VMLAB1_RAPIDBODY_H
#include <eigen3/Eigen/Dense>



using namespace Eigen;

struct RigidBody {
/* Constant quantities */
    double mass, innerRadius, outerRadius; /* mass M */

    Matrix3d IBody; /* Ibody */
    Matrix3d IBodyInv; /* I−1 body (inverse of Ibody) */
/* State variables */
    Vector3d x;   /* x(t) */
    Matrix3d R;   /* R(t) */
    Vector3d P;  /* P(t) */
    Vector3d L;  /* L(t) */
/* Derived quantities (auxiliary variables) */
    Matrix3d Iinv;    /* I−1(t) */
    Vector3d v;   /* v(t) */
    Vector3d omega;      /* ω(t) */
/* Computed quantities */
    Vector3d force;    /* F(t) */
    Vector3d torque;         /* τ(t) */


    double x0,y0,z0;
};


static RigidBody *rb = new RigidBody();

static double timeRB = 0;
    void init(RigidBody* rb);
    void stateToArray(RigidBody *rb, double *y);
    void arrayToState(RigidBody *rb, double *y);
    Matrix3d star(Vector3d a);
    void computeForceAndTorque(RigidBody *rb, double t);
    void ddtStateToArray(double *xdot, RigidBody *rb);
    void dxdt( double t, double *y, double *ydot, RigidBody* rb);
    void runSimulation(RigidBody *rb, double *x);
    void ode(RigidBody *rb, double *x0, double *xEnd, int len, double t0,
             double t1);
    void printInvariant(RigidBody *rb);
    void sum(double* s1, double s2);
    void sum(double* s1, double *s2);
    void mul(double* m1, double m2);


#endif
