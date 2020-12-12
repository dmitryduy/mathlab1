#include <iostream>
#include "RapidBody.h"
#define SQRT5 2.236067977499789694091736687312762

static const double a2 = 0.40;
static const double a3 = (14.0 - 3.0 * SQRT5) / 16.0;
static const double b31 = (-2889.0 + 1428.0 * SQRT5) / 1024.0;
static const double b32 = (3785.0 - 1620.0 * SQRT5) / 1024.0;
static const double b41 = (-3365.0 + 2094.0 * SQRT5) / 6040.0;
static const double b42 = (-975.0 - 3046.0 * SQRT5) / 2552.0;
static const double b43 = (467040.0 + 203968.0 * SQRT5) / 240845.0;
static const double g1 = (263.0 + 24.0 * SQRT5) / 1812.0;
static const double g2 = (125.0 - 1000.0 * SQRT5) / 3828.0;
static const double g3 = 1024.0 * (3346.0 + 1623.0 * SQRT5) / 5924787.0;
static const double g4 = (30.0 - 4.0 * SQRT5) / 123.0;

RapidBody::RapidBody(double x0, double y0, double z0, double mass, double innerRadius, double outerRadius) {
    this->x0 = x0;
    this->y0 = y0;
    this->z0 = z0;
    this->mass = mass;
    this->innerRadius = innerRadius;
    this->outerRadius = outerRadius;

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            this->R(i, j) = 0;
            if (i == j) {
                this->R(i, j) = 1;
            }
        }
    }

    this->IBody <<mass/8*(5*outerRadius*outerRadius+4*innerRadius*innerRadius),0,0,
    0,mass/8*(5*outerRadius*outerRadius+4*innerRadius*innerRadius),0,
    0,0,mass/4*(3*outerRadius*outerRadius+4*innerRadius*innerRadius);
    this->IBodyInv = this->IBody.reverse();

    this->x = {0, 0, 0};
    this->P = {0, 0, 0};
    this->L = {0.5, 0, 0};
    double y[stateSize];
    for (double &j : y) {
        j = 0;
    }
    stateToArray(this, y);

    arrayToState(this, y);
}

void RapidBody::stateToArray(RapidBody *rb, double *y) {
    *y++ = rb->x[0];
    *y++ = rb->x[1];
    *y++ = rb->x[2];
    for(int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            *y++ = rb->R(i, j);
    *y++ = rb->P[0];
    *y++ = rb->P[1];
    *y++ = rb->P[2];
    *y++ = rb->L[0];
    *y++ = rb->L[1];
    *y++ = rb->L[2];

}

void RapidBody::arrayToState(RapidBody *rb, double *y) {
    rb->x[0] = *y++;
    rb->x[1] = *y++;
    rb->x[2] = *y++;
    for(int i = 0; i < 3; i++)
        for(int j = 0; j < 3; j++)
            rb->R(i,j) = *y++;
    rb->P[0] = *y++;
    rb->P[1] = *y++;
    rb->P[2] = *y++;
    rb->L[0] = *y++;
    rb->L[1] = *y++;
    rb->L[2] = *y++;

    rb->v = rb->P*(1/ rb->mass);
    rb->Iinv = rb->R * rb->IBodyInv * rb->R.transpose();
    rb->omega = rb->Iinv * rb->L;

}

Matrix3d RapidBody::star(Vector3d a) {
    Matrix3d matrix;
    matrix << 0, -a[2], a[1],
             a[2], 0, -a[0],
             -a[1], a[0], 0;
    return matrix;
}

void RapidBody::computeForceAndTorque(RapidBody *rb, double t) {
    rb->force = {0, 0, 0};
    Vector3d r = {0, 0, 0};

}

void RapidBody::ddtStateToArray(double *xdot, RapidBody *rb) {
    *xdot++ = rb->v[0];
    *xdot++ = rb->v[1];
    *xdot++ = rb->v[2];

    Matrix3d Rdot = star(rb->omega) * rb->R;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            *xdot++ = Rdot(i, j);
    *xdot++ = rb->force[0];
    *xdot++ = rb->force[1];
    *xdot++ = rb->force[2];
    *xdot++ = rb->torque[0];
    *xdot++ = rb->torque[1];
    *xdot++ = rb->torque[2];

}

void RapidBody::dxdt( double t, double* y, double *ydot) {
    computeForceAndTorque(this, t);
    ddtStateToArray(ydot, this);
}

void RapidBody::ode(RapidBody *rb, double *y0, double *yEnd, int len, double t0, double t1) {
    double h = t1 - t0;

    double h2 = a2 * h;
    double h3 = a3 * h;

   /* k1 = (*f)(x0,y0);
    k2 = (*f)(x0+h2, y0 + h2 * k1);
    k3 = (*f)(x0+h3, y0 + h * (b31 * k1 + b32 * k2));
    x0 += h;
    k4 = (*f)(x0, y0 + h * (b41 * k1 + b42 * k2 + b43 * k3));
    y0 += h * ( g1 * k1 + g2 * k2 + g3 * k3 + g4 * k4 );

    return y0;*/
    //k1 = (*f)(x0,y0);
   double k1[len], k2[len], k3[len], k4[len];// x0 = k
   dxdt(t0, y0, k1);

    //k2 = (*f)(x0+h2, y0 + h2 * k1);
   double temp_y0_h2_k1[len];
   for (int i = 0 ; i < len; i++)
       temp_y0_h2_k1[i] = k1[i];


   mul(temp_y0_h2_k1, h2);
   sum(temp_y0_h2_k1, y0);

   dxdt(t0, temp_y0_h2_k1, k2);

   //k3 = (*f)(x0+h3, y0 + h * (b31 * k1 + b32 * k2));
   double temp_b31_k1[len], temp_b32_k2[len];
   for (int i = 0 ; i < len; i++)
       temp_b31_k1[i] = k1[i];
   mul(temp_b31_k1, b31);
    for (int i = 0 ; i < len; i++)
        temp_b32_k2[i] =k2[i];
    mul(temp_b32_k2, b32);
    sum(temp_b31_k1, temp_b32_k2);
    mul(temp_b31_k1, h);
    sum(temp_b31_k1, y0);
    dxdt(t0, temp_b31_k1, k3);
    //k4 = (*f)(x0, y0 + h * (b41 * k1 + b42 * k2 + b43 * k3));
    double temp_b41_k1[len], temp_b42_k2[len], temp_b43_k3[len];
    for (int i = 0 ; i < len ; i ++)
        temp_b41_k1[i] = k1[i];
    for (int i = 0 ; i < len ; i ++)
        temp_b42_k2[i] = k2[i];
    for (int i = 0 ; i < len ; i ++)
        temp_b43_k3[i] = k3[i];
    mul(temp_b41_k1, b41);
    mul(temp_b42_k2, b42);
    mul(temp_b43_k3, b43);
    sum(temp_b42_k2, temp_b43_k3);
    sum(temp_b41_k1, temp_b42_k2);
    mul(temp_b41_k1, h);
    sum(temp_b41_k1, y0);
    dxdt(t0, temp_b41_k1, k4);
    //y0 += h * ( g1 * k1 + g2 * k2 + g3 * k3 + g4 * k4 );
    for (int i = 0; i < len; i++)
    {
        yEnd[i] = y0[i] + h * (g1 * k1[i] + g2 * k2[i] + g3 * k3[i] + g4 * k4[i]);
    }

}

void RapidBody::sum(double *s1, double s2) {
    for (int i = 0; i < stateSize; i++)
        s1[i]+= s2;
}

void RapidBody::sum(double *s1, double *s2) {
    for (int i = 0; i < stateSize; i++)
        s1[i]+= s2[i];
}

void RapidBody::mul(double *m1, double m2) {
    for (int i = 0; i < stateSize; i++)
        m1[i]+= m2;
}

void RapidBody::printInvariant(){
    std::cout<<"Value: ";
    std::cout<<0.5*mass*v.transpose()*v+0.5*omega.transpose()*L<<" (time: "<<timeRB<<")"<<"\n";
}

void RapidBody::runSimulation(RapidBody *rb, double *x) {
    timeRB+=0.025;
    printInvariant();
    double y[stateSize], yEnd[stateSize];
    for (int i = 0; i < stateSize; i++) {
        y[i] = 0;
        yEnd[stateSize] = 0;
    }

    stateToArray(this, y);

    ode(this, y, yEnd, stateSize, timeRB-0.025, timeRB);

    arrayToState(this, yEnd);

    Vector3d v1 = rb->R.col(0);
    Vector3d v2 = rb->R.col(1);
    Vector3d v3 = rb->R.col(2);
    v1.normalize();
    v2=v2-(v1[0]*v2[0]+v1[1]*v2[1]+v1[2]*v2[2])*v1;
    v2.normalize();
    v3 = v3-(v1[0]*v3[0]+v1[1]*v3[1]+v1[2]*v3[2])*v1-(v3[0]*v2[0]+v3[1]*v2[1]+v3[2]*v2[2])*v2;
    v3.normalize();
    rb->R.col(0)=v1;
    rb->R.col(1)=v2;
    rb->R.col(2)=v3;

    stateToArray(this, x);

}