
#include "RapidBody.h"

RapidBody::RapidBody(double x0, double y0, double z0, int mass, double height) {
    this->x0 = x0;
    this->y0 = y0;
    this->z0 = z0;
    this->mass = mass;
    this->height = height;

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            this->R(i, j) = 0;
            if (i == j) {
                this->R(i, j) = 1;
            }
        }
    }

    this->IBody << y0*y0 + z0*z0, x0*y0, x0*z0,
                  x0*y0, x0*x0+z0*z0, y0*z0,
                  x0*z0, y0*z0, y0*y0 + x0*x0;
    this->IBody = this->IBody * (this->mass / 12);
    this->IBodyInv = this->IBody.reverse();

    this->x = {0, 0, 0};
    this->P = {0, 0, 0};
    this->L = {0.5, 1, 1};
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

    rb->v = rb->P/ rb->mass;
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
    rb->torque = rb->force.cross(r);
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

void RapidBody::dxdt(RapidBody *rb, double t, double *xdot) {
    computeForceAndTorque(rb, t);
    ddtStateToArray(xdot, rb);
}

void RapidBody::ode(RapidBody *rb, double *x0, double *xEnd, int len, double t0, double t1) {
    double dot1[len];
    for (int i = 0; i < len; i++) {
        dot1[i] = 0;
    }
    this->dxdt(rb, t0, dot1);
    double dot2[len];
    for (int i = 0; i < len; i++) {
        dot2[i] = 0;
    }
    double x1[len];
    for (int i = 0; i < len; i++) {
        x1[i] = x0[i] + (t1 - t0) / 2 * dot1[i];
    }

    this->dxdt(rb, t1/2, dot2);
    for (int i = 0; i < len; i++) {
        xEnd[i] = x0[i] + (t1 - t0) * dot2[i];
    }
}

void RapidBody::runSimulation(RapidBody *rb, double *x) {
    times += 0,01;
    double y[stateSize], yEnd[stateSize];
    for (int i = 0; i < stateSize; i++) {
        y[i] = 0;
        yEnd[stateSize] = 0;
    }

    stateToArray(this, y);

    ode(this, y, yEnd, stateSize, times, times+0.001);

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