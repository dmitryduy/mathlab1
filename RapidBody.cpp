
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

void init(RigidBody* rb) {
    std::string tmp;
    std::cout<<"enter mass of torus(in range(0;10]). If you want to set mass by default, just press'q'\n";
    std::cin>>tmp;
    if (tmp == "q") {
        std::cout<<"mass set by default (10)\n";
        rb->mass = 10;
    }
    else {
        if (std::stod(tmp)<=0 || std::stod(tmp) >10) {
            std::cout<<"mass must be in range (0,10], mass will be set by default (10)\n";
            rb->mass = 10;
        }
        else rb->mass =  std::stod(tmp);
    }

    std::cout<<"enter inner radius(in range [0.1, 0.5]. If you want to set mass by default(0.5), just press'q'\n";
    std::cin>>tmp;
    if (tmp == "q") {
        std::cout<<"inner radius set by default (0.5)\n";
        rb->innerRadius = 0.3;
    }
    else {
        if (std::stod(tmp)<0.1 || std::stod(tmp) >0.5) {
            std::cout<<"inner radius must be in range [0.1,0.5], inner radius will be set by default (0.5)\n";
        }
        else rb->innerRadius =  std::stod(tmp);
    }

    std::cout<<"enter outer radius(in range [0.5, 1]). If you want to set mass by default(0.6), just press'q'\n";
    std::cin>>tmp;
    if (tmp == "q") {
        std::cout<<"outer radius set by default (0.6)\n";
        rb->outerRadius = 0.6;
    }
    else {
        if (std::stod(tmp)<0.5 || std::stod(tmp) >1) {
            std::cout<<"outer radius must be in range [0.5,1], outer radius will be set by default (0.6)\n";
        }
        else rb->outerRadius =  std::stod(tmp);
    }


    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            rb->R(i, j) = 0;
            if (i == j) {
                rb->R(i, j) = 1;
            }
        }
    }
    std::cout<<rb->outerRadius - rb->innerRadius<<std::endl;
    std::cout<<rb->mass*(5.0/8.0*0.25*(rb->outerRadius - rb->innerRadius)*(rb->outerRadius - rb->innerRadius) + 0.5*0.25*(rb->outerRadius+rb->innerRadius)*(rb->outerRadius+rb->innerRadius))<<"---------------\n";
    std::cout<<rb->mass*(3.0/4.0*0.25*(rb->outerRadius - rb->innerRadius)*(rb->outerRadius - rb->innerRadius) + 0.25*(rb->outerRadius+rb->innerRadius)*(rb->outerRadius+rb->innerRadius))<<std::endl;
     /* rb->IBody<<rb->mass*(5.0/8.0*0.25*(rb->outerRadius - rb->innerRadius)*(rb->outerRadius - rb->innerRadius) + 0.5*0.25*(rb->outerRadius+rb->innerRadius)*(rb->outerRadius+rb->innerRadius)),0,0,
            0,rb->mass*(5.0/8.0*0.25*(rb->outerRadius - rb->innerRadius)*(rb->outerRadius - rb->innerRadius) + 0.5*0.25*(rb->outerRadius+rb->innerRadius)*(rb->outerRadius+rb->innerRadius)),0,
            0,0,rb->mass*(3.0/4.0*0.25*(rb->outerRadius - rb->innerRadius)*(rb->outerRadius - rb->innerRadius) +0.25*(rb->outerRadius+rb->innerRadius)*(rb->outerRadius+rb->innerRadius));
    */
     rb->IBody<<rb->mass/8.0*(5*rb->innerRadius*rb->innerRadius + 4*rb->outerRadius*rb->outerRadius),0,0,
     0,rb->mass/8.0*(5*rb->innerRadius*rb->innerRadius + 4*rb->outerRadius*rb->outerRadius),0,
     0,0,rb->mass/8.0*(5*rb->innerRadius*rb->innerRadius + 4*rb->outerRadius*rb->outerRadius);
     rb->IBodyInv = rb->IBody.reverse();
    rb->x = {0, 0, 0};
    rb->P = {0, 0, 0};
    rb->L = {0.5, 0.5, 0.2};
    double y[18];
    for (double &j : y) {
        j = 0;
    }
    stateToArray(rb, y);

    arrayToState(rb, y);
}



void stateToArray(RigidBody *rb, double *y) {
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

void arrayToState(RigidBody *rb, double *y) {
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

Matrix3d star(Vector3d a) {
    Matrix3d matrix;
    matrix << 0, -a[2], a[1],
             a[2], 0, -a[0],
             -a[1], a[0], 0;
    return matrix;
}

void computeForceAndTorque(RigidBody *rb, double t) {
    rb->force = {0, 0, 0};
    Vector3d r = {0, 0, 0};

}

void ddtStateToArray(double *xdot, RigidBody *rb) {
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

void dxdt( double t, double* y, double *ydot, RigidBody* rb) {
    computeForceAndTorque(rb, t);
    ddtStateToArray(ydot, rb);
}

void sum(double *s1, double s2) {
    for (int i = 0; i < 18; i++)
        s1[i]+= s2;
}

void sum(double *s1, double *s2) {
    for (int i = 0; i < 18; i++)
        s1[i]+= s2[i];
}

void mul(double *m1, double m2) {
    for (int i = 0; i < 18; i++)
        m1[i]+= m2;
}

void ode(RigidBody *rb, double *y0, double *yFinal, int len, double t0, double t1) {
    double h = t1 - t0;


 double h2 = a2 * h;
     double h3 = a3 * h;


 /*k1 = (*f)(x0,y0);
    k2 = (*f)(x0+h2, y0 + h2 * k1);
    k3 = (*f)(x0+h3, y0 + h * (b31 * k1 + b32 * k2));
    x0 += h;
    k4 = (*f)(x0, y0 + h * (b41 * k1 + b42 * k2 + b43 * k3));
    y0 += h * ( g1 * k1 + g2 * k2 + g3 * k3 + g4 * k4 );

    return y0;*/
    //k1 = (*f)(x0,y0);
   double k1[len], k2[len], k3[len], k4[len];// x0 = k
   dxdt(t0, y0, k1, rb);

    //k2 = (*f)(x0+h2, y0 + h2 * k1);
   double temp_y0_h2_k1[len];
   for (int i = 0 ; i < len; i++)
       temp_y0_h2_k1[i] = k1[i];


   mul(temp_y0_h2_k1, h2);
   sum(temp_y0_h2_k1, y0);

   dxdt(t0, temp_y0_h2_k1, k2, rb);

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
    dxdt(t0, temp_b31_k1, k3, rb);
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
    dxdt(t0, temp_b41_k1, k4, rb);
    //y0 += h * ( g1 * k1 + g2 * k2 + g3 * k3 + g4 * k4 );
    for (int i = 0; i < len; i++)
    {
        yFinal[i] = y0[i] + h * (g1 * k1[i] + g2 * k2[i] + g3 * k3[i] + g4 * k4[i]);
    }

}

void printInvariant(RigidBody *rb){
    std::cout<<"Value: ";
    std::cout.precision(11);
    std::cout<<0.5*rb->mass*rb->v.transpose()*rb->v+0.5*rb->omega.transpose()*rb->L<<" (time: "<<timeRB<<")"<<"\n";
}

void runSimulation(RigidBody *rb, double *x) {
    timeRB+=0.025;
    printInvariant(rb);
    double y[18], yEnd[18];
    for (int i = 0; i < 18; i++) {
        y[i] = 0;
        yEnd[18] = 0;
    }

    stateToArray(rb, y);

    ode(rb, y, yEnd, 18, timeRB-0.025, timeRB);

    arrayToState(rb, yEnd);

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

    stateToArray(rb, x);

}
