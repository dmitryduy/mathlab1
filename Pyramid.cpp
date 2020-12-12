//
// Created by dmitry on 06.11.2020.
//

#include "Pyramid.h"


void Pyramid::DrawPyramid(RapidBody *rb) {

    glColor3f(1.0f, 1.0f, 1.0f);
    glColor3f(1.0f, 1.0f, 0.0f);

    glutWireTorus(rb->innerRadius, rb->outerRadius, 50 , 50);

}