//
// Created by dmitry on 06.11.2020.
//

#include "Pyramid.h"

void Pyramid::DrawPyramid(RapidBody *rb) {     glBegin(GL_TRIANGLES);           // Begin drawing the pyramid with 4 triangles
    // Front
    glColor3f(1.0f, 0.0f, 0.0f);     // Red
    glVertex3d( rb->x0, rb->y0, rb->z0);
    glColor3f(0.0f, 0.5f, 0.0f);     // Green
    glVertex3d(-rb->x0, -rb->y0, rb->z0);
    glColor3f(0.0f, 0.0f, 1.0f);     // Blue
    glVertex3d(rb->x0, -rb->y0, rb->z0);
    glEnd();
    // Right
    glBegin(GL_TRIANGLES);
    glColor3f(1.0f, 0.0f, 0.0f);     // Red
    glVertex3d(rb->x0, rb->y0, rb->z0);
    glColor3f(0.0f, 0.0f, 1.0f);     // Blue
    glVertex3d(rb->x0, -rb->y0, rb->z0);
    glColor3f(0.0f, 1.0f, 0.0f);     // Green
    glVertex3d(rb->x0, -rb->y0, -rb->z0);
    glEnd();

    // Back
    glBegin(GL_TRIANGLES);
    glColor3f(1.0f, 0.0f, 0.0f);     // Red
    glVertex3d(rb->x0, rb->y0, rb->z0);
    glColor3f(0.0f, 1.0f, 0.0f);     // Green
    glVertex3d(rb->z0, -rb->y0, -rb->z0);
    glColor3f(0.0f, 0.0f, 1.0f);     // Blue
    glVertex3d(-rb->x0, -rb->y0, -rb->z0);
    glEnd();
    // Left
    glBegin(GL_TRIANGLES);
    glColor3f(1.0f,0.0f,0.0f);       // Red
    glVertex3d( rb->x0, rb->y0, rb->z0);
    glColor3f(0.0f,0.0f,1.0f);       // Blue
    glVertex3d(-rb->x0,-rb->y0,-rb->z0);
    glColor3f(0.0f,1.0f,0.0f);       // Green
    glVertex3d(-rb->x0,-rb->y0, rb->z0);
    glEnd();   // Done drawing the pyramid

    glBegin(GL_QUADS);// основание пирамиды
    glColor3f(1.0,0.51,0.28); // сделали основание рыжим





    glVertex3d(-rb->x0, -rb->y0, rb->z0);
    glVertex3d(rb->x0, -rb->y0, rb->z0);
    glVertex3d(rb->z0, -rb->y0, -rb->z0);
    glVertex3d(-rb->x0,-rb->y0,-rb->z0);
    glEnd();
}