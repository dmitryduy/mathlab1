#include "Pyramid.h"
#include "RapidBody.h"
 //Handler for window re-size event. Called back when the window first appears and
  // whenever the window is re-sized with its new width and height


int main(int argc, char** argv) {
    glutInit(&argc, argv);            // Initialize GLUT
    glutInitDisplayMode(GLUT_DOUBLE| GLUT_RGB| GLUT_DEPTH); // Enable double buffered mode
    glutCreateWindow("h");          // Create window with the given title
    glEnable(GL_DEPTH_TEST);
    glutDisplayFunc(display);       // Register callback handler for window re-paint event
    glutReshapeFunc(reshape);       // Register callback handler for window re-size event
    initGL();                       // Our own OpenGL initialization
    glutMainLoop();                 // Enter the infinite event-processing loop
    return 0;
}


