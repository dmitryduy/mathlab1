#include "Pyramid.h"
#include "RapidBody.h"
static double *y = new double[18];
static double* m = new double[16];
RapidBody *rb =  new RapidBody(1.0,2.0,1.0,5,0.5, 1.5);

void reshape(GLsizei width, GLsizei height) {  // GLsizei for non-negative integer
    // Compute aspect ratio of the new window
    if (height == 0) height = 1;                // To prevent divide by 0
    GLfloat aspect = (GLfloat) width / (GLfloat) height;

    // Set the viewport to cover the new window
    glViewport(0, 0, width, height);

    // Set the aspect ratio of the clipping volume to match the viewport
    glMatrixMode(GL_PROJECTION);  // To operate on the Projection matrix
    glLoadIdentity();             // Reset
    // Enable perspective projection with fovy, aspect, zNear and zFar
    //gluPerspective(45.0f, aspect, 0.1f, 100.0f);
    gluPerspective(30.0f, aspect, 1.0f, 100.0f);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(0,100,0,0,0,0,0,0,1);
}
void initGL() {

    for (int i = 0; i < 18; i ++) {
        y[i] = 0;
    }
    for (int i = 0; i < 16; i ++) {
        m[i]=0;
    }
    m[0]=1;
    m[5]=1;
    m[10]=1;
    m[15]=1;
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f); // Set background color to black and opaque
    glClearDepth(1.0f);// Set background depth to farthest
    glEnable(GL_DEPTH_TEST);   // Enable depth testing for z-culling
    glDepthFunc(GL_LEQUAL);    // Set the type of depth-test
    glShadeModel(GL_SMOOTH);   // Enable smooth shading
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);  // Nice perspective corrections
}

void timer(int value) {
    glutPostRedisplay();      // Post re-paint request to activate display()
     //glutTimerFunc(15, timer, 0); // next timer call milliseconds later
}
void display() {
    Pyramid *pyramid = new Pyramid();
    rb->runSimulation(rb, y);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // Clear color and depth buffers
    glMatrixMode(GL_MODELVIEW);     // To operate on model-view matrix

    // Render a color-cube consisting of 6 quads with different colors
    glLoadIdentity();                 // Reset the model-view matrix
    for (int i = 0; i < 3; i++) {
        m[0+i*4]=y[3+i];
        m[1+i*4]=y[6+i];
        m[2+i*4]=y[9+i];
    }
    glTranslated(y[0], y[1], -10 + y[2]);
    glMultMatrixd(m);
    //glLoadMatrixd(m);




    pyramid->DrawPyramid(rb);

    glutSwapBuffers();
    glutTimerFunc(15, timer, 0);
}

/* Handler for window re-size event. Called back when the window first appears and
   whenever the window is re-sized with its new width and height */

int main(int argc, char** argv) {
    std::string tmp;
    std::cout<<"enter mass of torus(in range(0;10]). If you want to set mass by default, just press'q'\n";
    std::cin>>tmp;
    if (tmp == "q") std::cout<<"mass set by default (10)\n";
    else {
        if (std::stod(tmp)<=0 || std::stod(tmp) >10) {
            std::cout<<"mass must be in range (0,10], mass will be set by default (10)\n";
        }
        else rb->mass =  std::stod(tmp);
    }

    std::cout<<"enter inner radius(in range [0.5, 1]. If you want to set mass by default(0.5), just press'q'\n";
    std::cin>>tmp;
    if (tmp == "q") std::cout<<"inner radius set by default (0.5)\n";
    else {
        if (std::stod(tmp)<0.5 || std::stod(tmp) >1) {
            std::cout<<"inner radius must be in range [0.5,1], inner radius will be set by default (0.5)\n";
        }
        else rb->innerRadius =  std::stod(tmp);
    }

    std::cout<<"enter outer radius(in range [1, 2]). If you want to set mass by default(1.5), just press'q'\n";
    std::cin>>tmp;
    if (tmp == "q") std::cout<<"outer radius set by default (0.5)\n";
    else {
        if (std::stod(tmp)<1 || std::stod(tmp) >2) {
            std::cout<<"outer radius must be in range [1,2], outer radius will be set by default (1.5)\n";
        }
        else rb->outerRadius =  std::stod(tmp);
    }
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