/***********************************************************
             CSC418 - St George , Winter 2017 version

                     Boids.cpp

	This assignment will help you become familiar with
	the basic structure and shape of an OpenGL program.

	Please take time to read through the code and note
	how the viewing parameters, viewing volume, and
	general OpenGL options are set. You will need to
	change those in future assignments.

	You should also pay attention to the way basic
	OpenGL drawing commands work and what they do.
        You should check the OpenGL reference manual
        for the different OpenGL functions you will
        find here. In particular, those that set up
        the viewing parameters.

	Note that this program is intended to display
	moving objects in real time. As such, it is
	strongly recommended that you run this code locally,
	on one of the machines at the CS lab. Alternately,
	install the OpenGL libraries on your own computer.

	Working remotely over ssh, or working on a non-
	Linux machine will give you headaches.

    Instructions:

	The assignment handout contains a detailed
	description of what you need to do. Please be
	sure to read it carefully.

	You must complete all sections marked
        // TO DO

	In addition to this, you have to complete
	all information requested in the file called
	REPORT.TXT. Be sure to answer in that
	report any
	// QUESTION:
	parts found in the code below.

	Sections marked
	// CRUNCHY:
	Are worth extra credit. How much bonus you get
	depends on the quality of your extensions
	or enhancements. Be sure to indicate in your
	REPORT.TXT any extra work you have done.

	The code is commented, and the comments provide
	information about what the program is doing and
	what your task will be.

	As a reminder. Your code must compile and run
	on 'mathlab.utsc.utoronto.ca' under Linux. We will
	not grade code that fails to compile or run
	on these machines.

Written by: F. Estrada, Jun 2011.
            Main loop/init derived from older 418
            OpenGL assignments
            Updated, Jan. 2017
***********************************************************/

/*
  Headers for 3DS management - model loading for point clouds
*/
#include<lib3ds/types.h>
#include<lib3ds/mesh.h>
#include<lib3ds/file.h>

/*
   Headers for OpenGL libraries. If you want to run this
   on your computer, make sure you have installed OpenGL,
   GLUT, and GLUI

   NOTE: The paths below assume you're working on mathlab.
   On your system the libraries may be elsewhere. Be sure
   to check the correct location for include files and
   library files (you may have to update the Makefile)
*/
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <GL/glui.h>

/* Standard C libraries */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>

// *************** GLOBAL VARIABLES *************************
#define MAX_BOIDS 2000
#define SPACE_SCALE 75
#define SPEED_SCALE 2
#define ADJUST_SPEED .2
#define PREDATOR_SPEED 0.05
const float PI = 3.14159;
int nBoids;				// Number of boids to dispay
int nLeaders;				// Number of leaders in the population (1 to 5)
float Boid_Location[MAX_BOIDS][3];	// Pointers to dynamically allocated
float Boid_Velocity[MAX_BOIDS][3];	// Boid position & velocity data
float Boid_Color[MAX_BOIDS][3];	 	// RGB colour for each boid
float *modelVertices;                   // Imported model vertices
int n_vertices;                         // Number of model vertices

float Boid_Previous_Locations[MAX_BOIDS][100][3]; // Holds the previous 10 locations of each boid
int Boid_Leaders[5] = {-1,-1,-1,-1,-1}; // Boids that are considered leaders, -1 is place holder for not chosen
int Boid_Predator = -1;
float Obstacle_Location[3];
float Obstacle_Radius;

// *************** USER INTERFACE VARIABLES *****************
int windowID;               // Glut window ID (for display)
GLUI *glui;                 // Glui window (for controls)
int Win[2];                 // window (x,y) size
float r_rule1;		    // Parameters of the boid update function.
float r_rule2;		    // see updateBoid()
float r_rule3;
float r_leader;
float r_predator;
float k_rule1;
float k_rule2;
float k_rule3;
float k_rule0;
float k_leader;
float k_predator;
float k_obstacle;
float shapeness;
float global_rot;
int show_frame;
int predator_on;
int obstacle_on;
int boid_trail_on;

// ***********  FUNCTION HEADER DECLARATIONS ****************
// Initialization functions
void initGlut(char* winName);
void initGlui();
void GL_Settings_Init();
float *read3ds(const char *name, int *n);

// Callbacks for handling events in glut
void WindowReshape(int w, int h);
void WindowDisplay(void);

// Callback for handling events in glui
void GLUI_Control(int id);

// Return the current system clock (in seconds)
double getTime();

// Functions for handling Boids
float sign(float x){if (x>=0) return(1.0); else return(-1.0);}
void updateBoid(int i);
void drawBoid(int i);
void HSV2RGB(float H, float S, float V, float *R, float *G, float *B);

// ******************** FUNCTIONS ************************

/*
   main()

   Read command line parameters, initialize boid positions
   and velocities, and initialize all OpenGL data needed
   to set up the image window. Then call the GLUT main loop
   which handles the actual drawing
*/
int main(int argc, char** argv)
{
    // Process program arguments
    if(argc < 4 || argc > 5) {
        fprintf(stderr,"Usage: Boids width height nBoids [3dmodel]\n");
        fprintf(stderr," width & height control the size of the graphics window\n");
        fprintf(stderr," nBoids determined the number of Boids to draw.\n");
        fprintf(stderr," [3dmodel] is an optional parameter, naming a .3ds file to be read for 3d point clouds.\n");
        exit(0);
    }
    Win[0]=atoi(argv[1]);
    Win[1]=atoi(argv[2]);
    nBoids=atoi(argv[3]);

    if (nBoids>MAX_BOIDS)
    {
     fprintf(stderr,"Too Many Boids! Max=%d\n",MAX_BOIDS);
     exit(0);
    }

    // If a model file is specified, read it, normalize scale
    n_vertices=0;
    modelVertices=NULL;
    if (argc==5)
    {
     float mx=0;
     n_vertices=nBoids;
     modelVertices=read3ds(argv[4],&n_vertices);
     if (n_vertices>0)
     {
      fprintf(stderr,"Returned %d points\n",n_vertices);
      for (int i=0; i<n_vertices*3; i++)
       if (fabs(*(modelVertices+i))>mx) mx=fabs(*(modelVertices+i));
      for (int i=0; i<n_vertices*3; i++) *(modelVertices+i)/=mx;
      for (int i=0; i<n_vertices*3; i++) *(modelVertices+i)*=(SPACE_SCALE*.5);
     }
    }

    // Initialize Boid positions and velocity
    // Mind the SPEED_SCALE. You may need to change it to
    // achieve smooth animation - increase it if the
    // animation is too slow. Decrease it if it's too
    // fast and choppy.
    srand48(1522);
    for (int i=0; i<nBoids; i++)
    {
     // Initialize Boid locations and velocities randomly
     Boid_Location[i][0]=(-.5+drand48())*SPACE_SCALE;
     Boid_Location[i][1]=(-.5+drand48())*SPACE_SCALE;
     Boid_Location[i][2]=(-.5+drand48())*SPACE_SCALE;
     Boid_Velocity[i][0]=(-.5+drand48())*SPEED_SCALE;
     Boid_Velocity[i][1]=(-.5+drand48())*SPEED_SCALE;
     Boid_Velocity[i][2]=(-.5+drand48())*SPEED_SCALE;
     
     // Initialize all 10 previous locations of the Boid to its starting location
     for (int j = 0; j < 100; j++) {
       for (int k = 0; k < 3; k++) {
	 Boid_Previous_Locations[i][j][k] = Boid_Location[i][k];
       }
     }
     // Initialize boid colour to solid blue-ish
     // You may want to change this
     Boid_Color[i][0]=.15;
     Boid_Color[i][1]=.15;
     Boid_Color[i][2]=1;
    }
    
     
    //Variables used for finding leaders and predators 
    bool found;
    int temp; 
     
    //Randomly select 5 leaders for the simulation
    //All other fish will be attracted to the leaders movement
    nLeaders = nBoids / 10;
    if (nLeaders < 1) {
      nLeaders = 1;
    } else if (nLeaders > 5) {
      nLeaders = 5;
    }
    
    //Find nLeaders (1 to 5) unique, random boids to make leaders
    for (int j = 0; j < nLeaders; j++) {
      //Select a random boid, look through the list of existing leaders
      //If the boid is in the list, select a new leader and test it again
      //if the boid is not in the list, insert it
      do {
	found = true;
	temp = rand() % nBoids;
	for (int k = 0; k < nLeaders; k++) {
	  if (temp == Boid_Leaders[k]) {
	    found = false;
	  }
	}
      } while(!found);
      
      Boid_Leaders[j] = temp;
    }
    
    //if there are greater than 10 boids, introduce a predators
    //select a random integer, if that number is already a leader try again
    //if it is not a leader, set that boid as the predator
    if (nBoids >= 10) {
      do {
	found = true;
	temp = rand() % nBoids;
	for (int k = 0; k < nLeaders; k++) {
	  if (temp == Boid_Leaders[k]) {
	    found = false;
	  }
	}
      } while (!found);
      Boid_Predator = temp;
    }
    
    //Initialize the obstacle
    Obstacle_Location[0] = 0;
    Obstacle_Location[1] = 0;
    Obstacle_Location[2] = 0;
    Obstacle_Radius = 20;

    // Initialize glut, glui, and opengl
    glutInit(&argc, argv);
    initGlut(argv[0]);
    initGlui();
    GL_Settings_Init();

    // Initialize variables that control the boid updates
    r_rule1=10;
    r_rule2=5;
    r_rule3=25;
    r_leader=10;
    r_predator=20;
    k_rule1=.15;
    k_rule2=.5;
    k_rule3=.15;
    k_rule0=.5;
    k_leader=.15;
    k_predator=0.5;
    k_obstacle=0.5;
    shapeness=0;
    global_rot=0;
    show_frame=1;
    predator_on=1;
    obstacle_on=0;
    boid_trail_on=1;

    // Invoke the standard GLUT main event loop
    glutMainLoop();
    exit(0);         // never reached
}

// Initialize glut and create a window with the specified caption
void initGlut(char* winName)
{
    // Set video mode: double-buffered, color, depth-buffered
    glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    // We will learn about all of these later, for now, what you
    // need to know is that with the settings above, the graphics
    // window will keep track of the depth of objects so that
    // objects in the back can be properly obscured by objects
    // in front of them. The double buffer is used to ensure
    // smooth-looking animation.

    // Create window
    glutInitWindowPosition (0, 0);
    glutInitWindowSize(Win[0],Win[1]);
    windowID = glutCreateWindow(winName);

    // Setup callback functions to handle window-related events.
    // In particular, OpenGL has to be informed of which functions
    // to call when the image needs to be refreshed, and when the
    // image window is being resized.
    glutReshapeFunc(WindowReshape);   // Call WindowReshape whenever window resized
    glutDisplayFunc(WindowDisplay);   // Call WindowDisplay whenever new frame needed
}

// Quit button handler.  Called when the "quit" button is pressed.
void quitButton(int)
{
  if (modelVertices!=NULL && n_vertices>0) free(modelVertices);
  exit(0);
}

// Initialize GLUI and the user interface
void initGlui()
{
    GLUI_Master.set_glutIdleFunc(NULL);

    // Create GLUI window
    glui = GLUI_Master.create_glui("Boid CSC 418 Window", 0, Win[0]+10, 0);

    ////////////////////////////////////////
    // LEARNING OBJECTIVES:
    //
    // This part of the assignment is meant to help
    // you learn about:
    //
    // - How to create simple user interfaces for your
    //   OpenGL programs using GLUI.
    // - Defining and using controllers to change
    //   variables in your code.
    // - Using the GUI to change the behaviour of
    //   your program and the display parameters
    //   used by OpenGL.
    //
    // Be sure to check on-line references for GLUI
    // if you want to learn more about controller
    // types.
    //
    // See: http://www.eng.cam.ac.uk/help/tpl/graphics/using_glui.html
    //
    ////////////////////////////////////////

    ///////////////////////////////////////////////////////////
    // TO DO:
    //   Add controls to change the values of the variables
    //   used in the updateBoid() function. This includes
    //
    //   - k_rule1, k_rule2, k_rule3, k_rule0
    //   - r_rule2, r_rule3
    //
    //   Ranges for these variables are given in the updateBoid()
    //   function. Make sure to set the increments to a reasonable
    //   value.
    //
    //   An example control for r_rule1 is shown below.
    //
    //   In addition to this, add a single control to provide
    //   global rotation aound the vertical axis of the scene
    //   so that we can view the scene from different angles.
    //   The variable for this is called 'global_rot'.
    //
    ///////////////////////////////////////////////////////////

// EXAMPLE control for r_rule1
// Controller name ---|
//                    v
    GLUI_Spinner *r1_spinner
	= glui->add_spinner("r_rule1", GLUI_SPINNER_FLOAT, &r_rule1);
//                           ^               ^               ^
// Text to show -------------|               |               |
// Type of control variable -----------------|               |
// Variable whose value will be changed ---------------------|
//
// Change speed ----------|
//                        v
    r1_spinner->set_speed(5.0);
    r1_spinner->set_float_limits(10, 100, GLUI_LIMIT_CLAMP);
//                                ^  ^
// Min and max values ------------|--|

    /*
    *
    * Start
    *
    */
    //r_rule2 spinner
    GLUI_Spinner *r2_spinner = 
	    glui->add_spinner("r_rule2", GLUI_SPINNER_FLOAT, &r_rule2);
    r2_spinner->set_speed(2.0);
    r2_spinner->set_float_limits(1, 15, GLUI_LIMIT_CLAMP);

    //r_rule3 spinner
    GLUI_Spinner *r3_spinner = 
	    glui->add_spinner("r_rule3", GLUI_SPINNER_FLOAT, &r_rule3);
    r3_spinner->set_speed(5.0);
    r3_spinner->set_float_limits(10, 100, GLUI_LIMIT_CLAMP);
    
    //r_leader spinner
    GLUI_Spinner *rLeader_spinner = 
	    glui->add_spinner("r_leader", GLUI_SPINNER_FLOAT, &r_leader);
    rLeader_spinner->set_speed(5.0);
    rLeader_spinner->set_float_limits(10, 100, GLUI_LIMIT_CLAMP);
    
    //r_predator spinner
    GLUI_Spinner *rpredator_spinner = 
	    glui->add_spinner("r_predator", GLUI_SPINNER_FLOAT, &r_predator);
    rpredator_spinner->set_speed(5.0);
    rpredator_spinner->set_float_limits(10, 100, GLUI_LIMIT_CLAMP);
    
    //Seperator
    glui->add_separator();

    //k_rule0 spinner
    GLUI_Spinner *k0_spinner = 
	    glui->add_spinner("k_rule0", GLUI_SPINNER_FLOAT, &k_rule0);
    k0_spinner->set_speed(1.0);
    k0_spinner->set_float_limits(0, 1, GLUI_LIMIT_CLAMP);

    //k_rule1 spinner
    GLUI_Spinner *k1_spinner = 
	    glui->add_spinner("k_rule1", GLUI_SPINNER_FLOAT, &k_rule1);
    k1_spinner->set_speed(1.0);
    k1_spinner->set_float_limits(0, 1, GLUI_LIMIT_CLAMP);

    //k_rule2 spinner
    GLUI_Spinner *k2_spinner = 
	    glui->add_spinner("k_rule2", GLUI_SPINNER_FLOAT, &k_rule2);
    k2_spinner->set_speed(1.0);
    k2_spinner->set_float_limits(0, 1, GLUI_LIMIT_CLAMP);

    //k_rule3 spinner
    GLUI_Spinner *k3_spinner = 
	    glui->add_spinner("k_rule3", GLUI_SPINNER_FLOAT, &k_rule3);
    k3_spinner->set_speed(1.0);
    k3_spinner->set_float_limits(0, 1, GLUI_LIMIT_CLAMP);
    
    //k_leader spinner
     GLUI_Spinner *kLeader_spinner = 
	    glui->add_spinner("k_leader", GLUI_SPINNER_FLOAT, &k_leader);
    kLeader_spinner->set_speed(1.0);
    kLeader_spinner->set_float_limits(0, 1, GLUI_LIMIT_CLAMP);
    
    //k_predator spinner
     GLUI_Spinner *kpredator_spinner = 
	    glui->add_spinner("k_predator", GLUI_SPINNER_FLOAT, &k_predator);
    kpredator_spinner->set_speed(1.0);
    kpredator_spinner->set_float_limits(0, 1, GLUI_LIMIT_CLAMP);
    
    //k_obstacle spinner
     GLUI_Spinner *kobstacle_spinner = 
	    glui->add_spinner("k_obstacle", GLUI_SPINNER_FLOAT, &k_obstacle);
    kobstacle_spinner->set_speed(1.0);
    kobstacle_spinner->set_float_limits(0, 1, GLUI_LIMIT_CLAMP);

    //Global_Rot
    GLUI_Spinner *global_rot_spinner =
	    glui->add_spinner("global_rot", GLUI_SPINNER_FLOAT, &global_rot);
    global_rot_spinner->set_speed(1.0);
    global_rot_spinner->set_float_limits(0, 360, GLUI_LIMIT_CLAMP);
    
    //Add a toggle to control if the box is shown
    glui->add_separator();
    glui->add_checkbox("show_frame", &show_frame);
    
    //Add a toggle for the predator
    glui->add_separator();
    glui->add_checkbox("predator_on", &predator_on);
    
    //Add a toggle for the obstacle
    glui->add_separator();
    glui->add_checkbox("obstacle_on", &obstacle_on);
    
    //Add a toggle for the obstacle
    glui->add_separator();
    glui->add_checkbox("boid_trail_on", &boid_trail_on);
    /*
    *
    * End
    *
    */
	
    // Add "Quit" button
    glui->add_separator();
    glui->add_button("Quit", 0, quitButton);

    // Set the main window to be the "active" window
    glui->set_main_gfx_window(windowID);
}

/*
  Reshape callback function. Takes care of handling window resizing
  events.
*/
void WindowReshape(int w, int h)
{
    // Setup projection matrix for new window

    // We will learn about projections later on. The projection mode
    // determines how 3D objects are 'projected' onto the 2D image.

    // Most graphical operations in OpenGL are performed through the
    // use of matrices. Below, we let OpenGL know that we will be
    // working with the GL_PROJECTION matrix, which controls the
    // projection of objects onto the image.
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();			// Initialize with identity matrix

    // We will use perspective projection. This simulates a simple
    // pinhole camera.

    // The line below specifies the general shape and properties of
    // the viewing volume, that is, the region of space that is
    // visible within the image window.
    gluPerspective(45,1,15,500);
    //              ^ ^  ^  ^
    // FOV ---------| |  |  |		// See OpenGL reference for
    // Aspect Ratio --|  |  |		// more details on using
    // Near plane -------|  |		// gluPerspective()
    // Far plane -----------|

    // Set the OpenGL viewport - this corresponds to the 2D image
    // window. It uses pixels as units. So the instruction below
    // sets the image window to start at pixel coordinate (0,0)
    // with the specified width and height.
    glViewport(0,0,w,h);
    Win[0] = w;
    Win[1] = h;

    // glutPostRedisplay()	// Is this needed?
}

void GL_Settings_Init()
{
 // Initialize OpenGL parameters to be used throughout the
 // life of the graphics window

    // Set the background colour
    glClearColor(0,0.5,1,1);

    // Enable alpha blending for transparency
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

/**** Illumination set up start ****/

    // The section below controls the illumination of the scene.
    // Nothing can be seen without light, so the section below
    // is quite important. We will discuss different types of
    // illumination and how they affect the appearance of objecs
    // later in the course.
    glClearDepth(1);
    glEnable(GL_DEPTH_TEST);    // Enable depth testing
    glEnable(GL_LIGHTING);      // Enable lighting
    glEnable(GL_LIGHT0);        // Enable LIGHT0 for diffuse illumination
    glEnable(GL_LIGHT1);        // Enable LIGHT1 for ambient illumination

    // Set up light source colour, type, and position
    GLfloat light0_colour[]={1.0,1.0,1.0};
    GLfloat light1_colour[]={.25,.25,.25};
    GLfloat light0_pos[]={500,0,500,0};
    glLightfv(GL_LIGHT0,GL_DIFFUSE,light0_colour);
    glLightfv(GL_LIGHT1,GL_AMBIENT,light1_colour);
    glLightfv(GL_LIGHT0,GL_POSITION,light0_pos);
    glShadeModel(GL_SMOOTH);

    // Enable material colour properties
    glEnable(GL_COLOR_MATERIAL);

/***** Illumination setup end ******/
}

/*
   Display callback function.
   This has to be called whenever the image needs refreshing,
   either as a result of updates to the graphical content of
   the window (e.g. animation is taking place), or as a
   result of events outside this window (e.g. other programs
   may create windows that partially occlude the OpenGL window,
   when the occlusion ends, the obscured region has to be
   refreshed)
*/
void WindowDisplay(void)
{
    // Clear the screen and depth buffers
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

/***** Scene drawing start *********/

    // Here is the section where shapes are actually drawn onto the image.
    // In this case, we call a function to update the positions of boids,
    // and then draw each boid at the updated location.

    // Setup the model-view transformation matrix
    // This is the matrix that determines geometric
    // transformations applied to objects. Typical
    // transformations include rotations, translations,
    // and scaling.
    // Initially, we set this matrix to be the identity
    // matrix.
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // The line below specifies the position and orientation of the
    // camera, as well as the direction it's pointing at.
    // The X and Z location of the camera may vary depending on the value of global_rot
    // This allows the user to control the camera's rotation about the Y axis
    gluLookAt(155*cos(global_rot*PI/180),105,130*sin(global_rot*PI/180),0,0,0,0,1,0);
    // The first three parameters are the camera's X,Y,Z location
    // The next three specify the (x,y,z) position of a point
    // the camera is looking at.
    // The final three parameters specify a vector that indicates
    // what direction is 'up'
    
    if (show_frame == 1) {
      // Draw box bounding the viewing area
      glColor4f(.95,.95,.95,.95);
      glBegin(GL_LINE_LOOP);
      glVertex3f(-50,-50,-50);
      glVertex3f(-50,-50,50);
      glVertex3f(-50,50,50);
      glVertex3f(-50,50,-50);
      glEnd();

      glBegin(GL_LINE_LOOP);
      glVertex3f(-50,-50,-50);
      glVertex3f(-50,-50,50);
      glVertex3f(50,-50,50);
      glVertex3f(50,-50,-50);
      glEnd();

      glBegin(GL_LINE_LOOP);
      glVertex3f(-50,-50,-50);
      glVertex3f(-50,50,-50);
      glVertex3f(50,50,-50);
      glVertex3f(50,-50,-50);
      glEnd();

      glBegin(GL_LINE_LOOP);
      glVertex3f(50,50,50);
      glVertex3f(50,50,-50);
      glVertex3f(50,-50,-50);
      glVertex3f(50,-50,50);
      glEnd();

      glBegin(GL_LINE_LOOP);
      glVertex3f(50,50,50);
      glVertex3f(50,50,-50);
      glVertex3f(-50,50,-50);
      glVertex3f(-50,50,50);
      glEnd();

      glBegin(GL_LINE_LOOP);
      glVertex3f(50,50,50);
      glVertex3f(50,-50,50);
      glVertex3f(-50,-50,50);
      glVertex3f(-50,50,50);
      glEnd();
    }
    
    GLUquadric *new_quad;

    new_quad=gluNewQuadric();
    
    //Obstacle (SPONGEBOB'S HOUSE!!!!!!!!!!!!!)
    if (obstacle_on == 1) {
      glColor4f(1,0.65,0,1);
      
      //cylinder for the pinapple
      glPushMatrix();
	glRotatef(90,1,0,0);
	gluCylinder(new_quad, 15, 15, 15, 30, 30);
      glPopMatrix();
      
      //spere for top of pineapple
      glPushMatrix();
	gluSphere(new_quad, 15, 30, 30);
      glPopMatrix();
      
      glColor4f(0,0.5,0,1);
      //spikes on top of a pineapple
      glPushMatrix();
	glTranslatef(0,15,0);
	glRotatef(-90,1,0,0);
	gluCylinder(new_quad,2,0,12,30,30);
      glPopMatrix();
      glPushMatrix();
	glRotatef(15,1,0,0);
	glTranslatef(0,15,0);
	glRotatef(-90,1,0,0);
	gluCylinder(new_quad,2,0,12,30,30);
      glPopMatrix();
      glPushMatrix();
	glRotatef(-15,1,0,0);
	glTranslatef(0,15,0);
	glRotatef(-90,1,0,0);
	gluCylinder(new_quad,2,0,12,30,30);
      glPopMatrix();
      glPushMatrix();
	glRotatef(15,0,0,1);
	glTranslatef(0,15,0);
	glRotatef(-90,1,0,0);
	gluCylinder(new_quad,2,0,12,30,30);
      glPopMatrix();
      glPushMatrix();
	glRotatef(-15,0,0,1);
	glTranslatef(0,15,0);
	glRotatef(-90,1,0,0);
	gluCylinder(new_quad,2,0,12,30,30);
      glPopMatrix();
      glPushMatrix();
	glRotatef(15,1,0,1);
	glTranslatef(0,15,0);
	glRotatef(-90,1,0,0);
	gluCylinder(new_quad,2,0,12,30,30);
      glPopMatrix();
      glPushMatrix();
	glRotatef(-15,1,0,1);
	glTranslatef(0,15,0);
	glRotatef(-90,1,0,0);
	gluCylinder(new_quad,2,0,12,30,30);
      glPopMatrix();
      glPushMatrix();
	glRotatef(15,1,0,-1);
	glTranslatef(0,15,0);
	glRotatef(-90,1,0,0);
	gluCylinder(new_quad,2,0,12,30,30);
      glPopMatrix();
      glPushMatrix();
	glRotatef(15,-1,0,1);
	glTranslatef(0,15,0);
	glRotatef(-90,1,0,0);
	gluCylinder(new_quad,2,0,12,30,30);
      glPopMatrix();
      
      //draw the door and windows
      glColor4f(0.15,0.15,0.15,1);
      
      //window 1
      glPushMatrix();
	glTranslatef(14,3,-5);
	glRotatef(15,0,1,0);
	glScalef(0.3,1,1);
	gluSphere(new_quad,3,30,30);
      glPopMatrix();
      
      //window 2
      glPushMatrix();
	glTranslatef(14,-3,6);
	glRotatef(-15,0,1,0);
	glScalef(0.3,1,1);
	gluSphere(new_quad,3,30,30);
      glPopMatrix();
      
      //door
      glPushMatrix();
	glTranslatef(15,-6.5,0);
	glRotatef(90,1,0,0);
	glScalef(0.2,1,1);
	gluCylinder(new_quad,3,3,9,30,30);
      glPopMatrix();
    }

    for (int i=0; i<nBoids; i++)
    {
     updateBoid(i);		// Update position and velocity for boid i
     drawBoid(i);		// Draw this boid
    }

    // Make sure all OpenGL commands are executed
    glFlush();

    // Swap buffers to enable smooth animation
    glutSwapBuffers();
/***** Scene drawing end ***********/

  // synchronize variables that GLUT uses
  glui->sync_live();

  // Tell glut window to update itself
  glutSetWindow(windowID);
  glutPostRedisplay();
}

//Determine if two boids are within a certain radius of eachother and return true or false
bool Within_Range (int boidOne, int boidTwo, float radius) {
  //create a vector that holds the x, y, and z distance between the two boids
  float distanceVector[3];
  for (int i = 0; i < 3; i++){
    distanceVector[i] = Boid_Location[boidOne][i] - Boid_Location[boidTwo][i];
  }
  
  //calculate the distance between the two boids and determine if they are within the proper radius
  float distance = sqrt(pow(distanceVector[0], 2) + pow(distanceVector[1], 2) + pow(distanceVector[2], 2));
  if (distance <= radius) {
    return true;
  } else {
    return false;
  }
}

//Determine the distance between two boids
float Distance_Between_Boids (int boidOne, int boidTwo) {
  //create a vector that holds the x, y, and z distance between the two boids
  float distanceVector[3];
  for (int i = 0; i < 3; i++){
    distanceVector[i] = Boid_Location[boidOne][i] - Boid_Location[boidTwo][i];
  }
  
  //calculate the distance between the two boids and determine if they are within the proper radius
  float distance = sqrt(pow(distanceVector[0], 2) + pow(distanceVector[1], 2) + pow(distanceVector[2], 2));
  
  return distance;
}

void updateBoid(int i)
{
  /*
  * 
  * VARIABLES
  * 
  */
  
  //Variables used to find velocity for Boids Rule 1
  float centerOfMass[3] = {0,0,0};
  float velocityRuleOne[3] = {0,0,0};
  int numOfBoids = 0;

  //Variables used to find velocity for Boids Rule 2
  float velocityRuleTwo[3] = {0,0,0};

  //Variables used to find velocity for Boids Rule 3
  float velocityRuleThree[3] = {0,0,0}; 
  float averageSurroundingVel[3] = {0,0,0};
  
  //Variables used to find velocity for Boids Rule 0
  float velocityRuleZero[3] = {0,0,0};
  
  //Variables used to find velocity for Leader Rule
  float velocityLeader[3] = {0,0,0};
  float centerOfLeaders[3] = {0,0,0};
  
  //Variables used to find velocity for Predator Rule
  float velocityPredator[3] = {0,0,0};
  
  //Variables used to find velocity for Obstacle Rule
  float velocityObstacle[3] = {0,0,0};
  float distanceToObstacle;
  
  
  /*
  * 
  * BOIDS RULE 1
  * 
  */

  //compare boid i with everyother boid
  //if the other boid is within the range of boid i, add it's location to the center of mass
  for (int j = 0; j < nBoids; j++) {
    if (i != j) {
      if (Within_Range(i, j, r_rule1)) {
	numOfBoids++;
	for (int k = 0; k < 3; k++){
	  centerOfMass[k] += Boid_Location[j][k];
	}
      }
    }
  }
  
  if (numOfBoids != 0) {
    //determine the center of mass by dividing the sum of the boid positions by the number of boids
    for (int k = 0; k < 3; k++) {
	centerOfMass[k] /= numOfBoids;
    }
    
    //calculate the offset velocity and normalize the movement towards the point
    for (int k = 0; k < 3; k++) {
      velocityRuleOne[k] = (centerOfMass[k] - Boid_Location[i][k]) * k_rule1;
    }
  }

  ///////////////////////////////////////////
  // QUESTION:
  //  Is this the optimal way to implement this
  // rule? can you see any problems or ways
  // to improve this bit?
  ///////////////////////////////////////////
    
  /*
  * 
  * BOIDS RULE 2
  * 
  */
  
  //for every boid j, if it is within range of i modify the velocity to avoid collisions
  for (int j = 0; j < nBoids; j++) {
    if (i != j) {
      if (Within_Range(i, j, r_rule2)) {
	for (int k = 0; k < 3; k++) {
	  velocityRuleTwo[k] -= (Boid_Location[j][k] - Boid_Location[i][k]) * k_rule2;
	}
      }
    }
  }

  /*
  * 
  * BOIDS RULE 3
  * 
  */

  //compare boid i with everyother boid
  //if the other boid is within the range of boid i, add it's velocity to averageSurroundingVel
  numOfBoids = 0;
  
  for (int j = 0; j < nBoids; j++) {
    if (i != j) {
      if (Within_Range(i, j, r_rule3)) {
	numOfBoids++;
	for (int k = 0; k < 3; k++){
	  averageSurroundingVel[k] += Boid_Velocity[j][k];
	}
      }
    }
  }
  
  if (numOfBoids != 0) {
    //determine the average surround velocity by dividing the sum by the num of boids
    for (int k = 0; k < 3; k++) {
	averageSurroundingVel[k] /= numOfBoids;
    }
    
    //calculate the offset velocity and normalize the movement towards the point
    for (int k = 0; k < 3; k++) {
      velocityRuleThree[k] = (averageSurroundingVel[k] - Boid_Velocity[i][k]) * k_rule3;
    }
  }
  
  /*
  * 
  * LEADER RULE
  * 
  */

  numOfBoids = 0;

  for (int j = 0; j < nBoids; j++) {
    if (i != j) {
      for (int l = 0; l < nLeaders; l++) {
	if (j == l && Within_Range(i,j,r_leader)) {
	  numOfBoids++;
	  for (int k = 0; k < 3; k++) {
	    centerOfLeaders[k] += Boid_Location[j][k];
	  }
	}
      }
    }
  }
  
  if (numOfBoids != 0) {
    //determine the center of mass by dividing the sum of the boid positions by the number of boids
    for (int k = 0; k < 3; k++) {
	centerOfLeaders[k] /= numOfBoids;
    }
    
    //calculate the offset velocity and normalize the movement towards the point
    for (int k = 0; k < 3; k++) {
      velocityLeader[k] = (centerOfLeaders[k] - Boid_Location[i][k]) * k_leader;
    }
  }
  
  /*
  * 
  * PREDATOR RULE
  * 
  */
  
  //if the boid is not the predator and is within range of the predator
  //the boid will want to move in the opposite direction of the predator
  if (i != Boid_Predator && predator_on == 1) {
    if (Within_Range(i,Boid_Predator,r_predator)) {
      for (int k = 0; k < 3; k++) {
	velocityPredator[k] -= (Boid_Location[Boid_Predator][k] - Boid_Location[i][k]) * k_predator;
      }
    }
  }
  
  /*
  * 
  * OBSTACLE RULE
  * 
  */
  
  //if the boid is within range of the obstacle, the boid will try to avoid it
  distanceToObstacle = sqrt(pow(Boid_Location[i][0],2) + pow(Boid_Location[i][1],2) + pow(Boid_Location[i][2],2));
  if (distanceToObstacle <= Obstacle_Radius) {
    for (int k = 0; k < 3; k++) {
      velocityObstacle[k] -= (Obstacle_Location[k] - Boid_Location[i][k]) * k_obstacle;
    }
  }
    
 ///////////////////////////////////////////
 // Enforcing bounds on motion
 //
 //  This is already implemented: The goal
 // is to ensure boids won't stray too far
 // from the viewing area.
 //
 //  This is done exactly as described in the
 // reference.
 //
 //  Bounds on the viewing region are
 //
 //  -50 to 50 on each of the X, Y, and Z
 //  directions.
 ///////////////////////////////////////////
 if (Boid_Location[i][0]<-50) Boid_Velocity[i][0]+=1.0;
 if (Boid_Location[i][0]>50) Boid_Velocity[i][0]-=1.0;
 if (Boid_Location[i][1]<-50) Boid_Velocity[i][1]+=1.0;
 if (Boid_Location[i][1]>50) Boid_Velocity[i][1]-=1.0;
 if (Boid_Location[i][2]<-50) Boid_Velocity[i][2]+=1.0;
 if (Boid_Location[i][2]>50) Boid_Velocity[i][2]-=1.0;

 ///////////////////////////////////////////
 // CRUNCHY: Add a 'shapeness' component.
 //  this should give your Boids a tendency
 //  to hover near one of the points of a
 //  3D model imported from file. Evidently
 //  each Boid should hover to a different
 //  point, and you must figure out how to
 //  make the void fly toward that spot
 //  and hover more-or-less around it
 //  (depending on all Boid parameters
 //   for the above rules). 
 //
 //  3D model data is imported for you when 
 //  the user specifies the name of a model
 //  file in .3ds format from the command
 //  line. 
 //
 //  The model data
 //  is stored in the modelVertices array
 //  and the number of vertices is in
 //  n_vertices (if zero, there is no model
 //  and the shapeness component should
 //  have no effect whatsoever)
 //
 //  The coordinates (x,y,z) of the ith
 //  mode, vertex can be accessed with
 //  x=*(modelVertices+(3*i)+0);
 //  y=*(modelVertices+(3*i)+1);
 //  z=*(modelVertices+(3*i)+2);
 //
 //  Evidently, if you try to access more
 //  points than there are in the array
 //  you will get segfault (don't say I
 //  didn't warn you!). Be careful with
 //  indexing.
 //
 //  shapeness should be in [0,1], and
 //  there is already a global variable
 //  to store it. You must add a slider
 //  to the GUI to control the amount of
 //  shapeness (i.e. how strong the shape
 //  constraints affect Boid position).
 //
 //  .3ds models can be found online, you
 //  *must ensure* you are using a freely
 //  distributable model!
 //
 //////////////////////////////////////////

 ///////////////////////////////////////////
 // Velocity Limit:
 //  This is already implemented. The goal
 // is simply to avoid boids shooting off
 // at unrealistic speeds.
 //
 //  You can tweak this part if you like,
 // or you can simply leave it be.
 //
 //  The speed clamping used here was determined
 // 'experimentally', i.e. I tweaked it by hand!
 ///////////////////////////////////////////
 Boid_Velocity[i][0]=sign(Boid_Velocity[i][0])*sqrt(fabs(Boid_Velocity[i][0]));
 Boid_Velocity[i][1]=sign(Boid_Velocity[i][1])*sqrt(fabs(Boid_Velocity[i][1]));
 Boid_Velocity[i][2]=sign(Boid_Velocity[i][2])*sqrt(fabs(Boid_Velocity[i][2]));

 /*
  * 
  * BOIDS RULE 0
  * 
  */
    
    //calculate a portion of the previous velocity that will act as inertia
    //to make the boids movement more natural
    for (int k = 0; k < 3; k++) {
      velocityRuleZero[k] = Boid_Velocity[i][k] * k_rule0;
    }
    
    
 ///////////////////////////////////////////
 // QUESTION: Why add inertia at the end and
 //  not at the beginning?
 ///////////////////////////////////////////
 
 /*
  * 
  * UPDATE VELOCITY AND POSITION
  * 
  */

    //update the velocity for boid i by using rules 1, 2, and 3
    for (int k = 0; k < 3; k++) {
      Boid_Velocity[i][k] += velocityRuleOne[k] + velocityRuleTwo[k] + velocityRuleThree[k]
      + velocityRuleZero[k] + velocityLeader[k] + velocityPredator[k] + velocityObstacle[k];
    }
 
    //Use boid velocity to update the position of the boid
    for (int k = 0; k < 3; k++) {
      if (i == Boid_Predator && predator_on == 1) {
	Boid_Location[i][k] += Boid_Velocity[i][k] * PREDATOR_SPEED;
      } else {
	Boid_Location[i][k] += Boid_Velocity[i][k] * ADJUST_SPEED;
      }
    }
    
    //Update the previous boid location array
    //Rotate the array one element to the left
    for (int j = 0; j < 99; j++) {
      for (int k = 0; k < 3; k++) {
	Boid_Previous_Locations[i][j][k] = Boid_Previous_Locations[i][j+1][k];
      }
    }
    //insert the newest position
    for (int k = 0; k < 3; k++) {
      Boid_Previous_Locations[i][99][k] = Boid_Location[i][k];
    }
    
 ///////////////////////////////////////////
 // CRUNCHY:
 //
 //  Things you can add here to make the behaviour
 // more interesting. Be sure to note in your
 // report any extra work you have done.
 //
 // - Add a few obstacles (boxes or something like it)
 //   and add code to have boids avoid these
 //   obstacles
 //
 // - Follow the leader: Select a handful
 //   (1 to 5) boids randomly. Add code so that
 //   nearby boids tend to move toward these
 //   'leaders'
 //
 // - Make the updates smoother: Idea, instead
 //   of having hard thresholds on distances for
 //   the update computations (r_rule1, r_rule2,
 //   r_rule3), use a weighted computation
 //   where contributions are weighted by
 //   distance and the weight decays as a
 //   function of the corresponding r_rule
 //   parameter.
 //
 // - Add a few 'predatory boids'. Select
 //   a couple of boids randomly. These become
 //   predators and the rest of the boids
 //   should have a strong tendency to
 //   avoid them. The predatory boids should
 //   follow the standard rules. However,
 //   Be sure to plot the predatory boids
 //   differently so we can easily see
 //   who they are.
 //
 // - Make it go FAST. Consider and implement
 //   ways to speed-up the boid update. Hint:
 //   good approximations are often enough
 //   to give the right visual impression.
 //   What and how to approximate? that is the
 //   problem.
 //
 //   Thoroughly describe any crunchy stuff in
 //   the REPORT.
 //
 ///////////////////////////////////////////

 return;
}

//returns the angle that the boid is flying at with respect to the Y-axis
float Angle_Of_Rotation (float x, float z) {
  //normalize the (x,z) vector that is passed in
  float angle;
  float magnitude = sqrt(pow(x,2) + pow(z,2));
  if (magnitude != 0) {
    x /= magnitude;
    z /= magnitude;
  }
  
  //calculate the angle depending on what quadrant the boid is in
  if (x == 0 && z == 0) {
    return -1;
  } else if (x >= 0 && z >= 0) {
    angle = acos(x);
  } else if (x <= 0 && z >= 0) {
    float alpha = asin(z);
    angle = 2*PI - alpha;
  } else if (x <= 0 && z <= 0) {
    float beta = atan(z/x);
    angle = PI + beta;
  } else {
    float gama = acos(x);
    angle = PI - gama;
  }
  
  //return the angle in degrees
  return angle*180/PI;
}

void drawBoid(int i)
{
  /*
    This function draws a boid i at the specified location.
  */
  GLUquadric *my_quad;	// Define a quadric() object
			// We'll talk about quadrics
			// later! for now suffice it
			// to say they are a family
			// of parametric surfaces
			// whose shape can be controlled
			// by changing a couple parameters.
			// Among the things you can do
			// with quadrics are spheres,
			// discs, and cylinders.

  my_quad=gluNewQuadric();	// Create a new quadric
  
  //automatically set the color of the boid to blue unless boid is predator
  if (i == Boid_Predator && predator_on == 1) {
    glColor4f(1,0,0,1);
  } else {
    glColor4f(0,0,1,1);
  }
  
  //if the boid is a leader, set the color to green
  for (int j = 0; j < nLeaders; j++) {
    if (i == j) {
      glColor4f(0,1,0,1);
      break;
    }
  }
  
  glPushMatrix();	// Save current transformation matrix
  
    //create a trail behind the boid
    if (boid_trail_on == 1) {
      glPushMatrix();
	glBegin(GL_LINES);
	for (int j = 0; j < 100; j++) {
	  glVertex3f(Boid_Previous_Locations[i][j][0],Boid_Previous_Locations[i][j][1],Boid_Previous_Locations[i][j][2]);
	}
	glEnd();
      glPopMatrix();
    }

    // Apply necessary transformations to this boid
    glPushMatrix();
      
      glTranslatef(Boid_Location[i][0],Boid_Location[i][1],Boid_Location[i][2]);
      
      if (Boid_Velocity[i][2] > 0) {
	gluLookAt(0,0,0,Boid_Velocity[i][0],Boid_Velocity[i][1],-Boid_Velocity[i][2],0,1,0);
      } else {
	gluLookAt(0,0,0,Boid_Velocity[i][0],-Boid_Velocity[i][1],-Boid_Velocity[i][2],0,1,0);
      }
      
      if (i == Boid_Predator && predator_on == 1) {
	glScalef(0.5,0.5,0.5);
      } else {
	glScalef(0.2,0.2,0.2);
      }
      
      // Draw this boid
      //Body
      glPushMatrix();
	glScalef(0.7,1,2);
	gluSphere(my_quad,4,10,10);
      glPopMatrix();
      
      //top fin
      glPushMatrix();
	glTranslatef(0,3,0);
	glScalef(0.2,1,1);
	gluSphere(my_quad,3,10,10);
      glPopMatrix();
      
      //right fin
      glPushMatrix();
	glTranslatef(3,0,0);
	glRotatef(-45,0,1,0);
	glScalef(0.2,1,1.5);
	gluSphere(my_quad,2,10,10);
      glPopMatrix();
      
      //left fin
      glPushMatrix();
	glTranslatef(-3,0,0);
	glRotatef(45,0,1,0);
	glScalef(0.2,1,1.5);
	gluSphere(my_quad,2,10,10);
      glPopMatrix();
      
      //tail final
      glPushMatrix();
	glTranslatef(0,0,-10);
	glScalef(0.2,1,2);
	gluCylinder(my_quad,3,1,3,10,10);
      glPopMatrix();
      
      //eyes
      glPushMatrix();
	glColor4f(1,1,1,1);
	glTranslatef(1,0.2,7);
	gluSphere(my_quad,0.5,10,10);
	glTranslatef(-2,0,0);
	gluSphere(my_quad,0.5,10,10);
      glPopMatrix();

    glPopMatrix();
    
  glPopMatrix();	

  ///////////////////////////////////////////
  // CRUNCHY:
  //
  //  Animate boids: You can draw your boids
  // differently on each frame to provide
  // animation. For example, birds may flap
  // their wings as they move, fish may
  // move their fins. Do something appropriate
  // that looks good for extra credit.
  //
  ///////////////////////////////////////////
}

void HSV2RGB(float H, float S, float V, float *R, float *G, float *B)
{
 // Handy function to convert a colour specified as an HSV triplet
 // to RGB values used by OpenGL. You can use this function to
 // set the boids' colours in a more intuitive way. To learn
 // about HSV colourspace, check the Wikipedia page.
 float c,x,hp,r1,g1,b1,m;

 hp=H*6;
 c=V*S;
 x=c*(1.0-fabs(fmod(hp,2)-1.0));
 if (hp<1){r1=c;g1=x;b1=0;}
 else if (hp<2){r1=x;g1=c;b1=0;}
 else if (hp<3){r1=0;g1=c;b1=x;}
 else if (hp<4){r1=0;g1=x;b1=c;}
 else if (hp<5){r1=x;g1=0;b1=c;}
 else{r1=c;g1=0;b1=x;}

 m=V-c;
 *R=r1+m;
 *G=g1+m;
 *B=b1+m;

 if (*R>1) *R=1;
 if (*R<0) *R=0;
 if (*G>1) *G=1;
 if (*G<0) *G=0;
 if (*B>1) *B=1;
 if (*B<0) *B=0;
}

float *read3ds(const char *name, int *n)
{
 /*
   Read a model in .3ds format from the specified file.
   If the model is read successfully, a pointer to a
   float array containing the vertex coordinates is
   returned.

   Input parameter n is used to specify the maximum
   number of vertex coordinates to return, as well
   as to return the actual number of vertices read.


   Vertex coordinates are stored consecutively
   so each vertex occupies in effect 3 consecutive
   floating point values in the returned array
 */
 Lib3dsFile *f;
 Lib3dsMesh *mesh;
 Lib3dsPoint *pt;
 int n_meshes, n_points;
 Lib3dsVector *vertex_data;
 float *vertices, *v_return;
 float inc_step;
 int idx;

 f=lib3ds_file_load(name);
 if (f==NULL)
 {
  fprintf(stderr,"Unable to load model data\n");
  *n=0;
//   return(NULL);
 }

 // Count meshes and faces
 n_points=0;
 n_meshes=0;
 mesh=f->meshes;
 while(mesh!=NULL)
 {
  n_meshes++;
  n_points+=mesh->points;
  mesh=mesh->next;
 }
 fprintf(stderr,"Model contains %d meshes, %d points, %d coordinates\n",n_meshes,n_points,3*n_points);

 // Allocate data for vertex array and put all input points (from all meshes) in the array
 vertex_data=(Lib3dsVector *)calloc(n_points,sizeof(Lib3dsVector));
 mesh=f->meshes;
 while(mesh!=NULL)
 {
  pt=mesh->pointL;
  for (int i=0; i < mesh->points; i++)
    memcpy((vertex_data+i),(pt+i),sizeof(Lib3dsVector));
  mesh=mesh->next;
 }

 vertices=(float *)vertex_data;
 // Release memory allocated to the file data structure and return the vertex array
 lib3ds_file_free(f);

 if (n_points<(*n)) *(n)=n_points;                      // Less points than expected!
 v_return=(float *)calloc((*(n))*3,sizeof(float));      // Allocate space for n points
 inc_step=(n_points-1)/(*n);                            // Sampling step
 for (int i=0; i<(*(n)); i++)
 {
  idx=floor(inc_step*i);
  *(v_return+(3*i)+0)=*(vertices+(3*idx)+0);            // Mind the ordering! it's shuffled
  *(v_return+(3*i)+1)=*(vertices+(3*idx)+2);            // to match our coordinate frame
  *(v_return+(3*i)+2)=*(vertices+(3*idx)+1);
 }

 free(vertex_data);
 return(v_return);
}


