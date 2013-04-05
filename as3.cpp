//
//  as3.cpp
//
//
//  Created by Matthew Visco on 3/19/13.
//
//


#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>

#ifdef _WIN32
#include <windows.h>
#else
#include <sys/time.h>
#endif

#ifdef OSX
#include <GLUT/glut.h>
#include <OpenGL/glu.h>
#else
#include <GL/glut.h>
#include <GL/glu.h>
#endif

#include "glm/glm.hpp"
#include <time.h>
#include <math.h>

#define PI 3.14159265
#define epsilon .0001

inline float sqr(float x) { return x*x; }

using namespace std;
using namespace glm;

//****************************************************
// Some Classes
//****************************************************

class Viewport;
class Patch;
class Curve;

class Viewport {
public:
    int w, h; // width and height
    
};

class Curve {
public:
    vec3 p0, p1, p2, p3;
    Curve(vec3 p0, vec3 p1, vec3 p2, vec3 p3) {Curve::p0 = p0; Curve::p1 = p1; Curve::p2 = p2; Curve::p3 = p3; }
    Curve() {}
};

class Patch {
public:
    Curve v0, v1, v2, v3, u0, u1, u2, u3;
    Patch() {}
    Patch(Curve v0, Curve v1, Curve v2, Curve v3, Curve u0, Curve u1, Curve u2, Curve u3) {Patch::v0 = v0; Patch::v1 = v1; Patch::v2 = v2; Patch::v3 = v3; Patch::u0 = u0; Patch::u1 = u1; Patch::u2 = u2; Patch::u3 = u3;}
};



//****************************************************
// Global Variables
//****************************************************
Viewport viewport;
vector<Patch> patches;
int bezStep; // change where this gets set
int patchSize; // set this in the parser
bool adaptive;
bool lines=true;
bool smooth=true;
float tolerance;

// angle of rotation for the object
float angleX = 0.0, angleY = 0, transX = 0, transY = 0;
// actual vector representing the camera's direction
float lx=0.0f,lz=-1.0f;
// XZ position of the camera
float x=0.0f,z=11.0f;
GLfloat light_position[4] = {0,0,-1,0}, light_ambient[4] = {0.0, 0.0, 0.0, 1.0}, light_diffuse[4] = {1.0, 1.0, 1.0, 1.0}, light_specular[4] = {1.0, 1.0, 1.0, 1.0};
float mcolor[] = { 1.0f, 0.5f, 0.0f, 1.0f };
float zcolor[]= { 0.0f, 1.0f, 0.0f, 1.0f };
float specReflection[] = { 0.8f, 0.8f, 0.8f, 1.0f };


//****************************************************
// Simple init function
//****************************************************
void initScene(){
    glClearColor (0.0, 0.0, 0.0, 0.0);
    glClearDepth(1.0);
    
    // Enable lighting and the light we have set up
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_LIGHT1);
    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE,0);
    glEnable(GL_DEPTH_TEST);
    
    // Set lighting parameters
    glLightfv(GL_LIGHT1,GL_POSITION,light_position);
    glLightfv(GL_LIGHT1, GL_AMBIENT,light_ambient);
    glLightfv(GL_LIGHT1, GL_DIFFUSE, light_diffuse);
    glLightfv(GL_LIGHT1, GL_SPECULAR, light_specular);

}


//****************************************************
// reshape viewport if the window is resized
//****************************************************
void myReshape(int w, int h) {
    viewport.w = w;
    viewport.h = h;
    float ratio =  w * 1.0 / h;
    
    // Use the Projection Matrix
	glMatrixMode(GL_PROJECTION);
    
    // Reset Matrix
	glLoadIdentity();
    
	// Set the viewport to be the entire window
	glViewport(0, 0, viewport.w, viewport.h);
    
	// Set the correct perspective.
	gluPerspective(45,ratio,1,100);
    
	// Get Back to the Modelview
	glMatrixMode(GL_MODELVIEW);

}



//****************************************************
// given the control points of a bezier curve and a
// parametric value, return the curve point and derivative
//****************************************************
void bezcurveinterp(Curve curve, float u, vec3& point, vec3& dPdu) {
    // first, split each of the three segments
    // to form two new ones AB and BC
    vec3 A,B,C,D,E;
    float uT = 1.0-u;
    
    A = curve.p0 * uT + curve.p1 * u;
    B = curve.p1 * uT + curve.p2 * u;
    C = curve.p2 * uT + curve.p3 * u;
    
    
    // now, split AB and BC to form a new segment DE
    D = A * uT + B * u;
    E = B * uT + C * u;
    
    // finally, pick the right point on DE,
    // this is the point on the curve
    point = D * uT + E * u;
    
    // compute derivative also
    dPdu = 3.0f * (E - D);
}


//****************************************************
// Given a control patch and (u,v) values,
// find the surface point and normal
//***************************************************
void bezpatchinterp(Patch patch, float u, float v, vec3& point, vec3& normal) {
    Curve vcurve, ucurve;
    vec3 p, dPdv, dPdu;
    
    
    //build control points for a Bezier curve in v
    bezcurveinterp(patch.u0, u, (vcurve.p0), dPdv);
    bezcurveinterp(patch.u1, u, (vcurve.p1), dPdv);
    bezcurveinterp(patch.u2, u, (vcurve.p2), dPdv);
    bezcurveinterp(patch.u3, u, (vcurve.p3), dPdv);

    
    //build control points for a Bezier curve in u
    bezcurveinterp(patch.v0, v, (ucurve.p0), dPdu);
    bezcurveinterp(patch.v1, v, (ucurve.p1), dPdu);
    bezcurveinterp(patch.v2, v, (ucurve.p2), dPdu);
    bezcurveinterp(patch.v3, v, (ucurve.p3), dPdu);
    
    //evaluate surface and derivative for u and v
    bezcurveinterp(vcurve, v, point, dPdv);
    bezcurveinterp(ucurve, u, point, dPdu);
    
    normal=normalize(cross(dPdu,dPdv));
}

void adaptiveTes(vec3 firstpoint, vec3 secondpoint, vec3 thirdpoint, vec3 firstnormal, vec3 secondnormal, vec3 thirdnormal, float u1, float v1, float u2, float v2, float u3, float v3, Patch patch, int recursion){
    vec3 point1, point2, point3, normal1, normal2, normal3;
    vec3 midpoint1((firstpoint.x+secondpoint.x)/2,(firstpoint.y+secondpoint.y)/2,(firstpoint.z+secondpoint.z)/2);
    bezpatchinterp(patch, (u1+u2)/2, (v1+v2)/2, point1, normal1);
    
    vec3 midpoint2((secondpoint.x+thirdpoint.x)/2,(secondpoint.y+thirdpoint.y)/2,(secondpoint.z+thirdpoint.z)/2);
    bezpatchinterp(patch, (u2+u3)/2, (v2+v3)/2, point2, normal2);
    
    vec3 midpoint3((thirdpoint.x+firstpoint.x)/2,(thirdpoint.y+firstpoint.y)/2,(thirdpoint.z+firstpoint.z)/2);
    bezpatchinterp(patch, (u3+u1)/2, (v3+v1)/2, point3, normal3);
    
    float diff1=sqrt(sqr(point1.x-midpoint1.x)+sqr(point1.y-midpoint1.y)+sqr(point1.z-midpoint1.z));
    float diff2=sqrt(sqr(point2.x-midpoint2.x)+sqr(point2.y-midpoint2.y)+sqr(point2.z-midpoint2.z));
    float diff3=sqrt(sqr(point3.x-midpoint3.x)+sqr(point3.y-midpoint3.y)+sqr(point3.z-midpoint3.z));
    
    // case when all sides are close enough
    if ((sqrt(sqr(point1.x-midpoint1.x)+sqr(point1.y-midpoint1.y)+sqr(point1.z-midpoint1.z))<tolerance && sqrt(sqr(point2.x-midpoint2.x)+sqr(point2.y-midpoint2.y)+sqr(point2.z-midpoint2.z))<tolerance && sqrt(sqr(point3.x-midpoint3.x)+sqr(point3.y-midpoint3.y)+sqr(point3.z-midpoint3.z))<tolerance) || recursion==0){
        if (lines){
            glNormal3f(firstnormal.x, firstnormal.y, firstnormal.z);
            glVertex3f(firstpoint.x, firstpoint.y, firstpoint.z);
            glNormal3f(secondnormal.x, secondnormal.y, secondnormal.z);
            glVertex3f(secondpoint.x, secondpoint.y, secondpoint.z);
            
            glNormal3f(secondnormal.x, secondnormal.y, secondnormal.z);
            glVertex3f(secondpoint.x, secondpoint.y, secondpoint.z);
            glNormal3f(thirdnormal.x, thirdnormal.y, thirdnormal.z);
            glVertex3f(thirdpoint.x, thirdpoint.y, thirdpoint.z);
            
            glNormal3f(thirdnormal.x, thirdnormal.y, thirdnormal.z);
            glVertex3f(thirdpoint.x, thirdpoint.y, thirdpoint.z);
            glNormal3f(firstnormal.x, firstnormal.y, firstnormal.z);
            glVertex3f(firstpoint.x, firstpoint.y, firstpoint.z);
        } else {
            glNormal3f(firstnormal.x, firstnormal.y, firstnormal.z);
            glVertex3f(firstpoint.x, firstpoint.y, firstpoint.z);
            glNormal3f(secondnormal.x, secondnormal.y, secondnormal.z);
            glVertex3f(secondpoint.x, secondpoint.y, secondpoint.z);
            glNormal3f(thirdnormal.x, thirdnormal.y, thirdnormal.z);
            glVertex3f(thirdpoint.x, thirdpoint.y, thirdpoint.z);
        }
    } else if (diff1>=tolerance && diff2<tolerance && diff3<tolerance){
        adaptiveTes(firstpoint, point1, thirdpoint, firstnormal, normal1, thirdnormal, u1, v1, (u1+u2)/2, (v1+v2)/2, u3, v3, patch, recursion-1);
        adaptiveTes(point1, secondpoint, thirdpoint, normal1, secondnormal, thirdnormal, (u1+u2)/2, (v1+v2)/2, u2, v2, u3, v3, patch, recursion-1);
    
    } else if (diff1<tolerance && diff2>=tolerance && diff3<tolerance){
        adaptiveTes(firstpoint, secondpoint, point2, firstnormal, secondnormal, normal2, u1, v1, u2, v2, (u2+u3)/2, (v2+v3)/2, patch, recursion-1);
        adaptiveTes(firstpoint, point2, thirdpoint, firstnormal, normal2, thirdnormal, u1, v1, (u2+u3)/2, (v2+v3)/2, u3, v3, patch, recursion-1);
    
    } else if (diff1<tolerance && diff2<tolerance && diff3>=tolerance){
        adaptiveTes(firstpoint, secondpoint, point3, firstnormal, secondnormal, normal3, u1, v1, u2, v2, (u3+u1)/2, (v3+v1)/2, patch, recursion-1);
        adaptiveTes(point3, secondpoint, thirdpoint, normal3, secondnormal, thirdnormal, (u3+u1)/2, (v3+v1)/2, u2, v2, u3, v3, patch, recursion-1);
    
    } else if (diff1>=tolerance && diff2>=tolerance && diff3<tolerance){
        adaptiveTes(firstpoint, point1, thirdpoint, firstnormal, normal1, thirdnormal, u1, v1, (u1+u2)/2, (v1+v2)/2, u3, v3, patch, recursion-1);
        adaptiveTes(point1, point2, thirdpoint, normal1, normal2, thirdnormal, (u1+u2)/2, (v1+v2)/2, (u2+u3)/2, (v2+v3)/2, u3, v3, patch, recursion-1);
        adaptiveTes(point1, secondpoint, point2, normal1, secondnormal, normal2, (u1+u2)/2, (v1+v2)/2, u2, v2, (u2+u3)/2, (v2+v3)/2, patch, recursion-1);
    
    } else if (diff1>=tolerance && diff2<tolerance && diff3>=tolerance){
        adaptiveTes(firstpoint, point1, point3, firstnormal, normal1, normal3, u1, v1, (u1+u2)/2, (v1+v2)/2, (u3+u1)/2, (v3+v1)/2, patch, recursion-1);
        adaptiveTes(point3, point1, thirdpoint, normal3, normal1, thirdnormal, (u3+u1)/2, (v3+v1)/2, (u1+u2)/2, (v1+v2)/2, u3, v3, patch, recursion-1);
        adaptiveTes(point1, secondpoint, thirdpoint, normal1, secondnormal, thirdnormal, (u1+u2)/2, (v1+v2)/2, u2, v2, u3, v3, patch, recursion-1);
    
    } else if (diff1<tolerance && diff2>=tolerance && diff3>=tolerance){
        adaptiveTes(firstpoint, point2, point3, firstnormal, normal2, normal3, u1, v1, (u2+u3)/2, (v2+v3)/2, (u3+u1)/2, (v3+v1)/2, patch, recursion-1);
        adaptiveTes(firstpoint, secondpoint, point2, firstnormal, secondnormal, normal2, u1, v1, u2, v2, (u2+u3)/2, (v2+v3)/2, patch, recursion-1);
        adaptiveTes(point3, point2, thirdpoint, normal3, normal2, thirdnormal, (u3+u1)/2, (v3+v1)/2, (u2+u3)/2, (v2+v3)/2, u3, v3, patch, recursion-1);
    
    } else {
        adaptiveTes(firstpoint, point1, point3, firstnormal, normal1, normal3, u1, v1, (u1+u2)/2, (v1+v2)/2, (u3+u1)/2, (v3+v1)/2, patch, recursion-1);
        adaptiveTes(point1, secondpoint, point2, normal1, secondnormal, normal2, (u1+u2)/2, (v1+v2)/2, u2, v2, (u2+u3)/2, (v2+v3)/2, patch, recursion-1);
        adaptiveTes(point3, point1, point2, normal3, normal1, normal2, (u3+u1)/2, (v3+v1)/2, (u1+u2)/2, (v1+v2)/2, (u2+u3)/2, (v2+v3)/2, patch, recursion-1);
        adaptiveTes(point3, point2, thirdpoint, normal3, normal2, thirdnormal, (u3+u1)/2, (v3+v1)/2, (u2+u3)/2, (v2+v3)/2, u3, v3, patch, recursion-1);
    }
    
}

//****************************************************
// given a patch, perform uniform subdivision compute how
// many subdivisions there are for this step size
//***************************************************
void subdividepatch(Patch patch, int step) {
    // make sure for loops hit iu = 1 and iv = 1
    float numdiv = ((1 + epsilon) / step);
    vec3 point, normal;
    float iu,iv;
    int u = 0,v = 0;
    vector<vector<vec3 > > points;
    vector<vector<vec3 > > normals;
    
    points.resize(step+1);
    normals.resize(step+1);
    for(int s = 0; s < step+1; ++s)
    {
        //Grow Columns by step
        points[s].resize(step+1);
        normals[s].resize(step+1);
    }
    
    //for each parametric value of iu
    for (iv = 0; iv <= 1; iv += numdiv) {
        //for each parametric value of iv
        for (iu = 0; iu <= 1; iu += numdiv) {
            
            //evaluate surface
            bezpatchinterp(patch, iu, iv, point, normal);
            
            points[v][u]=point;
            normals[v][u]=normal;
            u++;
        }
        bezpatchinterp(patch, 1, iv, point, normal);
        points[v][u]=point;
        normals[v][u]=normal;

        u=0;
        v++;
    }
    for (iu = 0; iu <= 1; iu += numdiv) {
        
        //evaluate surface
        bezpatchinterp(patch, iu, 1, point, normal);
        points[v][u]=point;
        normals[v][u]=normal;
        u++;
    }
    bezpatchinterp(patch, 1, 1, point, normal);
    points[v][u]=point;
    normals[v][u]=normal;
    
    // Renders the patch using the points calculated via interpolation
    if (smooth){
        glShadeModel(GL_SMOOTH);
    } else {
        glShadeModel(GL_FLAT);
    }
    glPushMatrix();
    glTranslatef(transX, transY, 0);
    glRotatef(angleX, 1, 0, 0);
    glRotatef(angleY, 0, 1, 0);
    if (lines){
        glBegin(GL_LINES);
    } else {
        glBegin(GL_TRIANGLES);
    }
    for (int k = 0; k < step; k++) {
        for (int r = 0; r < step; r++) {
            if (!adaptive) {
                //BOTTOM TRIANGLE
                glNormal3f(normals[k+1][r].x,normals[k+1][r].y,normals[k+1][r].z);
                glVertex3f(points[k+1][r].x,points[k+1][r].y,points[k+1][r].z);
            
                glNormal3f(normals[k+1][r+1].x,normals[k+1][r+1].y,normals[k+1][r+1].z);
                glVertex3f(points[k+1][r+1].x,points[k+1][r+1].y,points[k+1][r+1].z);
                
                glNormal3f(normals[k][r].x,normals[k][r].y,normals[k][r].z);
                glVertex3f(points[k][r].x,points[k][r].y,points[k][r].z);
                //if (!lines){
                    //TOP TRIANGLE
                    glNormal3f(normals[k+1][r+1].x,normals[k+1][r+1].y,normals[k+1][r+1].z);
                    glVertex3f(points[k+1][r+1].x,points[k+1][r+1].y,points[k+1][r+1].z);
                
                    glNormal3f(normals[k][r+1].x,normals[k][r+1].y,normals[k][r+1].z);
                    glVertex3f(points[k][r+1].x,points[k][r+1].y,points[k][r+1].z);

                
                    glNormal3f(normals[k][r].x,normals[k][r].y,normals[k][r].z);
                    glVertex3f(points[k][r].x,points[k][r].y,points[k][r].z);
                //} else {
                    //glNormal3f(normals[k+1][r].x,normals[k+1][r].y,normals[k+1][r].z);
                    //glVertex3f(points[k+1][r].x,points[k+1][r].y,points[k+1][r].z);
                //}
                
            } else {
                adaptiveTes(points[k][r],points[k+1][r],points[k+1][r+1],  normals[k][r], normals[k+1][r], normals[k+1][r+1], r*numdiv, k*numdiv, r*numdiv, (k+1)*numdiv,(r+1)*numdiv, (k+1)*numdiv, patch, 400);
                adaptiveTes(points[k][r+1], points[k+1][r+1], points[k][r], normals[k][r+1], normals[k+1][r+1], normals[k][r], (r+1)*numdiv, k*numdiv, (r+1)*numdiv, (k+1)*numdiv, r*numdiv, k*numdiv, patch,400);
            }
        }
    }
    glEnd();
    glPopMatrix();
}


    



//****************************************************
// function that does the actual drawing of stuff
//***************************************************
void myDisplay(void) {
    
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // Clear Color and Depth Buffers
    glLoadIdentity ();     // make sure transformation is "zero'd"
    
    gluLookAt(x, 1.0f, z, x+lx, 1.0f,  z+lz, 0.0f, 1.0f,  0.0f);
    
    // Sets the material properties of the teapot
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, mcolor);
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specReflection);
    glMateriali(GL_FRONT_AND_BACK, GL_SHININESS, 96);
    if (!adaptive){
        bezStep=1/tolerance; // Set this in main
    } else {
        bezStep=1;
    }
    
    //iterate through all the patches and render each patch individually
    
    for (int i = 0; i < patches.size(); i++) {
        subdividepatch(patches[i],bezStep);
    }

    glFlush();
    glutSwapBuffers();					// swap buffers (we earlier set double buffer)
}

// Parzer bezier files -- makes patch and curve objects from the file
void parseFile(string file) {
    bool init = false;
    int lineNum = 0;
    Patch patch;
    Curve v0,v1,v2,v3;
    
    ifstream inpfile(file.c_str());
    if(!inpfile.is_open()) {
        cout << "Unable to open file" << endl;
    } else {
        string line;
        while(inpfile.good()) {
            vector<string> splitline;
            string buf;
            getline(inpfile,line);
            stringstream ss(line);
            while (ss >> buf) {
                splitline.push_back(buf);
            }
            //Ignore blank lines
            if(splitline.size() == 0) {
                continue;
            } else if (!init) {
                bezStep = atoi(splitline[0].c_str()); //this isn't actually the step size, ignore it
                lineNum = 1;
                init = true;
            } else {
                vec3 p0(atof(splitline[0].c_str()),atof(splitline[1].c_str()),atof(splitline[2].c_str()));
                vec3 p1(atof(splitline[3].c_str()),atof(splitline[4].c_str()),atof(splitline[5].c_str()));
                vec3 p2(atof(splitline[6].c_str()),atof(splitline[7].c_str()),atof(splitline[8].c_str()));
                vec3 p3(atof(splitline[9].c_str()),atof(splitline[10].c_str()),atof(splitline[11].c_str()));
                Curve tmpU(p0,p1,p2,p3);
                
                // Based on which line number is being parsed for the current patch this switch statement
                // sets appropriate v and u curves to given control points
                switch (lineNum) {
                    case 1:
                        patch.u0 = tmpU;
                        v0.p0 = p0;
                        v1.p0 = p1;
                        v2.p0 = p2;
                        v3.p0 = p3;
                        lineNum++;
                        break;
                    case 2:
                        patch.u1 = tmpU;
                        v0.p1 = p0;
                        v1.p1 = p1;
                        v2.p1 = p2;
                        v3.p1 = p3;
                        lineNum++;
                        break;
                    case 3:
                        patch.u2 = tmpU;
                        v0.p2 = p0;
                        v1.p2 = p1;
                        v2.p2 = p2;
                        v3.p2 = p3;
                        lineNum++;
                        break;
                    case 4:
                        patch.u3 = tmpU;
                        v0.p3 = p0;
                        v1.p3 = p1;
                        v2.p3 = p2;
                        v3.p3 = p3;
                        patch.v0=v0;
                        patch.v1=v1;
                        patch.v2=v2;
                        patch.v3=v3;
                        patches.push_back(patch);
                        lineNum = 1;
                        break;
                    default:
                        lineNum = 1;
                        break;
                }
            }
        }
        inpfile.close();
    }
}

// Function that assigns zoom amount to +/-
void processNormalKeys(unsigned char key, int x, int y) {
    float fraction = 0.5f;
	if (key == 32) {
		exit(0);
    } else if (key == 43) {
        //x += lx * fraction; // remove global x, unneccessary 
        z += lz * fraction;
    } else if (key == 45) {
        //x -= lx * fraction;
        z -= lz * fraction;
    } else if (key == 119){
        lines=!lines;
    } else if (key == 115){
        smooth=!smooth;
    }
}

// Function that assigns rotation and transformation to directional keys
void processSpecialKeys(int key, int xx, int yy) {
    
	float fraction = 0.1f;
    int mod = glutGetModifiers();
    
	switch (key) {
		case GLUT_KEY_LEFT :
            if (mod == GLUT_ACTIVE_SHIFT){
                transX -= 0.5f;
            } else {
                angleY -= 5.0f;
            }
			break;
		case GLUT_KEY_RIGHT :
            if (mod == GLUT_ACTIVE_SHIFT){
                transX += 0.5f;
            } else {
                angleY += 5.0f;
			}
            break;
		case GLUT_KEY_UP :
            if (mod == GLUT_ACTIVE_SHIFT){
                transY += 0.5f;
            } else {
                angleX += 5.0f;
            }
			break;
		case GLUT_KEY_DOWN :
            if (mod == GLUT_ACTIVE_SHIFT){
                transY -= 0.5f;
            } else {
                angleX -= 5.0f;
            }
			break;
	}
}

int main(int argc, char *argv[]) {
    if (argc!=4){
        printf("IMPROPER INPUTS: FILE, STEPSIZE/TOLERANCE, UNIFORM/ADAPTIVE");
        exit(0);
    }
    string str(argv[1]);
    parseFile(str);
    if (strncmp(argv[3],"-a",2)==0){
        adaptive=true;
    } else {
        adaptive=false;
    }
    tolerance=atof(argv[2]);
  
    
    //This initializes glut
    glutInit(&argc, argv);
    
    //This tells glut to use a double-buffered window with red, green, and blue channels
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    
    // Initalize theviewport size
    viewport.w = 400;
    viewport.h = 400;
    
    //The size and position of the window
    glutInitWindowSize(viewport.w, viewport.h);
    glutInitWindowPosition(0,0);
    glutCreateWindow(argv[0]);
    
    initScene();							// quick function to set up scene
    
   
    
    glutDisplayFunc(myDisplay);				// function to run when its time to draw something
    glutReshapeFunc(myReshape);				// function to run when the window gets resized
    glutIdleFunc(myDisplay);
    
    //Reads in keystrokes to either change view angle or exit
    glutKeyboardFunc(processNormalKeys);
	glutSpecialFunc(processSpecialKeys);

    
    // OpenGL init
	glEnable(GL_DEPTH_TEST);
    
    glutMainLoop();	// infinite loop that will keep drawing and resizing
    
    return 0;
}