 ///////////////////////////////////////////////////////////////////////
//
// P3D Course
// (c) 2021 by Jo�o Madeiras Pereira
//Ray Tracing P3F scenes and drawing points with Modern OpenGL
//
///////////////////////////////////////////////////////////////////////

#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string.h>
#include <stdio.h>
#include <chrono>
#include <conio.h>

#include <GL/glew.h>
#include <GL/freeglut.h>
#include <IL/il.h>

#include "scene.h"
#include "rayAccelerator.h"
#include "maths.h"
#include "macros.h"

//Enable OpenGL drawing
bool drawModeEnabled = false;

bool P3F_scene = true; //choose between P3F scene or a built-in random scene

bool ANTIALIASING = false;
bool SOFT_SHADOWS = false;
bool SOFT_SHADOWS_AA = false;
bool DOF = false;

#define SPP scene->GetSamplesPerPixel()
//#define SPP 256.0f
#define DOF_SAMPLES 4

// EXTRA
bool SOFT_SHADOWS_JITTERING = false;
bool MOTION_BLUR = false;
bool FUZZY_REFLECTIONS = false;

#define MAX_DEPTH 6  //number of bounces

#define CAPTION "Whitted Ray-Tracer"
#define VERTEX_COORD_ATTRIB 0
#define COLOR_ATTRIB 1

unsigned int FrameCount = 0;

// Current Camera Position
float camX, camY, camZ;

//Original Camera position;
Vector Eye;

// Mouse Tracking Variables
int startX, startY, tracking = 0;

// Camera Spherical Coordinates
float alpha = 0.0f, beta = 0.0f;
float r = 4.0f;

// Frame counting and FPS computation
long myTime, timebase = 0, frame = 0;
char s[32];


// Points defined by 2 attributes: positions which are stored in vertices array and colors which are stored in colors array
float *colors;
float *vertices;
int size_vertices;
int size_colors;

//Array of Pixels to be stored in a file by using DevIL library
uint8_t *img_Data;

GLfloat m[16];  //projection matrix initialized by ortho function

GLuint VaoId;
GLuint VboId[2];

GLuint VertexShaderId, FragmentShaderId, ProgramId;
GLint UniformId;

Scene* scene = NULL;

Grid* grid_ptr = NULL;
BVH* bvh_ptr = NULL;
accelerator Accel_Struct = NONE;

int RES_X, RES_Y;

int WindowHandle = 0;



/////////////////////////////////////////////////////////////////////// ERRORS

bool isOpenGLError() {
	bool isError = false;
	GLenum errCode;
	const GLubyte *errString;
	while ((errCode = glGetError()) != GL_NO_ERROR) {
		isError = true;
		errString = gluErrorString(errCode);
		std::cerr << "OpenGL ERROR [" << errString << "]." << std::endl;
	}
	return isError;
}

void checkOpenGLError(std::string error)
{
	if(isOpenGLError()) {
		std::cerr << error << std::endl;
		exit(EXIT_FAILURE);
	}
}

/////////////////////////////////////////////////////////////////////// SHADERs

const GLchar* VertexShader =
{
	"#version 430 core\n"

	"in vec2 in_Position;\n"
	"in vec3 in_Color;\n"
	"uniform mat4 Matrix;\n"
	"out vec4 color;\n"

	"void main(void)\n"
	"{\n"
	"	vec4 position = vec4(in_Position, 0.0, 1.0);\n"
	"	color = vec4(in_Color, 1.0);\n"
	"	gl_Position = Matrix * position;\n"

	"}\n"
};

const GLchar* FragmentShader =
{
	"#version 430 core\n"

	"in vec4 color;\n"
	"out vec4 out_Color;\n"

	"void main(void)\n"
	"{\n"
	"	out_Color = color;\n"
	"}\n"
};

void createShaderProgram()
{
	VertexShaderId = glCreateShader(GL_VERTEX_SHADER);
	glShaderSource(VertexShaderId, 1, &VertexShader, 0);
	glCompileShader(VertexShaderId);

	FragmentShaderId = glCreateShader(GL_FRAGMENT_SHADER);
	glShaderSource(FragmentShaderId, 1, &FragmentShader, 0);
	glCompileShader(FragmentShaderId);

	ProgramId = glCreateProgram();
	glAttachShader(ProgramId, VertexShaderId);
	glAttachShader(ProgramId, FragmentShaderId);

	glBindAttribLocation(ProgramId, VERTEX_COORD_ATTRIB, "in_Position");
	glBindAttribLocation(ProgramId, COLOR_ATTRIB, "in_Color");
	
	glLinkProgram(ProgramId);
	UniformId = glGetUniformLocation(ProgramId, "Matrix");

	checkOpenGLError("ERROR: Could not create shaders.");
}

void destroyShaderProgram()
{
	glUseProgram(0);
	glDetachShader(ProgramId, VertexShaderId);
	glDetachShader(ProgramId, FragmentShaderId);

	glDeleteShader(FragmentShaderId);
	glDeleteShader(VertexShaderId);
	glDeleteProgram(ProgramId);

	checkOpenGLError("ERROR: Could not destroy shaders.");
}

/////////////////////////////////////////////////////////////////////// VAOs & VBOs

void createBufferObjects()
{
	glGenVertexArrays(1, &VaoId);
	glBindVertexArray(VaoId);
	glGenBuffers(2, VboId);
	glBindBuffer(GL_ARRAY_BUFFER, VboId[0]);

	/* S� se faz a aloca��o dos arrays glBufferData (NULL), e o envio dos pontos para a placa gr�fica
	� feito na drawPoints com GlBufferSubData em tempo de execu��o pois os arrays s�o GL_DYNAMIC_DRAW */
	glBufferData(GL_ARRAY_BUFFER, size_vertices, NULL, GL_DYNAMIC_DRAW);
	glEnableVertexAttribArray(VERTEX_COORD_ATTRIB);
	glVertexAttribPointer(VERTEX_COORD_ATTRIB, 2, GL_FLOAT, 0, 0, 0);
	
	glBindBuffer(GL_ARRAY_BUFFER, VboId[1]);
	glBufferData(GL_ARRAY_BUFFER, size_colors, NULL, GL_DYNAMIC_DRAW);
	glEnableVertexAttribArray(COLOR_ATTRIB);
	glVertexAttribPointer(COLOR_ATTRIB, 3, GL_FLOAT, 0, 0, 0);
	
// unbind the VAO
	glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
//	glDisableVertexAttribArray(VERTEX_COORD_ATTRIB); 
//	glDisableVertexAttribArray(COLOR_ATTRIB);
	checkOpenGLError("ERROR: Could not create VAOs and VBOs.");
}

void destroyBufferObjects()
{
	glDisableVertexAttribArray(VERTEX_COORD_ATTRIB);
	glDisableVertexAttribArray(COLOR_ATTRIB);
	
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);

	glDeleteBuffers(1, VboId);
	glDeleteVertexArrays(1, &VaoId);
	checkOpenGLError("ERROR: Could not destroy VAOs and VBOs.");
}

void drawPoints()
{
	FrameCount++;
	glClear(GL_COLOR_BUFFER_BIT);

	glBindVertexArray(VaoId);
	glUseProgram(ProgramId);

	glBindBuffer(GL_ARRAY_BUFFER, VboId[0]);
	glBufferSubData(GL_ARRAY_BUFFER, 0, size_vertices, vertices);
	glBindBuffer(GL_ARRAY_BUFFER, VboId[1]);
	glBufferSubData(GL_ARRAY_BUFFER, 0, size_colors, colors);

	glUniformMatrix4fv(UniformId, 1, GL_FALSE, m);
	glDrawArrays(GL_POINTS, 0, RES_X*RES_Y);
	glFinish();

	glUseProgram(0);
	glBindVertexArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	checkOpenGLError("ERROR: Could not draw scene.");
}

ILuint saveImgFile(const char *filename) {
	ILuint ImageId;

	ilEnable(IL_FILE_OVERWRITE);
	ilGenImages(1, &ImageId);
	ilBindImage(ImageId);

	ilTexImage(RES_X, RES_Y, 1, 3, IL_RGB, IL_UNSIGNED_BYTE, img_Data /*Texture*/);
	ilSaveImage(filename);

	ilDisable(IL_FILE_OVERWRITE);
	ilDeleteImages(1, &ImageId);
	if (ilGetError() != IL_NO_ERROR)return ilGetError();

	return IL_NO_ERROR;
}

/////////////////////////////////////////////////////////////////////// CALLBACKS

void timer(int value)
{
	std::ostringstream oss;
	oss << CAPTION << ": " << FrameCount << " FPS @ (" << RES_X << "x" << RES_Y << ")";
	std::string s = oss.str();
	glutSetWindow(WindowHandle);
	glutSetWindowTitle(s.c_str());
	FrameCount = 0;
	glutTimerFunc(1000, timer, 0);
}


// Callback function for glutCloseFunc
void cleanup()
{
	destroyShaderProgram();
	destroyBufferObjects();
}

void ortho(float left, float right, float bottom, float top, 
			float nearp, float farp)
{
	m[0 * 4 + 0] = 2 / (right - left);
	m[0 * 4 + 1] = 0.0;
	m[0 * 4 + 2] = 0.0;
	m[0 * 4 + 3] = 0.0;
	m[1 * 4 + 0] = 0.0;
	m[1 * 4 + 1] = 2 / (top - bottom);
	m[1 * 4 + 2] = 0.0;
	m[1 * 4 + 3] = 0.0;
	m[2 * 4 + 0] = 0.0;
	m[2 * 4 + 1] = 0.0;
	m[2 * 4 + 2] = -2 / (farp - nearp);
	m[2 * 4 + 3] = 0.0;
	m[3 * 4 + 0] = -(right + left) / (right - left);
	m[3 * 4 + 1] = -(top + bottom) / (top - bottom);
	m[3 * 4 + 2] = -(farp + nearp) / (farp - nearp);
	m[3 * 4 + 3] = 1.0;
}

void reshape(int w, int h)
{
    glClear(GL_COLOR_BUFFER_BIT);
	glViewport(0, 0, w, h);
	ortho(0, (float)RES_X, 0, (float)RES_Y, -1.0, 1.0);
}

void processKeys(unsigned char key, int xx, int yy)
{
	switch (key) {

		case 27:
			glutLeaveMainLoop();
			break;

		case 'r':
			camX = Eye.x;
			camY = Eye.y;
			camZ = Eye.z;
			r = Eye.length();
			beta = asinf(camY / r) * 180.0f / 3.14f;
			alpha = atanf(camX / camZ) * 180.0f / 3.14f;
			break;

		case 'c':
			printf("Camera Spherical Coordinates (%f, %f, %f)\n", r, beta, alpha);
			printf("Camera Cartesian Coordinates (%f, %f, %f)\n", camX, camY, camZ);
			break;
	}
}


// ------------------------------------------------------------
//
// Mouse Events
//

void processMouseButtons(int button, int state, int xx, int yy)
{
	// start tracking the mouse
	if (state == GLUT_DOWN) {
		startX = xx;
		startY = yy;
		if (button == GLUT_LEFT_BUTTON)
			tracking = 1;
		else if (button == GLUT_RIGHT_BUTTON)
			tracking = 2;
	}

	//stop tracking the mouse
	else if (state == GLUT_UP) {
		if (tracking == 1) {
			alpha -= (xx - startX);
			beta += (yy - startY);
		}
		else if (tracking == 2) {
			r += (yy - startY) * 0.01f;
			if (r < 0.1f)
				r = 0.1f;
		}
		tracking = 0;
	}
}

// Track mouse motion while buttons are pressed

void processMouseMotion(int xx, int yy)
{

	int deltaX, deltaY;
	float alphaAux, betaAux;
	float rAux;

	deltaX = -xx + startX;
	deltaY = yy - startY;

	// left mouse button: move camera
	if (tracking == 1) {


		alphaAux = alpha + deltaX;
		betaAux = beta + deltaY;

		if (betaAux > 85.0f)
			betaAux = 85.0f;
		else if (betaAux < -85.0f)
			betaAux = -85.0f;
		rAux = r;
	}
	// right mouse button: zoom
	else if (tracking == 2) {

		alphaAux = alpha;
		betaAux = beta;
		rAux = r + (deltaY * 0.01f);
		if (rAux < 0.1f)
			rAux = 0.1f;
	}

	camX = rAux * sin(alphaAux * 3.14f / 180.0f) * cos(betaAux * 3.14f / 180.0f);
	camZ = rAux * cos(alphaAux * 3.14f / 180.0f) * cos(betaAux * 3.14f / 180.0f);
	camY = rAux * sin(betaAux * 3.14f / 180.0f);
}

void mouseWheel(int wheel, int direction, int x, int y) {

	r += direction * 0.1f;
	if (r < 0.1f)
		r = 0.1f;

	camX = r * sin(alpha * 3.14f / 180.0f) * cos(beta * 3.14f / 180.0f);
	camZ = r * cos(alpha * 3.14f / 180.0f) * cos(beta * 3.14f / 180.0f);
	camY = r * sin(beta * 3.14f / 180.0f);
}


void setupGLEW() {
	glewExperimental = GL_TRUE;
	GLenum result = glewInit() ; 
	if (result != GLEW_OK) { 
		std::cerr << "ERROR glewInit: " << glewGetString(result) << std::endl;
		exit(EXIT_FAILURE);
	} 
	GLenum err_code = glGetError();
	printf ("Vendor: %s\n", glGetString (GL_VENDOR));
	printf ("Renderer: %s\n", glGetString (GL_RENDERER));
	printf ("Version: %s\n", glGetString (GL_VERSION));
	printf ("GLSL: %s\n", glGetString (GL_SHADING_LANGUAGE_VERSION));
}

void setupGLUT(int argc, char* argv[])
{
	glutInit(&argc, argv);
	
	glutInitContextVersion(4, 3);
	glutInitContextFlags(GLUT_FORWARD_COMPATIBLE);
	glutInitContextProfile(GLUT_CORE_PROFILE);

	glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE,GLUT_ACTION_GLUTMAINLOOP_RETURNS);
	
	glutInitWindowPosition(100, 250);
	glutInitWindowSize(RES_X, RES_Y);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA);
	glDisable(GL_DEPTH_TEST);
	WindowHandle = glutCreateWindow(CAPTION);
	if(WindowHandle < 1) {
		std::cerr << "ERROR: Could not create a new rendering window." << std::endl;
		exit(EXIT_FAILURE);
	}
}


/////////////////////////////////////////////////////YOUR CODE HERE///////////////////////////////////////////////////////////////////////////////////////

Color rayTracing(Ray ray, int depth, float ior_1)  //index of refraction of medium 1 where the ray is travelling
{	

	// -----------------------------------------------------------------------------
	// INTERSECTIONS
	// -----------------------------------------------------------------------------

	float t1, t2, tClosest;
	int closestObjIdx;

	Object* closestObj = nullptr;
	bool rayIntersepts = false;
	Vector hitPoint;

	/* Uniform Grid Acceleration */
	if (Accel_Struct == GRID_ACC) {
		rayIntersepts = grid_ptr->Traverse(ray, &closestObj, hitPoint);
	}

	/* BVH Acceleration */
	else if (Accel_Struct == BVH_ACC) {
		rayIntersepts = bvh_ptr->Traverse(ray, &closestObj, hitPoint);
	}

	/* No Acceleration Structure */
	else {
		for (int i = 0; i < scene->getNumObjects(); i++)
		{
			if (scene->getObject(i)->intercepts(ray, t1)) {
				if (rayIntersepts) {
					if (t1 < tClosest) {
						closestObjIdx = i;
						tClosest = t1;
					}
				}
				else {
					closestObjIdx = i;
					tClosest = t1;
					rayIntersepts = true;
				}
			}
		}
		if (rayIntersepts) {
			hitPoint = ray.origin + (ray.direction * tClosest);
			closestObj = scene->getObject(closestObjIdx);
		}
	}


	// -----------------------------------------------------------------------------
	// DIFFUSE AND SPECULAR COLOR
	// -----------------------------------------------------------------------------

	Color backgroundColor = scene->GetBackgroundColor();
	Color diffColor, specColor;

	if (!rayIntersepts) {
		return backgroundColor;
	}
	else {

		diffColor = closestObj->GetMaterial()->GetDiffColor();
		specColor = closestObj->GetMaterial()->GetSpecColor();
		Color resultingColor = Color(.0f, .0f, .0f);

		Vector normal = closestObj->getNormal(hitPoint);

		bool isInsideObject = normal * ray.direction > 0;
		normal = isInsideObject ? normal * -1.0f : normal;
		Vector bias = normal * EPSILON;
		Vector v = ray.direction * -1.0f;

		for (int j = 0; j < scene->getNumLights(); j++)
		{
			bool inShadow = false;
			int lightSamples = 1;
			if (SOFT_SHADOWS) lightSamples = 4;

			Vector a, b;
			if (SOFT_SHADOWS || SOFT_SHADOWS_AA || SOFT_SHADOWS_JITTERING) {
				a = Vector(.5, 0, 0);
				b = Vector(0, .5, 0);
			}


			for (int i = 0; i < lightSamples; i++) {

				Vector l;
				Color lightSampleColor = Color(.0f, .0f, .0f);

				if (SOFT_SHADOWS) {
					int areaLightSide = (int)sqrt(lightSamples);
					int q = i / areaLightSide;
					int p = i - (q * areaLightSide);
					Vector randomVector = scene->getLight(j)->position + a * p * (1.0f / (areaLightSide - 1.0f)) + b * q * (1.0f / (areaLightSide - 1.0f));
					l = randomVector - hitPoint;
				}
				else if (SOFT_SHADOWS_AA) {
					double random1 = (double) rand() / (RAND_MAX);
					double random2 = (double) rand() / (RAND_MAX);
					Vector randomVector = scene->getLight(j)->position + a * random1 + b * random2;
					l = randomVector - hitPoint;
				}
				else if (SOFT_SHADOWS_JITTERING) {
					int p = floor<int>(rand() * sqrt(SPP) / (RAND_MAX));
					int q = floor<int>(rand() * sqrt(SPP) / (RAND_MAX));
					double random1 = (double)rand() / (RAND_MAX);
					double random2 = (double)rand() / (RAND_MAX);
					Vector randomVector = scene->getLight(j)->position + a * ((p + random1) / sqrt(SPP)) + b * ((q + random2) / sqrt(SPP));
					l = randomVector - hitPoint;
				}
				else {
					l = scene->getLight(j)->position - hitPoint;
				}

				float lightDistance = l.length();
				l = l.normalize();

				if (l * normal > .0f) {

					Ray shadowFeeler = Ray(hitPoint + bias, l);

					/* Uniform Grid Acceleration */
					if (Accel_Struct == GRID_ACC) {
						inShadow = grid_ptr->Traverse(shadowFeeler);
					}

					/* BVH Acceleration */
					else if (Accel_Struct == BVH_ACC) {
						inShadow = bvh_ptr->Traverse(shadowFeeler);
					}

					/* No Acceleration Structure */
					else {
						int k = 0;
						while (!inShadow)
						{
							if (k >= scene->getNumObjects()) break;

							if (scene->getObject(k)->intercepts(shadowFeeler, t2)) {
								if (t2 < lightDistance)  // check if the shadow is being casted by an object in front of the light source
								{
									inShadow = true;
									break;
								}
							}
							k++;
						}
					}

					if (!inShadow) {
					
						Color lightColor = scene->getLight(j)->color;

						float shine = closestObj->GetMaterial()->GetShine();
						float kd = closestObj->GetMaterial()->GetDiffuse();
						float ks = closestObj->GetMaterial()->GetSpecular();

						Vector halfwayVector = (l + v).normalize();

						Color diff = lightColor * kd * diffColor * (normal * l);

						lightSampleColor += diff;

						int numLights = scene->getNumLights();
						float k1 = 1.25f;
						float katt = 1.0f / (k1 * (numLights));

						if (ks > .0f && (normal * halfwayVector) > .0f)
						{
							Color spec = lightColor * ks * specColor * pow(normal * halfwayVector, shine) * katt;
							lightSampleColor += spec;
						}

					}
				}
				resultingColor += lightSampleColor * (1.0f / lightSamples);
			}

		}

		// return resultingColor;


		// -----------------------------------------------------------------------------
		// REFLECTION AND REFRACTION
		// -----------------------------------------------------------------------------

		if (depth >= MAX_DEPTH) {
			return resultingColor;
		}
		else {

			Vector refletedRayDirection = (normal * (2 * (v * normal)) - v);
			
			if (FUZZY_REFLECTIONS) {
				float roughness = .3f;
				Vector rand_in_unit_sphere = Vector((float)rand() / (RAND_MAX), (float)rand() / (RAND_MAX), (float)rand() / (RAND_MAX));
				rand_in_unit_sphere = rand_in_unit_sphere.normalize();
				refletedRayDirection = (refletedRayDirection + rand_in_unit_sphere * roughness).normalize();
			}
			else {
				refletedRayDirection = refletedRayDirection.normalize();
			}

			Ray reflectedRay = Ray(hitPoint + bias, refletedRayDirection);

			float reflection = closestObj->GetMaterial()->GetReflection();
			float T = closestObj->GetMaterial()->GetTransmittance();
			float ior_2 = isInsideObject ? 1.0f : closestObj->GetMaterial()->GetRefrIndex();

			Vector vt = (normal * (v * normal)) - v;
			float sinI, cosI, sinT, cosT, kr;
			sinI = vt.length();
			cosI = sqrt(1 - pow(sinI, 2.0f));
			sinT = (ior_1 / ior_2) * sinI;

			if (sinT > 1 || T == 0) {  // total reflection or non-dieletric material

				if (reflection > 0) resultingColor += rayTracing(reflectedRay, depth + 1, ior_1) * specColor * reflection;
					
			}
			else {
				float R0 = pow((ior_1 - ior_2) / (ior_1 + ior_2), 2);
				cosT = sqrt(1 - pow(sinT, 2));
				float cos = isInsideObject ? cosT : cosI;
				kr = R0 + (1.0f - R0) * pow(1.0f - cos, 5);
				
				// reflection
				if (reflection > 0) resultingColor += rayTracing(reflectedRay, depth + 1, ior_1) * kr;  // without specular color

				// refraction (dieletric material and non-total reflection)
				Vector rt = vt.normalize() * sinT - normal * cosT;
				Ray transmittedRay = Ray(hitPoint - bias, rt);
				resultingColor += rayTracing(transmittedRay, depth + 1, ior_2) * (1 - kr);  // without diffuse
			}

			return resultingColor;
		}
	}
}

Color dofRayTracing(const Vector& pixel) {
	Color color = Color(.0f, .0f, .0f);

	for (int i = 0; i < DOF_SAMPLES; i++) {
		Vector lens_sample = rnd_unit_disk() * scene->GetCamera()->GetAperture();
		
		Ray ray = scene->GetCamera()->PrimaryRay(lens_sample, pixel);
		color += rayTracing(ray, 1, 1.0).clamp() * (float)(1.0 / DOF_SAMPLES);
		
	}

	return color;
}

// Render function by primary ray casting from the eye towards the scene's objects

void renderScene()
{
	int index_pos = 0;
	int index_col = 0;
	unsigned int counter = 0;

	if (drawModeEnabled) {
		glClear(GL_COLOR_BUFFER_BIT);
		scene->GetCamera()->SetEye(Vector(camX, camY, camZ));  //Camera motion
	}


	for (int y = 0; y < RES_Y; y++)
	{
		for (int x = 0; x < RES_X; x++)
		{
			
			Color color = Color(.0f, .0f,.0f);
			Vector pixel;
			
			/* ANTI-ALIASING */
			if (ANTIALIASING) {
				for (float i = 0; i < (int)sqrt(SPP); i++) {
					for (float j = 0; j < (int)sqrt(SPP); j++) {

						double ksi1 = (double)rand() / (RAND_MAX);
						double ksi2 = (double)rand() / (RAND_MAX);

						//viewport coordinates
						
						pixel.x = x + (i + ksi1) / sqrt(SPP);
						pixel.y = y + (j + ksi2) / sqrt(SPP);

						if (DOF && scene->GetCamera()->GetAperture() > 0)
							color += dofRayTracing(pixel).clamp() * (float)(1 / SPP);

						else {
							Ray ray = scene->GetCamera()->PrimaryRay(pixel);   //function from camera.h
							color += rayTracing(ray, 1, 1.0).clamp() * (float) (1 / SPP);

						}
					}
				}
			}
			else {
				pixel.x = x + 0.5f; 
				pixel.y = y + 0.5f; 

				if (DOF && scene->GetCamera()->GetAperture() > 0)
					color = dofRayTracing(pixel).clamp();

				else {
					Ray ray = scene->GetCamera()->PrimaryRay(pixel);
					color = rayTracing(ray, 1, 1.0).clamp(); // W/O ANTI-ALIASING
				}
			}


			img_Data[counter++] = u8fromfloat((float)color.r());
			img_Data[counter++] = u8fromfloat((float)color.g());
			img_Data[counter++] = u8fromfloat((float)color.b());

			if (drawModeEnabled) {
				vertices[index_pos++] = (float)x;
				vertices[index_pos++] = (float)y;
				colors[index_col++] = (float)color.r();

				colors[index_col++] = (float)color.g();

				colors[index_col++] = (float)color.b();
			}
		}

	}
	if (drawModeEnabled) {
		drawPoints();
		glutSwapBuffers();
	}
	else {
		printf("Terminou o desenho!\n");
		if (saveImgFile("RT_Output.png") != IL_NO_ERROR) {
			printf("Error saving Image file\n");
			exit(0);
		}
		printf("Image file created\n");
	}
}


///////////////////////////////////////////////////////////////////////  SETUP     ///////////////////////////////////////////////////////

void setupCallbacks()
{
	glutKeyboardFunc(processKeys);
	glutCloseFunc(cleanup);
	glutDisplayFunc(renderScene);
	glutReshapeFunc(reshape);
	glutMouseFunc(processMouseButtons);
	glutMotionFunc(processMouseMotion);
	glutMouseWheelFunc(mouseWheel);

	glutIdleFunc(renderScene);
	glutTimerFunc(0, timer, 0);
}
void init(int argc, char* argv[])
{
	// set the initial camera position on its spherical coordinates
	Eye = scene->GetCamera()->GetEye();
	camX = Eye.x;
	camY = Eye.y;
	camZ = Eye.z;
	r = Eye.length();
	beta = asinf(camY / r) * 180.0f / 3.14f;
	alpha = atanf(camX / camZ) * 180.0f / 3.14f;

	setupGLUT(argc, argv);
	setupGLEW();
	std::cerr << "CONTEXT: OpenGL v" << glGetString(GL_VERSION) << std::endl;
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
	createShaderProgram();
	createBufferObjects();
	setupCallbacks();
}


void init_scene(void)
{
	char scenes_dir[70] = "P3D_Scenes/";
	char input_user[50];
	char scene_name[70];

	scene = new Scene();

	if (P3F_scene) {  //Loading a P3F scene

		while (true) {
			cout << "Input the Scene Name: ";
			cin >> input_user;
			strcpy_s(scene_name, sizeof(scene_name), scenes_dir);
			strcat_s(scene_name, sizeof(scene_name), input_user);

			ifstream file(scene_name, ios::in);
			if (file.fail()) {
				printf("\nError opening P3F file.\n");
			}
			else
				break;
		}

		scene->load_p3f(scene_name);
		printf("Scene loaded.\n\n");
	}
	else {
		printf("Creating a Random Scene.\n\n");
		scene->create_random_scene();
	}


	RES_X = scene->GetCamera()->GetResX();
	RES_Y = scene->GetCamera()->GetResY();
	printf("\nResolutionX = %d  ResolutionY= %d.\n", RES_X, RES_Y);

	// Pixel buffer to be used in the Save Image function
	img_Data = (uint8_t*)malloc(3 * RES_X*RES_Y * sizeof(uint8_t));
	if (img_Data == NULL) exit(1);

	Accel_Struct = scene->GetAccelStruct();   //Type of acceleration data structure

	if (Accel_Struct == GRID_ACC) {
		grid_ptr = new Grid();
		vector<Object*> objs;
		int num_objects = scene->getNumObjects();

		for (int o = 0; o < num_objects; o++) {
			objs.push_back(scene->getObject(o));
		}
		grid_ptr->Build(objs);
		printf("Grid built.\n\n");
	}
	else if (Accel_Struct == BVH_ACC) {
		vector<Object*> objs;
		int num_objects = scene->getNumObjects();
		bvh_ptr = new BVH();

		for (int o = 0; o < num_objects; o++) {
			objs.push_back(scene->getObject(o));
		}
		bvh_ptr->Build(objs);
		printf("BVH built.\n\n");
	}
	else
		printf("No acceleration data structure.\n\n");

	unsigned int spp = scene->GetSamplesPerPixel();
	if (spp == 0)
		printf("Whitted Ray-Tracing\n");
	else
		printf("Distribution Ray-Tracing\n");

}

int main(int argc, char* argv[])
{
	//Initialization of DevIL 
	if (ilGetInteger(IL_VERSION_NUM) < IL_VERSION)
	{
		printf("wrong DevIL version \n");
		exit(0);
	}
	ilInit();

	int 
		ch;
	if (!drawModeEnabled) {

		do {
			init_scene();

			auto timeStart = std::chrono::high_resolution_clock::now();
			renderScene();  //Just creating an image file
			auto timeEnd = std::chrono::high_resolution_clock::now();
			auto passedTime = std::chrono::duration<double, std::milli>(timeEnd - timeStart).count();
			printf("\nDone: %.2f (sec)\n", passedTime / 1000);
			if (!P3F_scene) break;
			cout << "\nPress 'y' to render another image or another key to terminate!\n";
			delete(scene);
			free(img_Data);
			ch = _getch();
		} while((toupper(ch) == 'Y')) ;
	}

	else {   //Use OpenGL to draw image in the screen
		printf("OPENGL DRAWING MODE\n\n");
		init_scene();
		size_vertices = 2 * RES_X*RES_Y * sizeof(float);
		size_colors = 3 * RES_X*RES_Y * sizeof(float);
		vertices = (float*)malloc(size_vertices);
		if (vertices == NULL) exit(1);
		colors = (float*)malloc(size_colors);
		if (colors == NULL) exit(1);
		memset(colors, 0, size_colors);

		/* Setup GLUT and GLEW */
		init(argc, argv);
		glutMainLoop();
	}

	free(colors);
	free(vertices);
	printf("Program ended normally\n");
	exit(EXIT_SUCCESS);
}
///////////////////////////////////////////////////////////////////////