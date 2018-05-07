//
// Raytracing logic
//

#include "Object.h"
#include "Matrix.h"
#include "Util.h"
#include <queue>
#pragma once

#define MAX_DIST 1000
#define MAX_DEPTH 8

#define EDGE_ONLY_SUPER true //Optimization technique to restrict super sampling to the edges of objects

//All the objects in the scene
class World
{
public:
	Object** objects;
	Light** lights;
	int n;
	int l;
	Color ambient;
	Color background;
	Material * mat;
	//Creates a world of n objects
	World(Object** objects, int n, Light** lights=nullptr, int l=0, Color ambient=Color(1,1,1), Color background = BLACK)
		: objects(objects), n(n), lights(lights), l(l), ambient(ambient), background(background) {
		mat = new FlatMaterial(background); 
	}
	//Return the color traced by a ray
	CO spawn(Ray r, double focal, int depth = 0);
};

//The entity generating images
class Camera
{
public:
	Point eyepoint;
	Point lookat;
	Vector up;
	Vector n;
	Vector u;
	Vector v;
	Matrix transform = Matrix(4);
	//Normalizes the vectors before assigning them
	Camera(Point eyepoint, Point lookat, Vector up);
	void render(World w, double focal, double width, double height, Image* i, int samples=1, PixelMask* pm = nullptr);
	void render(World w, double focal, double width, double height, Image* i, PixelMask* pm);
	void renderPixelMask(GameBoard* gb, double focal, double width, double height, PixelMask* pm);
};

typedef struct Task {
	Ray r = Ray(Point(), Vector());
	int x;
	int y;
} Task;

//Used for parellelism, creates a number of threads to run the rendering task
//Threads are created and pull tasks from a queue until no tasks remain
//To use:
//	Create Needle
//	Push tasks
//	Call run (Must be done AFTER all tasks are pushed)
//	Delete Needle, or push new tasks
class Needle
{
	std::queue<Task *> tasks;
	std::mutex mu; //Protect queue
	std::thread* threads;

public:
	int numThreads;
	World* w;
	Image* im;
	PixelMask* pm; //Used for ThunkTS PixelMap optimization
	GameBoard* gb; //Used for ThunkTS PixelMap optimization
	double d;
	Vector du, dv; //Only used by super sampler

	Needle(World* w, Image* im, double d, int numThreads = 100) : w(w), im(im), d(d), numThreads(numThreads) {
		threads = new std::thread[numThreads];
	}
	Needle(World* w, Image* im, double d, Vector du, Vector dv, int numThreads = 100) : w(w), im(im), d(d), du(du), dv(dv), numThreads(numThreads) {
		threads = new std::thread[numThreads];
	}
	//Constructor for calculating pixel mask
	Needle(GameBoard* gb, PixelMask* pm, int numThreads = 100) : gb(gb), pm(pm), numThreads(numThreads) {
		threads = new std::thread[numThreads];
	}
	~Needle(){
		delete[] threads;
	}
	void pushTask(Ray r, int x, int y);
	//Used by threads
	Task* getTask();
	//Renders pixels to image
	void run();
	//Generates image with supersampling
	void superSampleRun(int samples);
	//Calculates pixelmask
	void pixelMaskRun();
};

//Called in threads
void executeTask(Needle* n);
void executeSuperSample(Needle* n, int samples);
void executeMaskTask(Needle* n);