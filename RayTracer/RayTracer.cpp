// RayTracer.cpp : Defines the entry point for the console application.
//

#include "Util.h"
#include "Object.h"
#include "Matrix.h"
#include "World.h"
#include "KDTree.h"
#include <iostream>
#include <SDL.h>
#include <fstream>
#include <sstream>
#include <string>
#include <chrono>
#include "Entities.h"


#define WINDOW_WIDTH 600
#define WINDOW_HEIGHT 600

using namespace std;

Image* mainA()
{
	Material* red = new PhongMaterial(RED);
	Material* blue = new PhongMaterial(BLUE);
	Material* green = new PhongMaterial(GREEN, WHITE, 0.2, 0.8, 0.0);
	Sphere* a = new Sphere(Point(-1.5, 1, 0), 0.8, red);
	Sphere* b = new Sphere(Point(-0.5, 0.65, -0.8), 0.6, blue);
	//Cylinder* b = new Cylinder(Point(-0.5, 0.05, -0.8), Point(-0.5, 1.25, -0.8), 0.6, blue);

	Point px = Point(-3, -0.1, -7.5);
	Point py = Point(-3, -0.1, 7.5);
	Point pz = Point(3, -0.1, -7.5);
	Point pw = Point(3, -0.1, 7.5);

	Triangle* t1 = new Triangle(px, pz, py, green);
	Triangle* t2 = new Triangle(py, pz, pw, green);

	Object** list = new Object*[4];
	list[0] = a;
	list[1] = b;
	list[2] = t1;
	list[3] = t2;

	//Object** kd = new Object*[1];
	//kd[0] = makeKDTree(list, 4);

	Light* lights[3];
	lights[0] = new Light(WHITE, Point(-2, 3.5, 4));

	World w = World(list, 4, lights, 1);
	//World w = World(kd, 1, lights, 1);

	Point eyepoint = Point(-1.5, 1, 5);
	Point lookat = Point(-1.5, 1.11, 3.00);
	Vector up = Vector(0, 1, 0);

	Camera c = Camera(eyepoint, lookat, up);

	Image* im = new Image(WINDOW_WIDTH, WINDOW_HEIGHT);

	c.render(w, .5, .5, .5, im);
	//c.render(w, .5, .5, .5, im, 16);

	return im;
}

//Hardcoded ply parser because no need for anything else
Image* bunny() {
	Image* im = new Image(WINDOW_WIDTH, WINDOW_HEIGHT);

	ifstream infile("bun_zipper.ply");
	string line;

	for(int i=0;i<4;i++)
		getline(infile, line);

	istringstream iss(line);

	string a, b;
	int numVertex;
	iss >> a >> b >> numVertex;

	for (int i = 0; i<6; i++)
		getline(infile, line);

	iss = istringstream(line);

	int numTriangles;
	iss >> a >> b >> numTriangles;
	
	for (int i = 0; i<2; i++)
		getline(infile, line);

	//End of header
	Point* v = new Point[numVertex];
	Object** o = new Object*[numTriangles];

	for (int i = 0; i < numVertex; i++) {
		double x, y, z;
		getline(infile, line);
		iss = istringstream(line);
		iss >> x >> y >> z;
		v[i] = Point(x*8, y*8-1, z*8);
	}

	Material* bun = new PhongMaterial(Color(0.8, 0.8, 0.8));

	for (int i = 0; i < numTriangles; i++) {
		int n, a, b, c;
		getline(infile, line);
		iss = istringstream(line);
		iss >> n >> a >> b >> c;
		o[i] = new Triangle(v[a], v[b], v[c], bun);
	}

	Object** kd = new Object*[1];
	auto start = chrono::system_clock::now();
	kd[0] = makeKDTree(o, numTriangles);
	auto end = chrono::system_clock::now();

	chrono::duration<double> elapsed_seconds = end - start;

	cout << elapsed_seconds.count() << "s\n";

	Light* lights[2];
	lights[0] = new Light(Color(2, 2, 2), Point(-1, 3, -1));
	lights[1] = new Light(Color(0, 0, 1), Point(-1.5, 1, 5));

	World w = World(kd, 1, lights, 2);
	//World w = World(o, numTriangles, lights, 2); //No kd tree. Why would you do this?

	Point eyepoint = Point(-1.5, 1, 5);
	Point lookat = Point(0, 0, 0);
	Vector up = Vector(0, 1, 0);

	Camera c = Camera(eyepoint, lookat, up);

	start = chrono::system_clock::now();
	c.render(w, 1, .5, .5, im);
	end = chrono::system_clock::now();

	elapsed_seconds = end - start;

	cout << elapsed_seconds.count() << "s\n";

	delete[] v;
	for (int i = 0; i < numTriangles; i++) {
		delete o[i];
	}
	delete[] o;

	return im;
}

Image* game() {
	Board* b = new Board();

	b->makeBoard(1);

	GameBoard* a = new GameBoard(b, 11, new BlinnMaterial(GREEN));

	Object** list = new Object*[1];
	list[0] = a;

	Light* lights[1];
	lights[0] = new Light(WHITE, Point(4.5, 3.5, 4.5));

	World w = World(list, 1, lights, 1);

	Point eyepoint = Point(-3, 11, 14);
	Point lookat = Point(5.5, 0, 5.5);
	Vector up = Vector(0, 1, 0);

	Camera c = Camera(eyepoint, lookat, up);

	Image* im = new Image(WINDOW_WIDTH, WINDOW_HEIGHT);

	c.render(w, .5, .5, .5, im);

	return im;
}

int main(int argc, char *args[])
{
	//Image* im = mainA();
	//Image* im = bunny();
	Image* im = game();

	SDL_Event event;
	SDL_Renderer *renderer;
	SDL_Window *window;
	int x, y;

	SDL_Init(SDL_INIT_EVERYTHING);
	SDL_CreateWindowAndRenderer(WINDOW_WIDTH, WINDOW_HEIGHT, 0, &window, &renderer);
	SDL_SetRenderDrawColor(renderer, 0, 0, 0, 0);
	SDL_RenderClear(renderer);
	for (x = 0; x < WINDOW_WIDTH; ++x) {
		for (y = 0; y < WINDOW_HEIGHT; ++y) {
			Color c = im->getPixel(x, y);
			SDL_SetRenderDrawColor(renderer, (Uint8)(c.r * 255), (Uint8)(c.g * 255), (Uint8)(c.b * 255), 255);
			SDL_RenderDrawPoint(renderer, x, y);
		}
	}
	SDL_RenderPresent(renderer);

	bool done = false;
	while (!done) {
		while (SDL_PollEvent(&event)) {
			if (event.type == SDL_QUIT)
			{
				done = true;
				break;
			}
			if (event.type == SDL_MOUSEBUTTONDOWN) {
				int id = im->getId(event.button.x, event.button.y);
				SDL_SetRenderDrawColor(renderer, 0, 0, 0, 0);
				SDL_RenderClear(renderer);
				for (x = 0; x < WINDOW_WIDTH; ++x) {
					for (y = 0; y < WINDOW_HEIGHT; ++y) {
						if (im->getId(x, y) == id) {
							Color c = im->getPixel(x, y);
							SDL_SetRenderDrawColor(renderer, (Uint8)(c.r * 127), (Uint8)(c.g * 127), (Uint8)(c.b * 255), 255);
							SDL_RenderDrawPoint(renderer, x, y);
						}
						else {
							Color c = im->getPixel(x, y);
							SDL_SetRenderDrawColor(renderer, (Uint8)(c.r * 255), (Uint8)(c.g * 255), (Uint8)(c.b * 255), 255);
							SDL_RenderDrawPoint(renderer, x, y);
						}
					}
				}
				SDL_RenderPresent(renderer);
			}
		}
	}
	SDL_DestroyRenderer(renderer);
	SDL_DestroyWindow(window);
	SDL_Quit();

	return 0;
}