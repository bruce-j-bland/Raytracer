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
	Material* glass = new PhongMaterial(WHITE);
	glass->setTransparency(0.8, .95);
	Material* mirror = new PhongMaterial(WHITE);
	mirror->setReflectivity(0.8);
	Material* green = new PhongMaterial(GREEN, WHITE, 0.2, 0.8, 0.0);
	Material* cyan = new PhongMaterial(Color(0,0.8,0.8), WHITE, 0.2, 0.8, 0.0);

	Material* checker = new Checkerboard(green, cyan, 20);

	Sphere* a = new Sphere(Point(-1.5, 1, 0), 0.8, glass);
	Sphere* b = new Sphere(Point(-0.5, 0.65, -0.8), 0.6, mirror);
	//Cylinder* b = new Cylinder(Point(-0.5, 0.05, -0.8), Point(-0.5, 1.25, -0.8), 0.6, mirror);

	Point px = Point(-3, -0.1, -7.5);
	Point py = Point(-3, -0.1, 3.5);
	Point pz = Point(3, -0.1, -7.5);
	Point pw = Point(3, -0.1, 3.5);

	Point tx = Point(0, 0, 0);
	Point ty = Point(0, 1, 0);
	Point tz = Point(1, 0, 0);
	Point tw = Point(1, 1, 0);

	Triangle* t1 = new Triangle(px, pz, py, checker);
	Triangle* t2 = new Triangle(py, pz, pw, checker);

	t1->setTextureMap(ty, tw, tx);
	t2->setTextureMap(tx, tw, tz);

	Object** list = new Object*[4];
	list[0] = a;
	list[1] = b;
	list[2] = t1;
	list[3] = t2;

	//Object** kd = new Object*[1];
	//kd[0] = makeKDTree(list, 4);

	Light* lights[3];
	double lightIntensity = 500.0;
	lights[0] = new Light(lightIntensity*WHITE, Point(-2, 3.5, 4));

	World w = World(list, 4, lights, 1, lightIntensity*WHITE, lightIntensity*WHITE);
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

Image* mainRecur(Image* image)
{
	Material* red = new PhongMaterial(RED);
	Material* blue = new PhongMaterial(BLUE);
	Material* green = new PhongMaterial(GREEN, WHITE, 0.2, 0.8, 0.0);
	Material* cyan = new PhongMaterial(Color(0, 0.8, 0.8), WHITE, 0.2, 0.8, 0.0);

	Material* imat = new ImageTexture(image);

	Sphere* a = new Sphere(Point(-1.5, 1, 0), 0.8, red);
	Sphere* b = new Sphere(Point(-0.5, 0.65, -0.8), 0.6, blue);
	//Cylinder* b = new Cylinder(Point(-0.5, 0.05, -0.8), Point(-0.5, 1.25, -0.8), 0.6, blue);

	Point px = Point(-3, -0.1, -7.5);
	Point py = Point(-3, -0.1, 3.5);
	Point pz = Point(3, -0.1, -7.5);
	Point pw = Point(3, -0.1, 3.5);

	Point tx = Point(0, 0, 0);
	Point ty = Point(0, 1, 0);
	Point tz = Point(1, 0, 0);
	Point tw = Point(1, 1, 0);

	Triangle* t1 = new Triangle(px, pz, py, imat);
	Triangle* t2 = new Triangle(py, pz, pw, imat);

	t1->setTextureMap(ty, tw, tx);
	t2->setTextureMap(tx, tw, tz);

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

	World w = World(kd, 1, lights, 2, WHITE, GREEN);
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

void refresh(SDL_Renderer* renderer, Image* im, int id, bool isBlue = true) {
	SDL_RenderClear(renderer);
	for (int x = 0; x < WINDOW_WIDTH; ++x) {
		for (int y = 0; y < WINDOW_HEIGHT; ++y) {
			bool selected = (im->getId(x, y) == id);
			if (id == -1) {
				if (x < 250 || x > 350 || y < 260 || y > 340)
					selected = false;
			}
			if (selected) {
				Color c = im->getPixel(x, y);
				if(isBlue)
					SDL_SetRenderDrawColor(renderer, (Uint8)(c.r * 127), (Uint8)(c.g * 127), (Uint8)(c.b * 127) + 127, 255);
				else
					SDL_SetRenderDrawColor(renderer, (Uint8)(c.r * 127) + 127, (Uint8)(c.g * 127), (Uint8)(c.b * 127), 255);
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

void clear() {
	for (int i = 0; i < 35; i++)
		cout << endl;
}

void sleep(int milli) {
	std::this_thread::sleep_for(std::chrono::milliseconds(milli));
}

void game() {
	clear();
	Board* b = new Board();

	b->makeBoard(1);
	b->start();

	GameBoard* a = new GameBoard(b, 11, new BlinnMaterial(GREEN));

	Cylinder* dropWall = new Cylinder(Point(5.5, 0.1, 5.5), Point(5.5, 0.9, 5.5), 0.4, new PhongMaterial(Color(.75, .75, .75)));


	Object** list = new Object*[2];
	list[0] = a;
	list[1] = dropWall;

	Light* lights[1];
	lights[0] = new Light(WHITE, Point(4.5, 13.5, 4.5));

	World w = World(list, 1, lights, 1);
	World w2 = World(list, 2, lights, 1);

	Point eyepoint = Point(-3, 11, 14);
	Point lookat = Point(5.5, 0, 5.5);
	Vector up = Vector(0, 1, 0);

	Camera c = Camera(eyepoint, lookat, up);

	Image* im = new Image(WINDOW_WIDTH, WINDOW_HEIGHT);

	PixelMask* pm = new PixelMask(WINDOW_WIDTH, WINDOW_HEIGHT, 122);

	c.render(w, .5, .5, .5, im);
	c.renderPixelMask(a, .5, .5, .5, pm);

	SDL_Event event;
	SDL_Renderer *renderer;
	SDL_Window *window;
	int x, y;

	//pm->loadMask(27);

	SDL_Init(SDL_INIT_EVERYTHING);
	SDL_CreateWindowAndRenderer(WINDOW_WIDTH, WINDOW_HEIGHT, 0, &window, &renderer);
	SDL_SetRenderDrawColor(renderer, 0, 0, 0, 0);
	SDL_RenderClear(renderer);
	for (x = 0; x < WINDOW_WIDTH; ++x) {
		for (y = 0; y < WINDOW_HEIGHT; ++y) {
			if (pm->getPixelMask(x, y)) {
				Color c = im->getPixel(x, y);
				SDL_SetRenderDrawColor(renderer, (Uint8)(c.r * 127)+127, (Uint8)(c.g * 127)+127, (Uint8)(c.b * 127)+127, 255);
				SDL_RenderDrawPoint(renderer, x, y);
			}
			else {
				Color c = im->getPixel(x, y);
				SDL_SetRenderDrawColor(renderer, (Uint8)(c.r * 255), (Uint8)(c.g * 255), (Uint8)(c.b * 255), 255);
				SDL_RenderDrawPoint(renderer, x, y);
			}
		}
	}

	pm->resetMasks();

	SDL_RenderPresent(renderer);

	bool done = false;
	int id = -5;

	Phase p = b->getPhase();
	bool playerOne = true;

	cout << "It is blue's turn." << endl;
	cout << "Click an empty tile to drop a barrier." << endl;

	while (!done) {
		while (SDL_PollEvent(&event)) {
			if (event.type == SDL_QUIT)
			{
				done = true;
				break;
			}
			if (event.type == SDL_MOUSEBUTTONDOWN) {
				if (p == Phase::place) {
					int nid = im->getId(event.button.x, event.button.y);
					if (nid == id) {
						int bx = id % 11;
						int by = id / 11;

						bool fall = false;

						if (id == -1) {
							bx = 5;
							by = 5;
							nid = 60;
							fall = true;
						}

						if (b->drop(by, bx)) {
							while (!(b->act()));
							p = b->next();
							id = -5;

							if (fall) {
								pm->loadMask(121);
								for (int i = 0; i < 10; i++) {
									c.render(w2, .5, .5, .5, im, pm);
									dropWall->a = dropWall->a + Vector(0, -0.25, 0);
									dropWall->b = dropWall->b + Vector(0, -0.25, 0);
									refresh(renderer, im, id, playerOne);
									sleep(1);
								}
								dropWall->a = Point(5.5, 0.1, 5.5);
								dropWall->b = Point(5.5, 0.9, 5.5);
								pm->resetMasks();
							}
							else {
								pm->loadMask(nid);
								c.render(w, .5, .5, .5, im, pm);
								pm->resetMasks();
								refresh(renderer, im, id, playerOne);
							}

							if (b->isDead(0)) {
								cout << "Blue loses. Red wins.\n";
								p = Phase::end;
							}
							else if (b->isDead(1)) {
								cout << "Red loses. Blue wins.\n";
								p = Phase::end;
							}
							else {
								cout << "Click an edge tile to move in that direction." << endl;
							}
						}
						else {
							continue;
						}
					}
					else if (nid == -1) {
						if (event.button.x < 350 && event.button.x > 250 && event.button.y > 260 && event.button.y < 340) {
							id = -1;
							refresh(renderer, im, id, playerOne);
						}
						else {
							if (id == -5)
								continue;
							else {
								id = -5;
								refresh(renderer, im, id, playerOne);
							}
						}
					}
					else {
						pm->resetMasks();
						id = nid;
						pm->loadMask(nid);
						refresh(renderer, im, id, playerOne);
					}
				}
				else if (p == Phase::move) {
					int nid = im->getId(event.button.x, event.button.y);
					if (nid == -1) {
						if (id == -5)
							continue;
						else {
							id = -5;
							refresh(renderer, im, id, playerOne);
						}
					}
					else if (nid == id) {
						int bx = id % 11;
						int by = id / 11;

						Direction d = Direction::blank;

						if ((by == 0 && (bx == 0 || bx == 10)) || (by == 10 && (bx == 0 || bx == 10))) {
							id = -5;
							refresh(renderer, im, id, playerOne);
						}
						else if (by == 0) {
							d = Direction::left;
						}
						else if (by == 10) {
							d = Direction::right;
						}
						else if (bx == 0) {
							d = Direction::up;
						}
						else if (bx == 10) {
							d = Direction::down;
						}
						else {
							id = -5;
							refresh(renderer, im, id, playerOne);
						}

						if (b->isLegal(d)) {
							Player* currentPlayer = b->getCurrentPlayer();
							pm->loadMask(currentPlayer->getY() + currentPlayer->getX() * 11);
							b->input(d);
							while (!(b->act()));
							p = b->next();
							if (b->isDead(0)) {
								cout << "Blue loses. Red wins.\n";
								p = Phase::end;
							}
							else if (b->isDead(1)) {
								cout << "Red loses. Blue wins.\n";
								p = Phase::end;
							}

							if (currentPlayer->getX() == -1) { //Fell in a pit
								Material* dm;
								if (playerOne)
									dm = new PhongMaterial(BLUE);
								else
									dm = new PhongMaterial(RED);
								Sphere* dead = new Sphere(Point(5.5, 0.5, 5.5), 0.4, dm);
								Object** list2 = new Object*[2];
								list2[0] = a;
								list2[1] = dead;
								World w3 = World(list2, 2, lights, 1);
								pm->loadMask(121);

								c.render(w3, .5, .5, .5, im, pm); //Needed to erase old position
								pm->resetMasks();

								pm->loadMask(121);
								for (int i = 0; i < 10; i++) {
									c.render(w3, .5, .5, .5, im, pm);
									dead->p = dead->p + Vector(0, -0.25, 0);
									refresh(renderer, im, id, playerOne);
									sleep(1);
								}
								pm->resetMasks();
							}
							pm->loadMask(currentPlayer->getY() + currentPlayer->getX() * 11);

							id = -5;

							c.render(w, .5, .5, .5, im, pm);
							pm->resetMasks();

							refresh(renderer, im, id, playerOne);

							playerOne = !playerOne;
							if (p != Phase::end) {
								clear();
								if (playerOne)
									cout << "It is blue's turn." << endl;
								else
									cout << "It is red's turn." << endl;
								cout << "Click an empty tile to drop a barrier." << endl;
							}							
						}
						else {
							continue;
						}
					}
					else {
						int bx = nid % 11;
						int by = nid / 11;

						if ((by == 0 && (bx == 0 || bx == 10)) || (by == 10 && (bx == 0 || bx == 10))) {
							id = -5;
						}
						else if (by == 0) {
							id = nid;
							refresh(renderer, im, id, playerOne);
						}
						else if (by == 10) {
							id = nid;
							refresh(renderer, im, id, playerOne);
						}
						else if (bx == 0) {
							id = nid;
							refresh(renderer, im, id, playerOne);
						}
						else if (bx == 10) {
							id = nid;
							refresh(renderer, im, id, playerOne);
						}
						else {
							id = -5;
						}
					}
				}
			}
		}
	}
	SDL_DestroyRenderer(renderer);
	SDL_DestroyWindow(window);
	SDL_Quit();
}

int main(int argc, char *args[])
{
	Image* im = nullptr;

	cout << "Select an image\n\n";

	int i = 0;

	while (i == 0) {
		cout << "Whitted: 1\nStanford Bunny: 2\nThunk TS: 3\n";
		string input = "";
		getline(cin, input);
		stringstream stream(input);
		if (!(stream >> i))
			continue;
		if (i == 1)
			im = mainA();
		else if (i == 2)
			im = bunny();
		else if (i == 3)
		{
			game();
			return 0;
		}
		else if (i == 4)
			im = mainRecur(bunny());
		else
			i = 0;
	}

	//im->naiveScale();

	im->wardTone();
	//im->photoTone(100);
	//im->histogramAdjustment();
	im->deviceScale();

	

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
	int id = -5;
	while (!done) {
		while (SDL_PollEvent(&event)) {
			if (event.type == SDL_QUIT)
			{
				done = true;
				break;
			}
			if (event.type == SDL_MOUSEBUTTONDOWN) {
				int nid = im->getId(event.button.x, event.button.y);
				if (nid == id)
					id = -5;
				else
					id = nid;
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