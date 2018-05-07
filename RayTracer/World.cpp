#include "stdafx.h"
#include "World.h"


CO World::spawn(Ray r, double focal, int depth)
{
	Intersection q;
	q.id = -1;
	q.d = MAX_DIST;
	q.mat = mat;
	for (int i = 0; i < n; i++) {
		Intersection e = objects[i]->intersect(r);
		if (e.d > 0) {
			q = leastPos(q, e);
		}
	}

	Light** reachable = new Light*[l];
	int numReachable = 0;

	if (q.id != -1) {
		for (int j = 0; j < l; j++) {
			Ray lightRay = Ray(q.p, lights[j]->p);
			bool reaches = true;
			for (int i = 0; i < n; i++) {
				Intersection e = objects[i]->intersect(lightRay, q.id);
				if (isPos(e)) { //Intersection of another object blocks light
					reaches = false;
					break;
				}
			}
			if (reaches) { //In the shadow
				reachable[numReachable++] = lights[j];
			}
		}
	}

	q.eye = r.d*-1;

	CO ret = {q.mat->getColor(q, reachable, numReachable, ambient), q.id};
	delete[] reachable;

	if (depth < MAX_DEPTH) {
		double reflectivity = q.mat->getReflectivity(q);
		double transparency = q.mat->getTransparency(q);
		double opacity = 1 - (reflectivity + transparency);
		ret.c = opacity * ret.c;
		if (reflectivity > 0) {
			Vector reflection = reflect(r.d, q.n).normalize();
			Ray reflectedRay = Ray(q.p+(EPSILON*2*reflection), reflection);
			Color reflectedResult = spawn(reflectedRay, EPSILON, depth + 1).c;
			ret.c = ret.c + ((reflectivity)*reflectedResult);
		}
		if (transparency > 0) {
			double index = q.mat->getIndexOfRefraction(q);
			Vector tran;
			if (index == 1.0) {
				tran = r.d; //No refraction from material
			}
			else {
				double ni = 1.0;
				double nt = index;
				if (q.inside) {
					ni = index;
					nt = 1.0;
				}
				double dot = r.d*q.n;
				Vector cross = q.n.cross(r.d);
				double refractionValue = 1-(((ni*ni)/(nt*nt))*(cross*cross));
				if (refractionValue < 0) { //total internal reflection
					tran = reflect(r.d, q.n).normalize();
				}
				else {
					tran = (ni / nt)*(q.n.cross(-1*cross)) - sqrt(refractionValue)*q.n;
				}
			}
			Ray transparencyRay = Ray(q.p + (EPSILON * 2 * tran), tran);
			Color transparencyResult = spawn(transparencyRay, EPSILON, depth + 1).c;
			ret.c = ret.c + ((transparency)*transparencyResult);
		}
	}

	return ret;
}

Camera::Camera(Point eyepoint, Point lookat, Vector up) : eyepoint(eyepoint), lookat(lookat), up(up.normalize())
{
	n = Vector(lookat, eyepoint).normalize();
	u = up.cross(n).normalize();
	v = n.cross(u);

	Vector eye = Vector(eyepoint.x, eyepoint.y, eyepoint.z); //eyepoint vector

	/*double vals[16] = {
		u.x,   v.x,   n.x,   0,
		u.y,   v.y,   n.y,   0,
		u.z,   v.z,   n.z,   0,
		-1 * (eye*u), -1 * (eye*v), -1 * (eye*n), 1
	};*/
	double vals[16] = {
		u.x, u.y, u.z, -1*(eye*u),
		v.x, v.y, v.z, -1*(eye*v),
		n.x, n.y, n.z, -1*(eye*n),
		0,   0,   0,   1
	};
	transform.setValues(vals);

	transform = transform.getInverse();
}
//n is towards camera, away from scene
//Focal is towards scene, away from camera
void Camera::render(World w, double focal, double width, double height, Image* im, int samples, PixelMask* pm)
{
	double p_width = width / im->width;
	double p_height = height / im->height;

	Point corner = Point(-width / 2 +p_width/2, height / 2 - p_height/2, -focal);
	Vector du = Vector(p_width, 0, 0);
	Vector dv = Vector(0, -p_height, 0);

	Point dHor = corner + du;
	Point dVer = corner + dv;

	corner = transform * corner;
	dHor = transform * dHor;
	dVer = transform * dVer;

	du = Vector(corner, dHor);
	dv = Vector(corner, dVer);

	Needle n(&w, im, focal, du, dv, 1);

	//For debugging purposes
	/*int x = 300;
	int y = 420;

	Point d = corner + (x*du) + (y*dv);
	Ray r = Ray(eyepoint, d);
	n.pushTask(r, x, y);*/

	for (int x = 0; x < im->width; x++) {
		for (int y = 0; y < im->height; y++) {
			if (!pm || pm->getPixelMask(x, y)){
				Point d = corner + (x*du) + (y*dv);
				Ray r = Ray(eyepoint, d);
				n.pushTask(r, x, y);
			}
		}
	}

	n.run();

	//Super sampling
	//First run is necessary to calculate edges to optimize supersampling
	if (samples > 1) {
		srand((unsigned)time(NULL));
		for (int x = 0; x < im->width; x++) {
			for (int y = 0; y < im->height; y++) { //If EDGE_ONLY_SUPER is true, only super sample edges of objects
				if ((!EDGE_ONLY_SUPER || im->isEdge(x, y)) && (!pm || pm->getPixelMask(x,y))) { 
					Point d = corner + (x*du) + (y*dv);
					Ray r = Ray(eyepoint, d);
					n.pushTask(r, x, y);
				}
			}
		}
		n.superSampleRun(samples);
	}
}

void Camera::render(World w, double focal, double width, double height, Image* im, PixelMask* pm) {
	render(w, focal, width, height, im, 1, pm);
}


void Camera::renderPixelMask(GameBoard* gb, double focal, double width, double height, PixelMask * pm)
{
	double p_width = width / pm->width;
	double p_height = height / pm->height;

	Point corner = Point(-width / 2 + p_width / 2, height / 2 - p_height / 2, -focal);
	Vector du = Vector(p_width, 0, 0);
	Vector dv = Vector(0, -p_height, 0);

	Point dHor = corner + du;
	Point dVer = corner + dv;

	corner = transform * corner;
	dHor = transform * dHor;
	dVer = transform * dVer;

	du = Vector(corner, dHor);
	dv = Vector(corner, dVer);

	Needle n(gb, pm);

	for (int x = 0; x < pm->width; x++) {
		for (int y = 0; y < pm->height; y++) {
			Point d = corner + (x*du) + (y*dv);
			Ray r = Ray(eyepoint, d);
			n.pushTask(r, x, y);
		}
	}

	n.pixelMaskRun();
}


void Needle::pushTask(Ray r, int x, int y)
{
	Task* t = new Task;
	t->r = r;
	t->x = x;
	t->y = y;
	tasks.push(t);
}

void Needle::run()
{
	for (int i = 0; i < numThreads; i++) {
		threads[i] = std::thread(executeTask, this);
	}
	for (int i = 0; i < numThreads; i++) {
		threads[i].join();
	}
}

void Needle::superSampleRun(int samples)
{
	for (int i = 0; i < numThreads; i++) {
		threads[i] = std::thread(executeSuperSample, this, samples);
	}
	for (int i = 0; i < numThreads; i++) {
		threads[i].join();
	}
}

void Needle::pixelMaskRun()
{
	for (int i = 0; i < numThreads; i++) {
		threads[i] = std::thread(executeMaskTask, this);
	}
	for (int i = 0; i < numThreads; i++) {
		threads[i].join();
	}
}

Task* Needle::getTask()
{
	Task* t = nullptr;
	mu.lock();
	if (!tasks.empty()) {
		t = tasks.front();
		tasks.pop();
	}
	mu.unlock();
	return t;
}

//Grab a task from the queue, repeat until no more tasks
void executeTask(Needle* n)
{
	Task* t = n->getTask();
	while (t) {
		CO co = n->w->spawn(t->r, n->d);
		n->im->setPixel(t->x, t->y, co.c, co.id);
		delete t;
		t = n->getTask();
	}
}

void executeSuperSample(Needle * n, int samples)
{
	Color* color = new Color[samples];

	Task* t = n->getTask();
	while (t) {
		color[0] = n->im->getPixel(t->x, t->y);

		for (int i = 1; i<samples; i++) {
			double scatterX = ((double)rand() / (double)RAND_MAX) - 0.5;
			double scatterY = ((double)rand() / (double)RAND_MAX) - 0.5;
			Ray r = Ray(t->r.o, t->r.d + scatterX * n->du + scatterY * n->dv);
			color[i] = n->w->spawn(r, n->d).c;
		}

		Color c = average(color, samples);
		n->im->setPixel(t->x, t->y, c);

		delete t;
		t = n->getTask();
	}
	

	delete[] color;
}

void executeMaskTask(Needle * n)
{
	Task* t = n->getTask();
	while (t) {
		bool* m = n->gb->whichIntersect(t->r);
		n->pm->setPixelMask(t->x, t->y, m);
		delete t;
		t = n->getTask();
	}
}
