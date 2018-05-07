#include "stdafx.h"
#include "Util.h"


Point::Point(const Point & p)
{
	x = p.x;
	y = p.y;
	z = p.z;
}

double Point::distance(Point p)
{
	double dx = p.x - x;
	double dy = p.y - y;
	double dz = p.z - z;
	return sqrt(dx*dx + dy*dy + dz*dz);
}

Point Point::transform(double dx, double dy, double dz)
{
	return Point(x + dx, y + dy, z + dz);
}

Vector reflect(Vector incident, Vector normal)
{
	return incident - (2*normal*(incident*normal));
}

Point operator+(Point p, Vector v) {
	return Point(p.x + v.x, p.y + v.y, p.z + v.z);
}

Color operator*(double d, Color c)
{
	return Color(d*c.r, d*c.g, d*c.b);
}

Color average(Color * colors, int num)
{
	double r = 0.0;
	double g = 0.0;
	double b = 0.0;
	for (int i = 0; i < num; i++) {
		r += colors[i].r;
		g += colors[i].g;
		b += colors[i].b;
	}
	return Color(r/num, g/num, b/num);
}

Vector::Vector(Point a, Point b)
{
	x = b.x - a.x;
	y = b.y - a.y;
	z = b.z - a.z;
}

Vector::Vector(const Vector &v)
{
	x = v.x;
	y = v.y;
	z = v.z;
}

Vector Vector::operator+(Vector v)
{
	return Vector(x+v.x, y+v.y, z+v.z);
}

Vector Vector::operator-(Vector v)
{
	return Vector(x - v.x, y - v.y, z - v.z);
}

Vector Vector::operator*(double d)
{
	return Vector(x*d, y*d, z*d);
}

Vector Vector::operator/(double d)
{
	return Vector(x/d, y/d, z/d);
}

double Vector::operator*(Vector v)
{
	return x*v.x + y*v.y + z*v.z;
}

Vector Vector::cross(Vector v)
{
	return Vector((y*v.z)-(z*v.y),(z*v.x)-(x*v.z),(x*v.y)-(y*v.x));
}

double Vector::selfDot()
{
	return x * x + y * y + z * z;
}

double Vector::length()
{
	return sqrt(x*x + y*y + z*z);
}

Vector Vector::normalize()
{
	return Vector(x,y,z)/length();
}

std::ostream & operator<<(std::ostream & os, Point const & p)
{
	return os << "(" << p.x << ", " << p.y << ", " << p.z << ")";
}

Vector operator*(double s, Vector v)
{
	return Vector(s*v.x, s*v.y, s*v.z);
}

std::ostream & operator<<(std::ostream & os, Vector const & v)
{
	return os << "<" << v.x << ", " << v.y << ", " << v.z << ">";
}

void Image::scale(double sf)
{
	mu.lock();
	for (int i = 0; i < width*height; i++)
		pixels[i] = pixels[i] * sf;
	mu.unlock();
}

void Image::naiveScale()
{
	scale(1 / maxIntensity);
	maxIntensity = 1;
}

void Image::wardTone()
{
	double sf = pow(ldMax / 2, 0.4);
	sf += 1.219;
	sf /= (1.219 + pow(getLogAverageLuminance(), 0.4));
	sf = pow(sf, 2.5);
	scale(sf);
}

void Image::photoTone()
{
	photoTone(getLogAverageLuminance());
}

void Image::photoTone(double key)
{
	mu.lock();
	double sf = 0.18 / key;
	for (int i = 0; i < width*height; i++){
		pixels[i] = pixels[i] * sf; //Map the key to middle luminance
		pixels[i] = Color(pixels[i].r / (1 + pixels[i].r), pixels[i].g / (1 + pixels[i].g), pixels[i].b / (1 + pixels[i].b)); //Reflectence
		pixels[i] = pixels[i] * ldMax; //Illuminate the film with bright white light
	}
	mu.unlock();
}

void Image::photoTone(int x, int y)
{
	photoTone(getPixel(x, y).getIlluminance());
}

void Image::deviceScale()
{
	scale(1 / ldMax);
}

void Image::histogramAdjustment(bool applyCeiling)
{
	mu.lock();
	//int s = (int)floor(2 * tan(0.5) / 0.01745); //Pixel length for one degree solid angle
	int s = 62;

	double* hist = new double[100];
	double* histZone = new double[100];

	double logMax = -1000;
	double logMin = 1000;
	for (int i = 0; i < width*height; i++) {
		double logL = log(EPSILON + pixels[i].getIlluminance());
		logMax = maximum(logMax, logL);
		logMin = minimum(logMin, logL);
	}

	for (int i = 0; i < 100; i++) {
		hist[i] = 0;
		histZone[i] = (logMax - logMin) * ((i + 1) / 100.0) + logMin;
	}
	int T = 0; //Number of samples

	//Get histogram samples of average log illuminance
	for (int i = s / 2; i < width - (s / 2); i += (s / 16)) {
		for (int j = s / 2; j < height - (s / 2); j += (s / 16)) {
			T += 1;

			double average = 0.0;
			int U = 0;
			for (int x = 0; x <= s; x++) {
				for (int y = 0; y <= s; y++) {
					U += 1;
					average += log(EPSILON + getPixel(i - (s / 2) + x, j - (s / 2) + y).getIlluminance());
				}
			}
			average /= U;

			int q = 0;
			while (average >= histZone[q]) {
				q++;
				if (q == 99)
					break;
			}
			hist[q]++;
		}
	}

	//Histogram Ceiling
	if (applyCeiling){
		double linScale = ((logMax - logMin) / 100.0) / (log(ldMax) + 4);
		while (true) {
			double ceiling = T * linScale;

			//No more than 2.5% of samples may exceed ceiling

			int overcount = 0;
			for (int i = 0; i < 100; i++) {
				if (hist[i] > ceiling)
					overcount += (hist[i] - ceiling);
			}

			if (overcount <= T * 0.025) //Passes the test
				break;

			for (int i = 0; i < 100; i++) {
				if (hist[i] > ceiling) { //Clamp and restart, since T changes
					int dif = hist[i] - ceiling;
					hist[i] = ceiling;
					T -= dif;
					continue;
				}
			}
		}
	}

	//Get cumulative distribution
	for (int i = 1; i < 100; i++) {
		hist[i] += hist[i - 1];
	}
	for (int i = 1; i < 100; i++) {
		hist[i] /= T;
	}

	double scale = log(ldMax) + 4; //Log minimum luminance is -4

	for (int i = 0; i < width*height; i++) {
		double logLW = log(EPSILON + pixels[i].getIlluminance());
		int q = 0;
		while (logLW >= histZone[q]) {
			q++;
			if (q == 99)
				break;
		}

		double logLD = scale * hist[q] - 4;
		pixels[i] = pixels[i] * (exp(logLD));
	}
	delete[] hist;
	delete[] histZone;
	mu.unlock();
}

void Image::setPixel(int x, int y, Color c, int id)
{
	mu.lock();
	pixels[x + y * width] = c;
	ids[x + y * width] = id;
	double m = maximum(c.r, c.g, c.b);
	maxIntensity = m > maxIntensity ? m : maxIntensity;
	mu.unlock();
}

Color Image::getPixel(int x, int y)
{
	return pixels[x + y * width];
}

int Image::getId(int x, int y)
{
	return ids[x + y * width];
}

//Compare against all adjacent and orthogonal pixels
//If any are from different objects, return true
bool Image::isEdge(int x, int y)
{
	int c = ids[x + y * width];

	if (x > 0) {
		if(c != ids[x - 1 + y * width])
			return true;
		if (y > 0 && c != ids[x - 1 + (y - 1) * width])
			return true;
		if (y < height - 1 && c != ids[x - 1 + (y + 1) * width])
			return true;
	}
	if (y > 0 && c != ids[x + (y - 1) * width])
		return true;
	if (y < height - 1 && c != ids[x + (y + 1) * width])
		return true;
	if (x < width-1) {
		if (c != ids[x + 1 + y * width])
			return true;
		if (y > 0 && c != ids[x + 1 + (y - 1) * width])
			return true;
		if (y < height - 1 && c != ids[x + 1 + (y + 1) * width])
			return true;
	}

	return false;
}

double Image::getLogAverageLuminance()
{
	double l = 0;
	for (int i = 0; i < width*height; i++)
		l += log(EPSILON + pixels[i].getIlluminance());
	l /= (width*height);
	l = exp(l);
	return l;
}

Point Ray::pointAt(double dist)
{
	return o + (d*dist);
}

double Ray::distTo(double q, axis ax)
{
	switch (ax) {
		case(xAxis):
			if (o.x == q)
				return 0;
			if (d.x == 0)
				return -1;
			return (q - o.x) / d.x;
		case(yAxis):
			if (o.y == q)
				return 0;
			if (d.y == 0)
				return -1;
			return (q - o.y) / d.y;
		case(zAxis):
			if (o.z == q)
				return 0;
			if (d.z == 0)
				return -1;
			return (q - o.z) / d.z;
	}
	return -1.0;
}

Color Color::operator+(Color c)
{
	return Color(r+c.r, g+c.g, b+c.b);
}

Color Color::operator*(double d)
{
	return Color(r*d, g*d, b*d);
}

Color Color::operator*(Color c)
{
	return Color(r * c.r, g * c.g, b * c.b);
}

double Color::getIlluminance()
{
	return 0.27*r + 0.67*g + 0.06*b;
}

Color PhongMaterial::getColor(Intersection in, Light ** lights, int numLights, Color ambientLight)
{
	Color amb = base * ambientLight;

	Color dif = Color(0, 0, 0);

	for (int i = 0; i < numLights; i++) {
		Color q = (base*lights[i]->c)*(Vector(in.p, lights[i]->p).normalize()*in.n);
		if (q.r >= 0 && q.g >= 0 && q.b >= 0)
			dif = dif + q;
	}

	Color spec = Color(0, 0, 0);

	for (int i = 0; i < numLights; i++) {
		Vector r = reflect(Vector(lights[i]->p, in.p).normalize(), in.n);
		double factor = (r*in.eye);
		if(factor > 0)	//Avoid specular edge with even exponents
			spec = spec + (specular*lights[i]->c)*(pow(factor, ke));
	}

	return (ka * amb) + (kd * dif) + (ks * spec);
}

Color BlinnMaterial::getColor(Intersection in, Light ** lights, int numLights, Color ambientLight)
{
	Color amb = base * ambientLight;

	Color dif = Color(0, 0, 0);

	for (int i = 0; i < numLights; i++) {
		Color q = (base*lights[i]->c)*(Vector(in.p, lights[i]->p).normalize()*in.n);
		if (q.r >= 0 && q.g >= 0 && q.b >= 0)
			dif = dif + q;
	}

	Color spec = Color(0, 0, 0);

	for (int i = 0; i < numLights; i++) {
		Vector h = (Vector(in.p, lights[i]->p).normalize() + in.eye).normalize();
		Color q = (specular*lights[i]->c)*(pow((h*in.n), ke));
		if (q.r >= 0 && q.g >= 0 && q.b >= 0)
			spec = spec + q;
	}

	return (ka * amb) + (kd * dif) + (ks * spec);
}

double Checkerboard::getReflectivity(Intersection in)
{
	double u = in.tex.x;
	double v = in.tex.y;

	u *= n;
	v *= n;

	int i = (int)(floor(u));
	int j = (int)(floor(v));

	if ((i + j) % 2) {
		return b->getReflectivity(in);
	}
	else {
		return a->getReflectivity(in);
	}
}

double Checkerboard::getTransparency(Intersection in)
{
	double u = in.tex.x;
	double v = in.tex.y;

	u *= n;
	v *= n;

	int i = (int)(floor(u));
	int j = (int)(floor(v));

	if ((i + j) % 2) {
		return b->getTransparency(in);
	}
	else {
		return a->getTransparency(in);
	}
}

double Checkerboard::getIndexOfRefraction(Intersection in)
{
	double u = in.tex.x;
	double v = in.tex.y;

	u *= n;
	v *= n;

	int i = (int)(floor(u));
	int j = (int)(floor(v));

	if ((i + j) % 2) {
		return b->getIndexOfRefraction(in);
	}
	else {
		return a->getIndexOfRefraction(in);
	}
}

Color Checkerboard::getColor(Intersection in, Light ** lights, int numLights, Color ambientLight)
{
	double u = in.tex.x;
	double v = in.tex.y;

	u *= n;
	v *= n;

	int i = (int)(floor(u));
	int j = (int)(floor(v));

	if ((i + j) % 2) {
		return b->getColor(in, lights, numLights, ambientLight);
	}
	else {
		return a->getColor(in, lights, numLights, ambientLight);
	}
}

Color ImageTexture::getColor(Intersection in, Light ** lights, int numLights, Color ambientLight)
{
	double u = in.tex.x;
	double v = in.tex.y;

	u *= 1-im->width;
	v *= im->height;

	int i = im->width - (int)(floor(u)) - 1;
	int j = im->height - (int)(floor(v)) - 1;

	Color c = im->getPixel(i, j);

	PhongMaterial* m = new PhongMaterial(c);
	c = m->getColor(in, lights, numLights, ambientLight);
	delete m;

	return c;
}

void PixelMask::setPixelMask(int x, int y, bool * m)
{
	mu.lock();
	maskSet[x + y * width] = m;
	mu.unlock();
}

void PixelMask::loadMask(int id)
{
	if (id<0 || id > n)
		return;
	mu.lock();
	for (int i = 0; i < width*height; i++) {
		if (maskSet[i][id]) {
			mask[i] = true;
		}
	}
	mu.unlock();
}

void PixelMask::resetMasks()
{
	mu.lock();
	for (int i = 0; i < width*height; i++) {
		mask[i] = false;
	}
	mu.unlock();
}

bool PixelMask::getPixelMask(int x, int y)
{
	return mask[x + y * width];
}
