#include "Entities.h"

//Entity Abstract Base Class

//First ID is alway zero
int Entity::id = 0;

//Resets the id field
void Entity::reset() {
	id = 0;
}

//Sets the id on constuction
Entity::Entity(Board* b) {
	this->b = b;
	pin = id++;
	x = -1;
	y = -1;
#ifdef EXPLIC
	std::cout << "Entity " << pin << " created.\n";
#endif
}

//Sets id to deleted
Entity::~Entity() {
#ifdef EXPLIC
	std::cout << "Entity " << pin << " destroyed.\n";
#endif
	markDead();
}

//Psuedo deconstructor to allow an entity to be 'killed' from anywhere.
void Entity::markDead() {
	x = -1;
	y = -1;
	pin = -1;
}

//Accessor for the pin
int Entity::getID() {
	return pin;
}

//Gets the x coordinate
int Entity::getX() {
	return x;
}

//Gets the y coordinate
int Entity::getY() {
	return y;
}

Direction Entity::getDirection()
{
	return lastDirection;
}

//Sets the x and y coordinates
//Should only be called by the board
void Entity::move(int a, int b, Direction d) {
	x = a;
	y = b;
	lastDirection = d;
}

//EntityLists allow multiple entities in the same position
EntityList::~EntityList() {
	while (list.size()>0) {
		Entity* e = list.back();
		list.pop_back();
		delete e;
	}
}

//Adds an entity to the list
void EntityList::add(Entity* e) {
	list.push_back(e);
}

//Returns whether the list contains no entities
bool EntityList::isEmpty() {
	return list.empty();
}

//Returns whether an entity with the given pin is contained
bool EntityList::contains(int pin) {
	auto it = list.begin();
	for (; it != list.end(); it++) {
		if ((*it)->getID() == pin) {
			return true;
		}
	}
	return false;
}

//Returns whether it contains an entity that shares a pin with e
//Fails on entites marked dead
bool EntityList::contains(Entity *e) {
	if (e->getID() == -1)
		return false;
	return contains(e->getID());
}

//Returns the entity contained with the matching pin, or nullptr if it is not contained
Entity* EntityList::get(int pin) {
	auto it = list.begin();
	for (; it != list.end(); it++) {
		Entity* e = *it;
		if (e->getID() == pin) {
			return e;
		}
	}
	return nullptr;
}

//Removes an entity from the list that has the given pin and returns it
//Returns nullptr if no such element existed
Entity* EntityList::remove(int pin) {
	Entity* r = nullptr;
	auto it = list.begin();
	for (; it != list.end(); it++) {
		if ((*it)->getID() == pin) {
			r = *it;
			break;
		}
	}
	if (r != nullptr)
		list.erase(it);
	return r;
}

//Removes an entity from the list that shares a pin with e
//Returns nullptr if no such element existed or if e has been marked dead
Entity* EntityList::remove(Entity* e) {
	if (e->getID() != -1)
		return remove(e->getID());
	return nullptr;
}

//Entity Lists return the character of the last entity
char EntityList::getChar() {
	return list.back()->getChar();
}

Object * EntityList::getModel()
{
	return list.back()->getModel();
}

//Returns true if any of the entites contained are solid
bool EntityList::isSolid(Entity *e) {
	for (std::vector<Entity*>::iterator it = list.begin(); it != list.end(); ++it) {
		if ((*it)->isSolid(e))
			return true;
	}
	return false;
}

//Calls passedOver on all elements within
void EntityList::passedOver(Entity *e) {
	for (std::vector<Entity*>::iterator it = list.begin(); it != list.end(); ++it) {
		(*it)->passedOver(e);
	}
}

//Calls triggered on all elements within
void EntityList::triggered(Entity *e) {
	for (std::vector<Entity*>::iterator it = list.begin(); it != list.end(); ++it) {
		(*it)->triggered(e);
	}
}

//Calls onInput on all elements within
void EntityList::onInput(Direction i) {
	for (std::vector<Entity*>::iterator it = list.begin(); it != list.end(); ++it) {
		(*it)->onInput(i);
	}
}

bool EntityList::isHole()
{
	for (std::vector<Entity*>::iterator it = list.begin(); it != list.end(); ++it) {
		if ((*it)->isHole())
			return true;
	}
	return false;
}

Material* Player1Mat = new PhongMaterial(BLUE);
Material* Player2Mat = new PhongMaterial(RED);

Player::Player(Board * b) : Entity(b)
{
	Material* m = Player1Mat;
	if (pin % 2) {
		m = Player2Mat;
	}
	model = new Sphere(Point(0, 0.5, 0), 0.4, m);
	b->addPlayer(this);
}

//Player Entites
//Players are represented by a captial letter on the textual board
char Player::getChar() {
	return 'A' + pin;
}

//Players are solid
bool Player::isSolid(Entity* e) {
	return true;
}

//Players do not react to being passed over
void Player::passedOver(Entity* e) {
	return;
}

//Players do not activate when hit by something
void Player::triggered(Entity* e) {
	return;
}

//Players request to go in the direction of input on their turn in the move phase
void Player::onInput(Direction i) {
	if (b->getTurn() == pin && b->getPhase() == Phase::move)
		b->requestMove(this, i);
	return;
}



//Blank Entities are empty space

//' ' represents blanks on the textual board
char Blank::getChar() {
	return ' ';
}

//Blanks are not solid
bool Blank::isSolid(Entity* e) {
	return false;
}

//Blanks do not react to being passed over
void Blank::passedOver(Entity* e) {
	return;
}

//Blanks do not activate when hit by something
void Blank::triggered(Entity* e) {
	return;
}

//Blanks do not respond to user input
void Blank::onInput(Direction i) {
	return;
}





//Wall Entities prevent movement into their space and nothing more

Material* WallMat = new PhongMaterial(Color(.75, .75, .75));

Wall::Wall(Board * b) : Entity(b)
{
	model = new Cylinder(Point(0, 0.1, 0), Point(0, 0.9, 0), 0.4, WallMat);
}

//# represents walls on the textual board
char Wall::getChar() {
	return '#';
}



//The Wall is universally solid
bool Wall::isSolid(Entity* e) {
	return true;
}

//Walls cannot be passed over, and wouldn't react regardless
void Wall::passedOver(Entity* e) {
	return;
}

//Walls do not activate when hit by something
void Wall::triggered(Entity* e) {
	return;
}

//Walls do not respond to user input
void Wall::onInput(Direction i) {
	return;
}





//Hole Entities destroy anything that moves into their space

//X represents holes on the textual board
char Hole::getChar() {
	return 'X';
}

//The Hole does not prevent movement into itself
bool Hole::isSolid(Entity* e) {
	return false;
}

//Holes destroy objects that passOver them
void Hole::passedOver(Entity* e) {
	b->insistMove(e, dead);
	return;
}

//Holes do not activate when hit by something
void Hole::triggered(Entity* e) {
	return;
}

//Holes do not respond to user input
void Hole::onInput(Direction i) {
	return;
}




//Panel Entities change the trajectory anything that moves into their space
Panel::Panel(Board* b, Direction d) : Entity(b) {
	//Force it to be a turning direction, rather than a moving direction
	if (d == up)
		d = upT;
	if (d == down)
		d = downT;
	if (d == left)
		d = leftT;
	if (d == right)
		d = rightT;
	dir = d;
}

//The character representation of panels varies
//With the limititation of ASCII graphics, not all Panels can be properly displayed
char Panel::getChar() {
	switch (dir) {
	case upT:
		return '^';
	case downT:
		return 'v';
	case rightT:
		return '>';
	case leftT:
		return '<';
	case clockwise:
		return ')';
	case counterclockwise:
		return '(';
	case reverse:
		return '!';
	case dead:
		return 'X';
	default:
		return '$';
	}
}

//Panels do not prevent movement into itself
bool Panel::isSolid(Entity* e) {
	return false;
}

//Panels change the trajectory of objects that passOver them
void Panel::passedOver(Entity* e) {
	b->insistMove(e, dir);
	return;
}

//Panels do not activate when hit by something
void Panel::triggered(Entity* e) {
	return;
}

//Panels do not respond to user input
void Panel::onInput(Direction i) {
	return;
}





//Bumpers reverse movement then self terminate
//& represents walls on the textual board
char Bumper::getChar() {
	return '&';
}

//The Bumper is universally solid
bool Bumper::isSolid(Entity* e) {
	return true;
}

//Bumper cannot be passed over, and wouldn't react regardless
void Bumper::passedOver(Entity* e) {
	return;
}

//Bumpers reverse trajectory when hit, then autotermiante
void Bumper::triggered(Entity* e) {
	Direction d = e->getDirection();;
	if (d == up)
		d = down;
	else if (d == right)
		d = left;
	else if (d == left)
		d = right;
	else if (d == down)
		d = up;
	b->insistMove(this, dead);
	b->requestMove(e, d);
	return;
}

//Bumpers do not respond to user input
void Bumper::onInput(Direction i) {
	return;
}




//Fallaway Entities act as blanks until they have been passed over c times
//Then they act as holes
//C cannot be greater than 9
Fallaway::Fallaway(Board* b, int c) : Entity(b) {
	count = c;
	if (count > 9)
		count = 9;
	if (count < 0)
		count = 0;
}

//Fallaways are represented by the count remaining, or by X at zero
//X represents holes on the textual board
char Fallaway::getChar() {
	if (count == 0)
		return 'X';
	return '0' + count;
}

//The Fallaway does not prevent movement into itself
bool Fallaway::isSolid(Entity* e) {
	return false;
}

//Fallaways decrement until reaching zero, after which destroy objects that passOver them
void Fallaway::passedOver(Entity* e) {
	if (count == 0)
		b->insistMove(e, dead);
	else {
		count--;
	}
	return;
}

//Fallaways do not activate when hit by something
void Fallaway::triggered(Entity* e) {
	return;
}

//Fallaways do not respond to user input
void Fallaway::onInput(Direction i) {
	return;
}



//Forcefield Entities act as walls except to a single entity (generally a player)
Forcefield::Forcefield(Board* b, int p) : Entity(b) {
	permeable = p;
}

//= represents Forcefields on the textual board
//This is a limitation that prevents obvious indication of who can pass through the field
char Forcefield::getChar() {
	return '=';
}

//The Forcefield is solid to all except the entity whose pin matches the stored value
bool Forcefield::isSolid(Entity* e) {
	return !(e->getID() == permeable);
}

//Nothing happens when passedOver
void Forcefield::passedOver(Entity* e) {
	return;
}

//Forcefields do not activate when hit by something
void Forcefield::triggered(Entity* e) {
	return;
}

//Forcefields do not respond to user input
void Forcefield::onInput(Direction i) {
	return;
}
