//
// Game Objects for ThunkTS
//

#ifndef ThunkTS_Entities_h
#define ThunkTS_Entities_h

//Debug information
//#define EXPLIC

#include <iostream>

#include "Board.h"
#include "Object.h"

//Abstract parent class of all entities
class Entity {
private:
	static int id; //Allows each entity to have a unique id
protected:
	Board * b; //The Board the Entity is associated with
	int pin; //Unique id for individual entities
	int x; //x-coordinate
	int y; //y-coordinate
	int z; //z-coordinate not currently in use
	Direction lastDirection = none; //Last movement. Used for dynamic manipulation of movement by triggers
	Object* model;
public:
	Entity(Board* b); //Constructor, sets pin and board
	static void reset(); //Resets id field;
	virtual ~Entity(); //Destructor
	void markDead(); //Pseudo destructor
	int getID(); //Returns the pin
	int getX(); //returns x coordinate
	int getY(); //returns y coordinate
	Direction getDirection(); //returns lastDirection
	void move(int a, int b, Direction d = none); //Sets new coordinates and direction, should only be done by board
	virtual char getChar() = 0; //Returns a character that can represent this entity on a textual board
	virtual Object* getModel() { return model; } //For GUI
	virtual bool isSolid(Entity* e) = 0; //Returns true if the entity acts as a wall to e
	virtual void passedOver(Entity* e) = 0; //Function called when e steps on this entity
	virtual void triggered(Entity* e) = 0; //Function called when e runs into this entity, should only matter if solid to e
	virtual void onInput(Direction i) = 0; //Functions called when an input is given
	virtual bool isContainer() { return false; }; //True only for EntityLists
	virtual bool isPlayer() { return false; }; //True only for Players
	virtual bool isHole() { return false; }; //True only for Holes
};

//Serves to allow multiple entities in the same location
class EntityList : public Entity {
private:
	std::vector<Entity*> list;
public:
	EntityList(Board* b) : Entity(b) {};
	virtual ~EntityList(); //Destroys all the entites within as well
	void add(Entity* e); //Adds the entity to the list
	bool isEmpty(); //Returns true if it contains no entities
	bool contains(int pin); //Returns true if it contains the entity with the given pin
	bool contains(Entity* e); //Returns true if it contains the entity with a matching pin
	Entity* remove(int pin); //Removes the entity with the given pin, returning it
	Entity* remove(Entity* e); //As above, but matches pin with e's pin
	Entity* get(int pin); //Returns the entity that matches the pin, or nullptr
	virtual char getChar(); // Returns the character of the first entity
	Object* getModel(); // Returns the model of the first entity
	virtual bool isSolid(Entity* e); //Returns true if any of the entities listed are solid
	virtual void passedOver(Entity* e); //Calls this function on all Entities within
	virtual void triggered(Entity* e); //Calls this function on all Entities within
	virtual void onInput(Direction i); //Calls this function on all Entities within
	virtual bool isContainer() { return true; }; //True only for EntityLists
};

//Classic Entities (Those that exist on the default board)

//Players are controled by the players. They are solid, but move with input
class Player : public Entity {
public:
	Player(Board* b);
	virtual char getChar(); // depends on which Player this is, which depends on the pin
	virtual bool isSolid(Entity* e); //Always true
	virtual void passedOver(Entity* e); //Nothing happens
	virtual void triggered(Entity* e); //Nothing happens
	virtual void onInput(Direction i); //Moves if it's its turn to move
	virtual bool isPlayer() { return true; }; //True only for players
};


//Blanks are unreactive entities
class Blank : public Entity {
public:
	Blank(Board* b) : Entity(b) {};
	virtual char getChar(); // ' ' character
	virtual bool isSolid(Entity* e); //Always false
	virtual void passedOver(Entity* e); //Nothing happens
	virtual void triggered(Entity* e); //Nothing happens
	virtual void onInput(Direction i); //Nothing happens
};

//Walls simply prevent movement into the space they occupy by any other entity
class Wall : public Entity {
public:
	Wall(Board* b);
	virtual char getChar(); //# character
	virtual bool isSolid(Entity* e); //Always true
	virtual void passedOver(Entity* e); //Nothing happens
	virtual void triggered(Entity* e); //Nothing happens
	virtual void onInput(Direction i); //Nothing happens
};

//Holes destroy any entity to pass over them
class Hole : public Entity {
public:
	Hole(Board* b) : Entity(b) {};
	virtual char getChar(); //X character
	virtual bool isSolid(Entity* e); //Always false
	virtual void passedOver(Entity* e); //Deletes entity e
	virtual void triggered(Entity* e); //Nothing happens
	virtual void onInput(Direction i); //Nothing happens
	virtual bool isHole() { return true; }; //True only for Holes
};



//Extraneous Entities (Not in default board)

//Panels, when passedOver, alter an entities trajectory
//Panels with the 'dead' Direction are effectively holes
class Panel : public Entity {
private:
	Direction dir;
public:
	Panel(Board* b, Direction d); //Creates a panel that forces a specific
	virtual char getChar(); //Varies based on dir
	virtual bool isSolid(Entity* e); //Always false
	virtual void passedOver(Entity* e); //Changes the movement of e to move dir-wise
	virtual void triggered(Entity* e); //Nothing happens
	virtual void onInput(Direction i); //Nothing happens
};

//Bumpers, when hit, reverse an entities trajectory, then are destroyed
class Bumper : public Entity {
public:
	Bumper(Board* b) : Entity(b) {};
	virtual char getChar(); //& character
	virtual bool isSolid(Entity* e); //Always true
	virtual void passedOver(Entity* e); //Nothing happens
	virtual void triggered(Entity* e); //Reverses the trajectory of e then self-terminates
	virtual void onInput(Direction i); //Nothing happens
};

//Fallaways, when passed over, count down. On reaching zero, they are effectively Holes
class Fallaway : public Entity {
private:
	//Number of passOvers remaining
	int count;
public:
	Fallaway(Board* b, int c); //Creates a fallaway that can allow c passOvers, max 9
	virtual char getChar(); //Varies based on count
	virtual bool isSolid(Entity* e); //Always false
	virtual void passedOver(Entity* e); //Decrements count or destroys e if at 0
	virtual void triggered(Entity* e); //Nothing happens
	virtual void onInput(Direction i); //Nothing happens
};

//Forcefields are walls that allow a specific entity through
class Forcefield : public Entity {
private:
	//The pin of the entity that may pass
	int permeable;
public:
	Forcefield(Board* b, int p); //Creates forcefield that will allow only entity p through
	virtual char getChar(); // = character
	virtual bool isSolid(Entity* e); //True for every entity but the one with the matching pin
	virtual void passedOver(Entity* e); //Nothing happens
	virtual void triggered(Entity* e); //Nothing happens
	virtual void onInput(Direction i); //Nothing happens
};
#endif