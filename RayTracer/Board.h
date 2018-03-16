//
// Board object and game logic for ThunkTS
//

#ifndef ThunkTS_Board_h
#define ThunkTS_Board_h

//Enforces place
//#define STRICT

// Puts Horizontal bars on textual print
//#define BARS

#include <iostream>
#include <fstream>
#include <map>
#include <vector>

//All possible directions, not all implemented
enum Direction { up, down, left, right, upT, downT, leftT, rightT, reverse, clockwise, counterclockwise, stop, dead, inward, outward, blank, none };

//All possible move phases, not all implemented
//End phase is reserved for when the game is not in play
//Next phase is reserved to go to the next player and restart the loop
enum Phase { end, place, move, destroy, placePanel, chaos, next };

class Entity;

//Helper class
//Keeps track of which action to do next
//Allows actions to be placed on the top of the stack or at the bottom
class QueueStack {
private:
	std::vector<std::pair<Entity*, Direction>> stack;
public:
	bool isEmpty(); //True if stack is empty
	void push(Entity* e, Direction d); //Add an action to the top of the stack as an interupt
	void queue(Entity* e, Direction d); //Adds an action to the end of the queue
	std::pair<Entity*, Direction> pop(); //Removes and returns the top action
	std::pair<Entity*, Direction> peak(); //Returns the top action without removing it
	bool remove(int pin);//Removes all pairs that involve the enitity with id pin
	bool remove(Entity* e);//Removes all pairs that involve the entity that shares pin with e
	Direction edit(int pin, Direction d); //Edits the direction of the first matching pair
	Direction edit(Entity* e, Direction d); //Edits the direction of the first matching pair
};

class Board {
private:
	//Storage of all Entities
	std::map<std::pair<int, int>, Entity*> map;

	//Number of players on this map
	int numPlayers;

	//Numbers of rows and columns
	int ROWS;
	int COLS;

	//Keeps track of whose turn it is
	int turn;

	//Keeps track of what phase within a turn it is currently
	int current;

	//Keeps track of the order of the phases
	//Defauls to [place,move,next], but can be altered with the gameboard
	//Phase orders can have up to ten phases as their sequence, but must end with 'next'
	//Phase orders should not contain 'end'
	Phase order[11] = {};

	//Variable allows for complicated interactions from move requests
	//Such as 'stop moving immediately, or alter direction
	//Also allows mutliple entites to 'simultaneously' act by breaking up move actions
	QueueStack qs;

public:
	Board() : turn(-1), current(-1) {};
	~Board(); //Destructure manages memory
	int getTurn(); //Returns whose turn it is
	Phase getPhase(); //Returns the current Phase
	Entity* getEntity(int x, int y); //Returns the entity in positon x,y or nullptr if none
	bool makeBoard(int i); //Creates a board from default options, returns if successfull
	Entity* pull(Entity* e, int x, int y); //Grabs the entity from its location on the map
	void place(Entity* e, int x, int y, Direction d = none); //Puts an entity into a given location with the given direction
	void move(Entity* e, int x1, int y1, int x2, int y2, Direction d = none); //Moves an entity from pos1 to pos2, stores direction

	void start(); //Starts the game
	void requestMove(Entity* e, Direction d); //Asks the board to move Entity e in direcion d when possible
	void insistMove(Entity* e, Direction d); //Asks the board to move Entity e in direcion d immediately
	Phase next(); //Goes to next phase of the turn, or next turn
	void input(Direction d); //Takes user input and acts on it
	bool drop(int x, int y); //Creates a block in position x,y if possible, returns true if so
	bool act(); //Pops the stack and follows its action. Returns true if stack is empty
	bool isDead(int p); //Returns true if player p is stuck or marked dead
	bool isLegal(Direction d); //Returns true is the active player can move d-wise
	void print(); //displays board to console

};

#endif