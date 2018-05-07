#include "Board.h"
#include "Entities.h"

//For print debugging
const char* directionTypes[] = {
	"up",
	"down",
	"left",
	"right",
	"upT",
	"downT",
	"leftT",
	"rightT",
	"reverse",
	"clockwise",
	"counterclockwise",
	"stop",
	"dead",
	"inward",
	"outward",
	"blank",
	"none"
};

//Returns true if there are no elements left in the stack
bool QueueStack::isEmpty() {
	return stack.empty();
}

//Adds an Entity/Direction pair to the stack on top
//Used as an interuption during a move, such as with passedOver and triggered
void QueueStack::push(Entity* e, Direction d) {
	stack.insert(stack.begin(), std::make_pair(e, d));
}

//Adds an Entity/Direction pair to the stack on bottom
//Used as a continuation of a move
void QueueStack::queue(Entity *e, Direction d) {
	stack.push_back(std::make_pair(e, d));
}

//Removes and returns an Entity/Direction pair from the top of the stack
std::pair<Entity*, Direction> QueueStack::pop() {
	std::pair<Entity*, Direction> p = stack.front();
	stack.erase(stack.begin());
	return p;
}

//Returns an Entity/Direction pair from the top of the stack
std::pair<Entity*, Direction> QueueStack::peak()
{
	std::pair<Entity*, Direction> p = stack.front();
	return p;
}

//Removes all instances of entity/directions pairs whose enitity has the pin pin
//Returns true if at least one element was removed
bool QueueStack::remove(int pin) {
	bool r = false;
	auto it = stack.begin();
	while (it != stack.end()) {
		if ((*it).first && (*it).first->getID() == pin) {
			r = true;
			it = stack.erase(it);
		}
		else
			it++;
	}
	return r;
}

//Removes all instances of entity/directions pairs whose enitity share a pin with e
//Returns true if at least one element was removed
//Autofails if e had been marked dead.
bool QueueStack::remove(Entity *e) {
	if (e->getID() == -1)
		return false;
	return remove(e->getID());
}


//Changes the direction of the first pair instance whose entity has the pin pin
//Returns the old direction if such an element exists
//Returns 'blank' if not
//Does not edit if 'blank' is the direction chosen
Direction QueueStack::edit(int pin, Direction d) {
	Direction r = blank;
	auto it = stack.begin();
	for (; it != stack.end(); it++) {
		if ((*it).first && (*it).first->getID() == pin) {
			r = it->second;
			if (d != blank)
				(*it).second = d;
			break;
		}
	}
	return r;
}

//Changes the direction of the first pair whose entity shares a pin with e
//Returns the old direction if such an element exists
//Returns 'blank' if not
//Autofails if e has been marked dead
//Does not edit if 'blank' is the direction chosen
Direction QueueStack::edit(Entity *e, Direction d) {
	if (e->getID() == -1)
		return blank;
	return edit(e->getID(), d);
}

//Frees up all created Entities
Board::~Board() {
	auto it = map.begin();
	for (; it != map.end(); it++)
		delete it->second;
}

//Makes one of the default boards
//Returns true if i is an implemented board
bool Board::makeBoard(int i) {
	Entity::reset();

	/*
	   |1|2|3|4|5|6|7|8|9|
	 |#|#|#|#|#|#|#|#|#|#|#|
	1|#| | | | | | | | | |#|1
	2|#| | | | | | | |B| |#|2
	3|#| | | | | | | | | |#|3
	4|#| | | | | | | | | |#|4
	5|#| | | | |X| | | | |#|5
	6|#| | | | | | | | | |#|6
	7|#| | | | | | | | | |#|7
	8|#| |A| | | | | | | |#|8
	9|#| | | | | | | | | |#|9
	 |#|#|#|#|#|#|#|#|#|#|#|
	   |1|2|3|4|5|6|7|8|9|
	*/
	if (i == 1) {
		numPlayers = 2;

		ROWS = 11;
		COLS = 11;

		order[0] = Phase::place;
		order[1] = Phase::move;
		order[2] = Phase::next;

		Player* p1 = new Player(this);
		Player* p2 = new Player(this);

		Entity* e;

		for (int y = 0; y<ROWS; y++) {
			for (int x = 0; x<COLS; x++) {
				if (x == 0 || y == 0 || x == COLS - 1 || y == ROWS - 1) {
					e = new Wall(this);
					place(e, x, y);
				}
				if (x == 5 && y == 5) {
					e = new Hole(this);
					place(e, x, y);
				}
			}
		}

		place(p1, 2, 8);
		place(p2, 8, 2);

		return true;
	}
	/*
	   |1|2|3|4|5|6|7|8|9|
	 |X|X|X|X|X|X|X|X|X|X|X|
	1|X| | | | | | | | | |X|1
	2|X| | | | | | | |B| |X|2
	3|X| | | | | | | | | |X|3
	4|X| | | | | | | | | |X|4
	5|X| | | | |#| | | | |X|5
	6|X| | | | | | | | | |X|6
	7|X| | | | | | | | | |X|7
	8|X| |A| | | | | | | |X|8
	9|X| | | | | | | | | |X|9
	 |X|X|X|X|X|X|X|X|X|X|X|
	   |1|2|3|4|5|6|7|8|9|
	*/
	if (i == 2) {
		numPlayers = 2;

		ROWS = 11;
		COLS = 11;

		order[0] = Phase::place;
		order[1] = Phase::move;
		order[2] = Phase::next;

		Player* p1 = new Player(this);
		Player* p2 = new Player(this);

		Entity* e;

		for (int y = 0; y<ROWS; y++) {
			for (int x = 0; x<COLS; x++) {
				if (x == 0 || y == 0 || x == COLS - 1 || y == ROWS - 1) {
					e = new Hole(this);
					place(e, x, y);
				}
				if (x == 5 && y == 5) {
					e = new Wall(this);
					place(e, x, y);
				}
			}
		}

		place(p1, 2, 8);
		place(p2, 8, 2);

		return true;
	}
	/*
	   |1|2|3|4|5|6|7|8|9|
	 |#|#|#|#|#|#|#|#|#|#|#|
	1|#|X| | | | | |v| |X|#|1
	2|#| |)| | | | | |B| |#|2
	3|#|>| |9|9|9|9|9| | |#|3
	4|#| | |9|&| |&|9| | |#|4
	5|#| | |9| |!| |9| | |#|5
	6|#| | |9|&| |&|9| | |#|6
	7|#| | |9|9|9|9|9| |<|#|7
	8|#| |A| | | | | |)| |#|8
	9|#|X| |^| | | | | |X|#|9
	 |#|#|#|#|#|#|#|#|#|#|#|
	   |1|2|3|4|5|6|7|8|9|
	*/
	if (i == 3) {
		numPlayers = 2;

		ROWS = 11;
		COLS = 11;

		order[0] = Phase::move;
		order[1] = Phase::place;
		order[2] = Phase::move;
		order[3] = Phase::next;

		Player* p1 = new Player(this);
		Player* p2 = new Player(this);

		Entity* e;

		for (int y = 0; y<ROWS; y++) {
			for (int x = 0; x<COLS; x++) {
				if (x == 0 || y == 0 || x == COLS - 1 || y == ROWS - 1) {
					e = new Wall(this);
					place(e, x, y);
				}
				else if (x == 1 || y == 1 || x == COLS - 2 || y == ROWS - 2) {
					;
				}
				else if (x == 2 || y == 2 || x == COLS - 3 || y == ROWS - 3) {
					;
				}
				else if (x == 3 || y == 3 || x == COLS - 4 || y == ROWS - 4) {
					e = new Fallaway(this, 9);
					place(e, x, y);
				}
			}
		}

		place(new Panel(this, reverse), 5, 5);

		place(new Panel(this, clockwise), 2, 2);
		place(new Panel(this, clockwise), 8, 8);

		place(new Panel(this, down), 7, 1);
		place(new Panel(this, up), 3, 9);
		place(new Panel(this, right), 1, 3);
		place(new Panel(this, left), 9, 7);

		place(new Hole(this), 1, 1);
		place(new Hole(this), 9, 9);
		place(new Hole(this), 1, 9);
		place(new Hole(this), 9, 1);

		place(new Bumper(this), 4, 4);
		place(new Bumper(this), 6, 6);
		place(new Bumper(this), 4, 6);
		place(new Bumper(this), 6, 4);

		place(p1, 2, 8);
		place(p2, 8, 2);

		return true;
	}
	return false;
}

//Allows others to see whose turn it is
int Board::getTurn() {
	return turn;
}

//Returns the current phase of the turn
Phase Board::getPhase() {
	if (current == -1)
		return end;
	return order[current];
}

//Returns the entity in position x,y if it exists, nullptr otherwise
Entity* Board::getEntity(int x, int y) {
	auto it = map.find(std::make_pair(x, y));
	if (it == map.end())
		return nullptr;
	return it->second;
}

//Grabs an entity from its position on the board, or nullptr if nothing is there.
//Cannot pull entites marked dead
Entity* Board::pull(Entity *e, int x, int y) {
	if (e->getID() == -1)
		return nullptr;
	Entity* f = getEntity(x, y);
	if (f == nullptr)
		return nullptr;
	if (f->isContainer()) {
		if (((EntityList*)f)->contains(e)) {
			Entity* g = ((EntityList*)f)->remove(e);
			if (((EntityList*)f)->isEmpty()) {
				auto it = map.find(std::make_pair(x, y));
				map.erase(it);
				delete f;
			}
			g->move(-1, -1);
			return g;
		}
	}
	else {
		if (f->getID() == e->getID()) {
			auto it = map.find(std::make_pair(x, y));
			map.erase(it);
			f->move(-1, -1);
			return f;
		}
	}
	return nullptr;
}

//Puts an entity onto a given position, with the diven direction
//Autofails if e is marked dead
//Should not be used on entities that have an existing positions
//Should not be outside the map boundries
void Board::place(Entity *e, int x, int y, Direction d) {
	if (e->getID() == -1)
		return;
#ifdef STRICT
	if (e->getX() == -1 || e->getY() == -1)
		return;
	if (x<0 || y <0 || x >= COLS || y >= ROWS)
		return;
#endif
	e->move(x, y, d);
	Entity *f = getEntity(x, y);
	if (f == nullptr) {
		map[std::make_pair(x, y)] = e;
	}
	else if (f->isContainer()) {
		((EntityList*)f)->add(e);
	}
	else {
		EntityList* l = new EntityList(this);
		l->move(x, y);
		l->add(f);
		l->add(e);
		map[std::make_pair(x, y)] = l;
	}
}

//Moves entity e from x1,y1 to x2,y2, marking it as having taken Direction d
//Fails if e is marked dead, isn't in position x1,y1, or x2,y2 is solid or out of bounds
void Board::move(Entity *e, int x1, int y1, int x2, int y2, Direction d) {
	if (e->getID() == -1)
		return;
	if (x2<0 || y2 <0 || x2 >= COLS || y2 >= ROWS)
		return;
	Entity* q = pull(e, x1, y1);
	if (q == nullptr)
		return;
	Entity* w = getEntity(x2, y2);
	if (w == nullptr || !w->isSolid(e)) {
		place(e, x2, y2, d);
		//std::cout << x2 << " " << y2 << "\n";
	}
}

void Board::addPlayer(Player * p)
{
	players[playerCount++] = p;
}

Player* Board::getCurrentPlayer()
{
	return players[turn%playerCount];
}


//Readies the board to play
void Board::start() {
	turn = 0;
	current = 0;
}

//Called by entities to influence the boardState without interuption
void Board::requestMove(Entity * e, Direction d)
{
	qs.queue(e, d);
}


//Called by entities to influence the boardState as an interupt
void Board::insistMove(Entity *e, Direction d) {
	//Place a marker on the stack
	qs.push(nullptr, none);

	qs.push(e, d);

	//Act on stack until marker is reached
	std::pair<Entity*, Direction> top = qs.peak();
	while (top.first != nullptr || top.second != none) {
		act();
		if (qs.isEmpty())
			return;
		top = qs.peak();
	}

	//Remove marker from stack
	qs.pop();
}

//Move to the next phase and return it
Phase Board::next() {
	current++;
	if (order[current] == Phase::next) {
		current = 0;
		turn++;
		if (turn == numPlayers)
			turn = 0;
		int c = 0;
		while (isDead(turn)) {
			c++;
			turn++;
			if (turn == numPlayers)
				turn = 0;
			if (c>numPlayers) {
				turn = -1;
				current = -1;
				break;
			}
		}
	}
	//std::cout << current << turn;
	return order[current];
}



//Passes user input to all entities
void Board::input(Direction d) {
	auto it = map.begin();
	for (; it != map.end(); it++)
		(*it).second->onInput(d);
}



//Places a Wall entity on position x,y if it is within bounds and the positon is not solid
//Activates passedOver if it lands on something
bool Board::drop(int x, int y) {
	if (x<0 || y <0 || x >= COLS || y >= ROWS)
		return false;
	Entity* e = new Wall(this);
	Entity* w = getEntity(x, y);
	if (w == nullptr || !w->isSolid(e)) {
		place(e, x, y);
		if (w != nullptr)
			w->passedOver(e);
		return true;
	}
	else {
		delete e;
		return false;
	}
}



//Follows the action on the top of the QueueStack
//Returns true if all actions have been emptied
//Not all directions implemented
bool Board::act() {
	if (qs.isEmpty())
		return true;
	std::pair<Entity*, Direction> p = qs.peak();
	//Ignore entities marked dead
	if (p.first->getID() == -1)
		return qs.isEmpty();
	bool moving = false;
	bool altering = false;
	bool kill = false;
	switch (p.second) {
	case up:
	case down:
	case left:
	case right:
		moving = true;
		break;
	case upT:
	case downT:
	case leftT:
	case rightT:
	case reverse:
	case clockwise:
	case counterclockwise:
		altering = true;
		break;
	case stop:
	case dead:
		kill = true;
		break;
	case none:
	default:
		break;
	}

	if (moving) {
		int x = p.first->getX();
		int y = p.first->getY();
		int dx = x;
		int dy = y;
		if (p.second == up)
			dy -= 1;
		if (p.second == down)
			dy += 1;
		if (p.second == left)
			dx -= 1;
		if (p.second == right)
			dx += 1;

		//First trigger whatever would be run into
		//As this might modify the stack, a stack pointer is used
		Entity* t = getEntity(dx, dy);
		if (t != nullptr) {
			t->triggered(p.first);
		}

		//Then check if the space to enter is not solid
		if (t == nullptr || !t->isSolid(p.first))
		{
			//Then, make the move
			move(p.first, x, y, dx, dy, p.second);

			//Next, activate passedOver
			if (t != nullptr)
				t->passedOver(p.first);

			if (qs.isEmpty())
				return true;

			std::pair<Entity*, Direction> confirm = qs.peak();

			if (confirm.first != p.first) { //The entity has been removed from the stack through stop or dead
				return qs.isEmpty();
				//std::cout << confirm.first->getID() << "\n";
				//std::cout << p.first->getID() << "\n";
			}

			p = qs.pop(); //Direction might have changed, and it needs to be removed from the stack

						  //Put the next step in the queue
			qs.queue(p.first, p.second);
		}
		else {
			//Computation is finished. Pop it from the stack
			qs.pop();
		}
	}
	else {
		//Only movement might be modified mid action
		//So it is safe to pop the action off of the stack now
		qs.pop();
	}

	if (altering) {
		//Edit the stack underneath
		Direction d = qs.edit(p.first, blank);

		//Generate a new direction as necessary
		if (p.second == upT) {
			d = up;
		}
		if (p.second == rightT) {
			d = right;
		}
		if (p.second == leftT) {
			d = left;
		}
		if (p.second == downT) {
			d = down;
		}
		if (p.second == reverse) {
			if (d == up)
				d = down;
			else if (d == right)
				d = left;
			else if (d == left)
				d = right;
			else if (d == down)
				d = up;
		}
		else if (p.second == clockwise) {
			if (d == up)
				d = right;
			else if (d == right)
				d = down;
			else if (d == left)
				d = up;
			else if (d == down)
				d = left;
		}
		else if (p.second == counterclockwise) {
			if (d == up)
				d = left;
			else if (d == right)
				d = up;
			else if (d == left)
				d = down;
			else if (d == down)
				d = right;
		}

		qs.edit(p.first, d);
	}

	if (kill) {
		qs.remove(p.first);
		if (p.second == dead) {
			if (!p.first->isPlayer())
				pull(p.first, p.first->getX(), p.first->getY());
			p.first->markDead();
		}
	}

	return qs.isEmpty();
}


//Returns true if player p is trapped, or if p has been marked dead
bool Board::isDead(int p) {
	Entity* play = nullptr;
	auto it = map.begin();
	for (; it != map.end(); it++) {
		if (it->second->getID() == p) {
			play = it->second;
			break;
		}
		if (it->second->isContainer()) {
			if (((EntityList*)(it->second))->contains(p)) {
				play = ((EntityList*)(it->second))->get(p);
				break;
			}
		}
	}
	//Has been marked dead or otherwise deleted
	if (play == nullptr)
		return true;


	//Is trapped
	int x = play->getX();
	int y = play->getY();

	Entity* e = getEntity(x + 1, y);
	if (e == nullptr || !e->isSolid(play))
		return false;

	e = getEntity(x - 1, y);
	if (e == nullptr || !e->isSolid(play))
		return false;

	e = getEntity(x, y + 1);
	if (e == nullptr || !e->isSolid(play))
		return false;

	e = getEntity(x, y - 1);
	if (e == nullptr || !e->isSolid(play))
		return false;

	//Player is surrounded by solid enitites
	return true;
}

//Returns false if the position in the d direction of the active player isSolid
bool Board::isLegal(Direction d) {
	if (turn == -1)
		return false;
	Entity* play = nullptr;
	auto it = map.begin();
	for (; it != map.end(); it++) {
		if (it->second->getID() == turn) {
			play = it->second;
			break;
		}
		if (it->second->isContainer()) {
			if (((EntityList*)(it->second))->contains(turn)) {
				play = ((EntityList*)(it->second))->get(turn);
				break;
			}
		}
	}
	if (play == nullptr)
		return false;

	int x = play->getX();
	int y = play->getY();

	if (d == up)
		y -= 1;
	if (d == down)
		y += 1;
	if (d == left)
		x -= 1;
	if (d == right)
		x += 1;

	Entity* e = getEntity(x, y);
	if (e == nullptr)
		return true;
	return !(e->isSolid(play));
}


//Renders Board to Terminal
void Board::print() {
	//Axis only works if there are less than 10 non border columns
	bool axis = ROWS <= 11 && COLS <= 11;
	std::cout << "\n";
	if (axis) {
		std::cout << "   |";
		for (int i = 1; i<COLS - 1; i++) {
			std::cout << i << '|';
		}
		std::cout << "  \n";

	}

	for (int y = 0; y<ROWS; y++) {

#ifdef BARS
		if (axis)
			std::cout << " ";

		for (int x = 0; x<COLS; x++)
			std::cout << "--";

		if (axis)
			std::cout << '-';

		std::cout << "\n";
#endif

		if (axis) {
			if (y == 0 || y == ROWS - 1)
				std::cout << " |";
			else
				std::cout << y << '|';
		}

		for (int x = 0; x<COLS; x++) {
			Entity* e = getEntity(x, y);
			if (e == nullptr)
				std::cout << " |";
			else
				std::cout << e->getChar() << '|';
		}

		if (axis) {
			if (y == 0 || y == ROWS - 1)
				std::cout << ' ';
			else
				std::cout << y;
		}
		std::cout << '\n';
	}

#ifdef BARS
	if (axis)
		std::cout << " ";

	for (int x = 0; x<COLS; x++)
		std::cout << "--";

	if (axis)
		std::cout << '-';

	std::cout << "\n";
#endif

	if (axis) {
		std::cout << "   |";
		for (int i = 1; i<COLS - 1; i++) {
			std::cout << i << '|';
		}
		std::cout << "  \n";
	}
	std::cout << "\n";
}

