#include "../platform.h" // This file will make exporting DLL symbols simpler for students.
//Apoorv Khosla : Project #1 : PathPlanner <HEADER> : AI for games

//Including other user-defined header files to allow importing and access of declared functions.

#include "Tile.h"
#include "TileMap.h"
#include "PriorityQueue.h"
#include <iostream>
#include <vector>
#include <map>
#include <queue>



using std::cout;
using std::map;
using std::vector;
using std::priority_queue;
using std::deque;
namespace ufl_cap4053
{
	namespace searches
	{
		struct PlannerNode	//Creating PlannerNode i.e. parentNode in case for map.
		{
			Tile* state;
			PlannerNode* parent;
			double costHeuristic;
			double costGiven;
			//PlannerNode constructor viz PlannerNode()
			PlannerNode(Tile* _state, PlannerNode* _parent, double _costHeuristic, double _costGiven);
			
			bool operator<(PlannerNode& l);
		};
		//bool variable comparison for>>
		bool compare(PlannerNode* const& l, PlannerNode* const& h);
		class PathSearch
		{
		// CLASS DECLARATION GOES HERE
		private:

			Tile* StartTile, * GoalTile;
			PlannerNode* currentPNode;
			TileMap* tilemap;
			bool Donema;	//bool created for  isDone intentional check
			map<Tile const*, PlannerNode*> mapD;

			PriorityQueue<PlannerNode*> open;
			vector<Tile const*> choiceofPath;
			vector<Tile*> CloseNodes;
			void areAdjacent(PlannerNode* currentPNode);
			double dist(Tile const* _tile1, Tile const* _tile2);
		public:

			PathSearch();//Constructor called for initialisation purposes
			//Constructor, takes no arguments.


			//Destructor complete.
			~PathSearch();
			//Destructor performs any final cleanup required.


			/***So now by default all Header files contain function or method declarations and
				.CPP extension files are held with their implementation i.e. function body i.e.
					using SRO[::] operator and hence called within the cpp body itself too.***/

			//So here-goes ->

			//Load function declaration
			void load(TileMap* _tileMap);
			//Called after tile map is loaded.

			void initialize(int startRow, int startColumn, int goalRow, int goalColumn);
			//Called before any update of the path planner.

			void update(long timeslice);

			//Still figuring out
			void shutdown();

			//Algorithm reset
			void unload();

			bool isDone() const;
			//Returns true if the update function has finished because it found a solution, and false otherwise.

			std::vector<Tile const*> const getSolution() const;
			//Return a vector containing the solution path as an ordered series of Tile pointers from finish to start.

			//Priority DLL requirement pre-input by Professor J
			DLLEXPORT PathSearch(); // EX: DLLEXPORT required for public methods - see platform.h

		};
	}
}  // close namespace ufl_cap4053::searches
