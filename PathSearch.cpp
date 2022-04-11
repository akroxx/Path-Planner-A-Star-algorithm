#include "PathSearch.h"
namespace ufl_cap4053
{
	namespace searches
	{	//Apoorv Khosla : Project #1 : PathPlanner <CPP> : AI for games
		// CLASS DEFINITION GOES HERE
		
		//Def Constructor <PlannerNode> initializer in laymen terms
		PlannerNode::PlannerNode(Tile* _state, PlannerNode* _parent, double _costHeuristic, double _costGiven)
		{
			state = _state;
			parent = _parent;
			costHeuristic = _costHeuristic;
			costGiven = _costGiven;
		}

		//Def constructor <PathSearch>
		PathSearch::PathSearch() : open(ufl_cap4053::algorithms::compare)
		{
			tilemap = NULL;
			StartTile = NULL;
			GoalTile = NULL;

		}

		//Destructor function body =>Empty right now ==>Called on every new tile move

		PathSearch::~PathSearch()
		{
		}
		//Load function body/definition
		void PathSearch::load(TileMap* _tileMap)
		{
			tilemap = _tileMap;
			Donema = false;
		}

		void PathSearch::initialize(int startRow, int startColumn, int goalRow, int goalColumn)
		{
			if (startRow >= 0 && startRow < tilemap->getRowCount()) //startRow check 
				StartTile = tilemap->getTile(startRow, startColumn);
			if (goalRow >= 0 && goalColumn < tilemap->getColumnCount())	//goalRow check
				GoalTile = tilemap->getTile(goalRow, goalColumn);
			
			open.push(new PlannerNode(StartTile, NULL, dist(StartTile, GoalTile) * 1.1, 0));
			mapD[StartTile] = open.front();
			Donema = false;	//byDef false initialized until assigned i.e. change visible
			currentPNode = NULL;	//created
		}


		
		void PathSearch::update(long timeslice)
		{
			PlannerNode* planNode = NULL;	//created
			double GCost;		//created
			
			while (!open.empty())	//while remains not empty-open
			{
				currentPNode = open.front(); //assigning
				open.pop();	//pop function
				if (currentPNode->state == GoalTile)
				{
					cout << "Give Cost : " << currentPNode->costGiven << "\n\n";
					Donema = true;
					while (currentPNode != NULL)
					{
						choiceofPath.push_back(currentPNode->state);
						currentPNode = currentPNode->parent;
					}
					cout << "Soultion Size : " << choiceofPath.size() << "\n\n";
					break;
				}
				areAdjacent(currentPNode);
				for (size_t i = 0; i < CloseNodes.size(); i++)
				{
					GCost = currentPNode->costGiven + (dist(CloseNodes[i], currentPNode->state) * CloseNodes[i]->getWeight());
					planNode = mapD[CloseNodes[i]];
					if (planNode)
					{
						if (planNode->costGiven > GCost)
						{
							open.remove(planNode);
							planNode->costGiven = GCost;
							
							planNode->costHeuristic = dist(planNode->state, GoalTile) * 1.1;
							planNode->parent = currentPNode;
							open.push(planNode);
							CloseNodes[i]->setFill(0xFFFF0000);
						}
					}
					else
					{
						PlannerNode* succNode = new PlannerNode(CloseNodes[i], currentPNode, dist(CloseNodes[i], GoalTile) * 1.1, GCost);
						mapD[succNode->state] = succNode;
						open.push(succNode);
						CloseNodes[i]->setFill(0xFF0000FF);
					}
				}
				
				//Now satisfactory condition for timeslice==0 i.e. only ONE iteration for the code_snippet
				if (timeslice == 0)
					break;
			}
		}

		void PathSearch::unload()
		{
			
			//Called by shutdown() ==> For deleting "this search" memory allocation
			//NOT AS SAME AS DESTRUCTOR
			map<Tile const*, PlannerNode*>::iterator i = mapD.begin();
			for (; i != mapD.end(); i++)
			{
				delete i->second;
			}
			open.clear();
			mapD.clear();
			choiceofPath.clear();
		}


		void PathSearch::shutdown()
		{
			unload();		//Calling unload to clear "this search" memory allocation.
			
		}

		bool PathSearch::isDone() const
		{
			return Donema;
		}

		std::vector<Tile const*> const PathSearch::getSolution() const
		{
			return choiceofPath;	//return referenced.
		}


		
		void PathSearch::areAdjacent(PlannerNode* current_PNode)
		{
			CloseNodes.clear();
			Tile* CurNode = current_PNode->state;
			
			//Gaps provided b/w conditions to avoid confusion repetition..
			Tile* current_tile = tilemap->getTile(CurNode->getRow() - 1, CurNode->getColumn());
			//1
			if (current_tile != NULL && current_tile->getWeight() != 0)
				CloseNodes.push_back(current_tile);

			
			current_tile = tilemap->getTile(CurNode->getRow() + 1, CurNode->getColumn());
			///2
			if (current_tile != NULL && current_tile->getWeight() != 0)
				CloseNodes.push_back(current_tile);
			//3
			current_tile = tilemap->getTile(CurNode->getRow(), CurNode->getColumn() - 1);
			if (current_tile != NULL && current_tile->getWeight() != 0)
				CloseNodes.push_back(current_tile);
			//4 5
			current_tile = tilemap->getTile(CurNode->getRow(), CurNode->getColumn() + 1);
			if (current_tile != NULL && current_tile->getWeight() != 0)
				CloseNodes.push_back(current_tile);
			if (!(CurNode->getRow() % 2) == 0)	//For odd i.e not divisible by 2. #BASICS
			{
				
				current_tile = tilemap->getTile(CurNode->getRow() - 1, CurNode->getColumn() + 1);
				if (current_tile != NULL && current_tile->getWeight() != 0)
					CloseNodes.push_back(current_tile);
				current_tile = tilemap->getTile(CurNode->getRow() + 1, CurNode->getColumn() + 1);
				if (current_tile != NULL && current_tile->getWeight() != 0)
					CloseNodes.push_back(current_tile);
				
			}
			else	//Automatic else switch for even divisible
			{
				
				current_tile = tilemap->getTile(CurNode->getRow() - 1, CurNode->getColumn() - 1);
				if (current_tile != NULL && current_tile->getWeight() != 0)
					CloseNodes.push_back(current_tile);
				current_tile = tilemap->getTile(CurNode->getRow() + 1, CurNode->getColumn() - 1);
				if (current_tile != NULL && current_tile->getWeight() != 0)
					CloseNodes.push_back(current_tile);
				
			}
		}

		double PathSearch::dist(Tile const* _tile1, Tile const* _tile2)	//final function 
		{
			double retain = ((_tile2->getXCoordinate() - _tile1->getXCoordinate()) * (_tile2->getXCoordinate() - _tile1->getXCoordinate())) +
				((_tile2->getYCoordinate() - _tile1->getYCoordinate()) * (_tile2->getYCoordinate() - _tile1->getYCoordinate()));
			retain = sqrt(retain);
			return retain;
		}

		bool compare(PlannerNode* const& l, PlannerNode* const& h)
		{
			return ((l->costGiven + l->costHeuristic) > (h->costGiven + h->costHeuristic)); //See notebook for pointed annotational explanation.
		}



	}
}  // close namespace ufl_cap4053::searches
