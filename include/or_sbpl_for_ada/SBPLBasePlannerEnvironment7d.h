#ifndef SBPL_BASE_PLANNER_ENVIRONMENT_H_
#define SBPL_BASE_PLANNER_ENVIRONMENT_H_

#include <or_sbpl_for_ada/SBPLBasePlannerTypes7d.h>
#include <sbpl/discrete_space_information/environment.h>
#include <sbpl/utils/mdp.h>
#include <sbpl/utils/mdpconfig.h>
#include <sbpl/utils/utils.h>

#include <openrave/openrave.h>

namespace or_sbpl_for_ada {

    /**
     * An implementation of an x,y,theta environment for use by SBPL planners
     */
    class SBPLBasePlannerEnvironment : public DiscreteSpaceInformation {

    public:
	/**
	 * Constructor
	 *
	 * @param robot The OpenRAVE robot to collision check during planning
	 */
        SBPLBasePlannerEnvironment(OpenRAVE::RobotBasePtr robot);

	/**
	 * Destructor
	 */
        ~SBPLBasePlannerEnvironment();

	/**
	 * Initialize the environment from the given parameters
	 *
	 * @param cellsize The size of the grid cellsize
	 * @param extents The extends of the environment
	 * @param numangles The number of angles to divide the space
	 * @param actions The list of valid actions
	 * @param lweight The weight to apply to linear translation
	 * @param tweight The weight to apply to change in orientation
	 * @param mweight The weight to apply to change the mode
	 */
        virtual bool Initialize(const double &cellsize,
                                const EnvironmentExtents &extents,
                                const int &numangles,
                                const ActionList &actions,
				const double &lweight,
				const double &tweight,
				const double &mweight,
				const int &nummodes);

	/**
	 * Not implemented - we want to initialize from the OpenRAVE planner parameters
	 */
        virtual bool InitializeEnv(const char* sEnvFile);
	
	/**
	 * Sets the start and the goal states in the MDPCfg 
	 */
        virtual bool InitializeMDPCfg(MDPConfig* MDPCfg);

	/**
	 * Returns the heuristic cost between the two states
	 * In this case, distance in millimeters.
	 *
	 * @param FromStateID The state to start at
	 * @param ToStateID The state to go to
	 * @return Euclidean distance between FromStateID and ToStateID (in mm)
	 */
        virtual int GetFromToHeuristic(int FromStateID, int ToStateID);

	/**
	 * Returns the heuristic cost between the two states
	 * In this case, distance in millimeters.
	 *
	 * @param stateID The state to start at
	 * @return Euclidean distance between the state and the goal
	 */
        virtual int GetGoalHeuristic(int stateID);

	/**
	 * Returns the distance in millimeters between
	 *  the given state and the start.
	 *
	 * @param stateID The state to start at
	 * @return Euclidean distance between the state and the start
	 */
        virtual int GetStartHeuristic(int stateID);

	/**
	 * Returns a list of valid successor states.  Actions leading to these successors
	 * have been checked for collision against the openrave environment.
	 *
	 * @param SourceStateID The state to get successors for
	 * @param SuccIDV The list of valid successors
	 * @param CostV The cost to move to each predecessor
	 */
        virtual void GetSuccs(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV);

	/**
	 * Returns a list of valid successor states.  Actions leading to these successors
	 * have been checked for collision against the openrave environment.
	 *
	 * @param SourceStateID The state to get successors for
	 * @param SuccIDV The list of valid successors
	 * @param CostV The cost to move to each predecessor
	 * @param ActionV The action that moves to the predecessor
	 */
        virtual void GetSuccs(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV, std::vector<ActionPtr>* ActionV);

	/**
	 * Not implemented
	 */
        virtual void GetPreds(int TargetStateID, std::vector<int>* PredIDV, std::vector<int>* CostV);

	/**
	 * Not implemented
	 */
        virtual void SetAllActionsandAllOutcomes(CMDPSTATE* state);

	/**
	 * Not implemented
	 */
        virtual void SetAllPreds(CMDPSTATE* state);

	/**
	 * @return The number of states currently in the environment
	 */
        virtual int SizeofCreatedEnv();

	/**
	 * Prints the state represented by the given id
	 *
	 * @param stateID The state to print
	 * @param bVerbose If true, prints both world and grid coordinate, otherwise only gridcoord is printed
	 * @param fOut The output location
	 */
        virtual void PrintState(int stateID, bool bVerbose, FILE* fOut = NULL);

	/**
	 * Not implemented
	 */
        virtual void PrintEnv_Config(FILE* fOut);

	/**
	 * Sets the start state.
	 *
	 * @param x The x position of the start
	 * @param y  The y position of the start
	 * @param theta The yaw of the start
	 * @return The associated state id
	 */
        virtual int SetStart(const double &x, const double &y, const double &z, const double &phi, const double &theta, const double &psi, const int &mode);
        
	/**
	 * Sets the goal state.
	 *
	 * @param x The x position of the goal
	 * @param y  The y position of the goal
	 * @param theta The yaw of the goal
	 * @return The associated state id
	 */
        virtual int SetGoal(const double &x, const double &y, const double &z, const double &phi, const double &theta, const double &psi, const int &mode);

	/**
	 * Returns x,y,theta-coords for each point along the path.
	 * 
	 *
	 * @param state_ids The list of state ids that make up the path
	 * @param path The converted path
	 */
        virtual double ConvertStateIDPathIntoWaypointPath(const std::vector<int> &state_ids,
							std::vector<PlannedWaypointPtr> &path);
        
	/**
	 * Converts from a grid coordinate to a world coordinate
	 *
	 * @param gcoord The grid coordinate to convert
	 * @return The world coordinate of the center of the associated grid cell
	 */
        WorldCoordinate GridCoordinateToWorldCoordinate(const GridCoordinate &gcoord) const;

	/**
	 * Converts from a world coordinate to a grid coordinate
	 *
	 * @param wcoord The world coordinate to convert
	 * @return The coordinate of the grid cell containing the world coordinate
	 */
        GridCoordinate WorldCoordinateToGridCoordinate(const WorldCoordinate &wcoord) const;

	/**
	 * Converts the state id to a grid coordinate
	 *
	 * @param stateidx The state id to convert
	 * @return The associated grid coordinate
	 */
        GridCoordinate StateIndexToGridCoordinate(unsigned int stateidx) const;

	/**
	 * Converts the grid coordinate to a state id
	 *
	 * @param gcoord The grid coordinate to convert
	 * @return The state id associated with the grid coordinate
	 */
        int GridCoordinateToStateIndex(const GridCoordinate &gcoord) const;

	/**
	 * Creates a state from the given coordinate
	 *
	 * @param gc The coordinate
	 * @return The id of the created state
	 */
        int CreateState(const GridCoordinate &gc);

	/**
	 * Checks a state id for validity
	 *
	 * @param state_id The id to check
	 * @return True if the id is valid
	 */
        bool IsValidStateId(const int &state_id) const;

	/**
	 * Flag for indicating an invalid state index
	 */
        static const int INVALID_INDEX = -1;

	/**
	 * Flag for indicating an uninitialized state id
	 */
        static const int UNINITIALIZED_ID = -1;

    protected:
	/**
	 * Map state ids to the associated grid coordinate
	 */
        std::vector<GridCoordinate> StateId2CoordTable;

	/**
	 * Map state index to the associated state id
	 */
        std::map<int, int> StateIndex2StateIdTable;

    private:
	/**
	 * Computes the cost of moving from one grid coordinate to another
	 * 
	 * @param c1 The start grid coordinate
	 * @param c2 The end grid coordinate
	 * @return The cost
	 */
        double ComputeCost(const GridCoordinate &c1, const GridCoordinate &c2) const;

        OpenRAVE::RobotBasePtr _robot;         
        double _cellsize;
        double _anglesize;

        int _goal;
        int _start;

        int _gridheight;
        int _gridwidth;
        int _griddepth;
        int _numangles;
        int _nummodes;

        ActionList _actions;
        double _timestep; // seconds per step

	double _tweight;
	double _lweight;
	double _mweight;

        EnvironmentExtents _extents;
    };

    typedef boost::shared_ptr<SBPLBasePlannerEnvironment> SBPLBasePlannerEnvironmentPtr;

}

#endif
