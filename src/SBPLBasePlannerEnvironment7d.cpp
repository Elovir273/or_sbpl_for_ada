#include <or_sbpl_for_ada/SBPLBasePlannerEnvironment7d.h>
#include <boost/foreach.hpp>
#include <boost/make_shared.hpp>
#include <boost/math/constants/constants.hpp>
#include <limits>

using namespace or_sbpl_for_ada; 


namespace bmc = boost::math::constants;

SBPLBasePlannerEnvironment::SBPLBasePlannerEnvironment(OpenRAVE::RobotBasePtr robot) 
    : _robot(robot), _timestep(0.05) {

    _lweight = 10;
    _tweight = 10;   
    _mweight = 10;
}

SBPLBasePlannerEnvironment::~SBPLBasePlannerEnvironment() {

    StateIndex2StateIdTable.clear();
    StateId2CoordTable.clear();
    _actions.clear();
}

bool SBPLBasePlannerEnvironment::Initialize(const double &cellsize,
                                            const EnvironmentExtents &extents,
                                            const int &numangles,
                                            const ActionList &actions,
					    const double &lweight, // linear weight, x,y,z
					    const double &tweight, // angle weight, phi, theta, psi
                        const double &mweight, // mode switching weight
                        const int &nummodes,
                        const int &start_mode){

    // Setup environment attributes
    _cellsize = cellsize;

    _gridwidth = static_cast<int>(ceil((extents.xmax - extents.xmin)/_cellsize));
    _gridheight = static_cast<int>(ceil((extents.ymax - extents.ymin)/_cellsize));
    _griddepth = static_cast<int>(ceil((extents.zmax - extents.zmin)/_cellsize));
    _extents = extents;

    //Modes 
    _nummodes = nummodes;
    _start_mode = start_mode;

    // Angles 
    _numangles = numangles;
    _anglesize = 2.0*bmc::pi<double>()/_numangles;
   
    // Weighting
    _lweight = lweight;
    _tweight = tweight;
    _mweight = mweight;

    // Actions
    _actions = actions;

     std::cout << "anglesize : "<<_anglesize << std::endl;

     unsigned long int space_size=_gridwidth*_gridheight*_griddepth*_nummodes*_numangles*_numangles*_numangles;
     unsigned long int max_int = 2147483647;
     if ( space_size > max_int ) {
        RAVELOG_ERROR("[SBPLBasePlannerEnvironment] Error : Space size > size int. \n");
     }
     std::cout << "space size : "<< space_size << std::endl;

    return true;
}

bool SBPLBasePlannerEnvironment::InitializeEnv(const char* sEnvFile) {

    RAVELOG_ERROR("[SBPLBasePlannerEnvironment] InitializeEnv not implemented");
    throw new SBPL_Exception();
}

bool SBPLBasePlannerEnvironment::InitializeMDPCfg(MDPConfig* MDPCfg) {

    MDPCfg->goalstateid = _goal;
    MDPCfg->startstateid = _start;

    return true;
}

int SBPLBasePlannerEnvironment::SetStart(const double &x, const double &y, const double &z, const double &phi, const double &theta, const double &psi) {

    WorldCoordinate wc(x, y, z, phi, theta, psi, _start_mode);
    GridCoordinate gc = WorldCoordinateToGridCoordinate(wc);

    RAVELOG_INFO("[SBPLBasePlannerEnvironment] Trying to set start to grid coordinate: %s\n", gc.toString().c_str());
    int idx = GridCoordinateToStateIndex(gc);
    std::cout << "setstart, idx : "<<idx<<std::endl;

    if( idx == INVALID_INDEX ) {
        RAVELOG_ERROR("[SBPLBasePlannerEnvironment] The start state %s is invalid.\n", gc.toString().c_str() );
        throw new SBPL_Exception();
    }

    std::map<int, int>::iterator it = StateIndex2StateIdTable.find(idx);
    int state_id;
    if(it == StateIndex2StateIdTable.end()){
        state_id = CreateState(gc);
    }else{
        state_id = it->second;
    }

    _start = state_id;

    RAVELOG_INFO("[SBPLBasePlannerEnvironment] Set start to id: %d\n", _start);

    return state_id;
}

int SBPLBasePlannerEnvironment::SetGoal(const double &x, const double &y, const double &z, const double &phi, const double &theta, const double &psi, const int &mode) {

    //Temporary : single goal for now. multiples later
    int temp_mode=mode;
    if (temp_mode == -1) {
        temp_mode=1;
    }

    WorldCoordinate wc(x, y, z, phi, theta, psi, temp_mode);
    GridCoordinate gc = WorldCoordinateToGridCoordinate(wc);

    int idx = GridCoordinateToStateIndex(gc);
    RAVELOG_INFO("[SBPLBasePlannerEnvironment] Trying to set goal to grid coordinate %d: %s (%s)\n", idx, wc.toString().c_str(), gc.toString().c_str());    
    if( idx == INVALID_INDEX ) {
        RAVELOG_ERROR("[SBPLBasePlannerEnvironment] The goal state %s is invalid.\n", gc.toString().c_str() );
        throw new SBPL_Exception();
    }

    std::map<int, int>::iterator it = StateIndex2StateIdTable.find(idx);
    int state_id;
    if(it == StateIndex2StateIdTable.end()){
        state_id = CreateState(gc);
    }else{
        state_id = it->second;
    }

    _goal = state_id;

    RAVELOG_INFO("[SBPLBasePlannerEnvironment] Set goal to id: %d\n", _goal);

    return state_id;
}

// Return the cost of the best path of this mode
void SBPLBasePlannerEnvironment::ConvertStateIDPathIntoWaypointPath(const std::vector<int> &state_ids,
                                                                    std::vector<PlannedWaypointPtr> &path, double &path_cost,
                                                                    std::vector<WorldCoordinate> &cart_path,
                                                                    std::vector<WorldCoordinate> &action_list ) {
      

    RAVELOG_INFO("[SBPLBasePlannerEnvironment] Begin ConvertStateIDPathIntoXYThetaPath\n");

    double path_cost_temp = 0;
    // clear out the vector just in case
    path.clear();

    // iterate through the path
    for(unsigned int pidx = 0; pidx < state_ids.size()-1; pidx++){
        
        // Grab the states that start and end this action
        int start_id = state_ids[pidx];
        int goal_id = state_ids[pidx+1];
       // std::cout <<"start id & goal id : "<< start_id << " "<<goal_id<<std::endl;

        // Now search through all successors to find the best (least cost) one
        //   that leads to the goal
        std::vector<int> succ_ids;
        std::vector<int> costs;
        std::vector<ActionPtr> actions;
        
        GetSuccs(start_id, &succ_ids, &costs, &actions);
        
        int best_idx = -1;
        double best_cost = std::numeric_limits<double>::infinity();
        for(unsigned int idx=0; idx < succ_ids.size(); idx++){

            int succ_id = succ_ids[idx];
            if(succ_id == goal_id && costs[idx] <= best_cost){
                best_cost = costs[idx];
                best_idx = idx;
            }
        }

        path_cost_temp+=best_cost;
        // If we didn't find a successor something has gone terribly wrong, bail
        if(best_idx == -1){
            RAVELOG_ERROR("[SBPLBasePlannerEnvironment] Failed to reconstruct path.");
            throw new SBPL_Exception();
        }

        // Play the action forward, setting all the intermediate states
        ActionPtr a = actions[best_idx];


        GridCoordinate gc = StateId2CoordTable[start_id];
    
        WorldCoordinate wc_current = GridCoordinateToWorldCoordinate(gc);
        std::cout<<"    Traj (world) : "
        <<wc_current.x<<" "<<wc_current.y<<" "<<wc_current.z<<" "<<wc_current.phi<<" "<<wc_current.theta<<" "<<wc_current.psi<<" "<<wc_current.mode<<std::endl;
     
        // Add initial state
          if ( pidx == 0 )  {
            cart_path.push_back(wc_current);
          }

        std::vector<WorldCoordinate> pts = a->applyWithIntermediates(wc_current, _robot);

        // pts[0] : because array with one value
      WorldCoordinate effect_action( pts[0].x - wc_current.x, pts[0].y - wc_current.y, pts[0].z - wc_current.z, 
                    pts[0].phi - wc_current.phi, pts[0].theta - wc_current.theta, pts[0].psi - wc_current.psi, pts[0].mode );
        action_list.push_back(effect_action);
        cart_path.push_back(pts[0]);

        BOOST_FOREACH(WorldCoordinate wc_next, pts){
            // Add this pose to the pose list
            PlannedWaypointPtr pt = boost::make_shared<PlannedWaypoint>(wc_next, a);
            path.push_back(pt);
        }
    }
    path_cost=path_cost_temp;

    RAVELOG_INFO("[SBPLBasePlannerEnvironment] Generated path of length %d\n", path.size());
    return;
}

int SBPLBasePlannerEnvironment::GetFromToHeuristic(int FromStateID, int ToStateID){
    if(FromStateID >= (int) StateId2CoordTable.size()){
        RAVELOG_ERROR("[SBPLBasePlannerEnvironment] ERROR in GetFromToHeuristic: FromStateID %d is illegal\n", FromStateID);
        throw new SBPL_Exception();
    }
    
    if(ToStateID >= (int) StateId2CoordTable.size()){
        RAVELOG_ERROR("[SBPLBasePlannerEnvironment] ERROR in GetFromToHeuristic: ToStateID %d is illegal\n", ToStateID);
        throw new SBPL_Exception();
    }

    // Grab the correct grid coordinate
    GridCoordinate gTo = StateId2CoordTable[ToStateID];
    GridCoordinate gFrom = StateId2CoordTable[FromStateID];

    // Calculate the euclidean distance
    double cost = ComputeCost(gFrom, gTo);
    return (int)(cost);

}

int SBPLBasePlannerEnvironment::GetGoalHeuristic(int stateID){
    return GetFromToHeuristic(stateID, _goal);
}

int SBPLBasePlannerEnvironment::GetStartHeuristic(int stateID){
    return GetFromToHeuristic(stateID, _start);

}
 
void SBPLBasePlannerEnvironment::GetSuccs(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV){

    std::vector<ActionPtr> ignored;
    GetSuccs(SourceStateID, SuccIDV, CostV, &ignored);
}

void SBPLBasePlannerEnvironment::GetSuccs(int SourceStateID, std::vector<int>* SuccIDV, std::vector<int>* CostV, std::vector<ActionPtr>* ActionV){

    SuccIDV->clear();
    CostV->clear();

    // Check validity of the state
    if( !IsValidStateId(SourceStateID) ){
        RAVELOG_ERROR("[SBPLBasePlanningEnvironment] Id %d is invalid state\n", SourceStateID);
        return;
    }

    // If this is the goal state, just return
    if( SourceStateID == _goal ){
        RAVELOG_INFO("[SBPLBasePlanningEnvironment] Expanded goal. Returning.\n");
        return;
    }

    // Convert to a world coordinate
    GridCoordinate gc = StateId2CoordTable[SourceStateID];
    WorldCoordinate wc = GridCoordinateToWorldCoordinate(gc);
    int orig_idx = GridCoordinateToStateIndex(gc);

    RAVELOG_DEBUG("[SBPLBasePlanningEnvironment] Expanding node %d: %s (%s)\n",
                  SourceStateID, wc.toString().c_str(), gc.toString().c_str());

    // Lock the environment
    OpenRAVE::EnvironmentBasePtr env = _robot->GetEnv();
    OpenRAVE::EnvironmentMutex::scoped_lock lock(env->GetMutex());

    // Now step through each of the actions
    //std::vector<ActionPtr> actions = _actions[gc.theta];
    std::vector<ActionPtr> actions = _actions[gc.mode];

    BOOST_FOREACH(ActionPtr a, actions) {

        // Apply each action, checking for collision along the way
        WorldCoordinate wc_final;

        bool valid = a->apply(wc, _robot, wc_final);
      //  std::cout << "Ltest2 : " << valid <<std::endl;
        if( valid ) {
            GridCoordinate gc_final = WorldCoordinateToGridCoordinate(wc_final);
            int state_idx = GridCoordinateToStateIndex(gc_final);
         //   std::cout << "Ltest3 : "<< state_idx << std::endl;
	    if(state_idx == orig_idx){
		RAVELOG_WARN("[SBPLBasePlanningEnvironment] Action did not move from original grid cell. Check primitives for validity.\n");
		continue;
	    }

            if(state_idx != INVALID_INDEX){
                // Action propagatin led to a valid state
                

                std::map<int, int>::iterator it = StateIndex2StateIdTable.find(state_idx);
                int state_id;
                if( it == StateIndex2StateIdTable.end() ){
                    state_id = CreateState(gc_final);
                }else{
                    state_id = it->second;
                }

                RAVELOG_DEBUG("[SBPLBasePlannerEnvironment] Adding successor state %d (idx %d): %s (%s)\n", 
                              state_id, state_idx, wc_final.toString().c_str(), 
                              gc_final.toString().c_str());

                SuccIDV->push_back(state_id);

                double cost = ComputeCost(gc, gc_final);
		int icost = cost * a->getWeight();
		if(icost <= 0){
		    RAVELOG_WARN("[SBPLBasePlannerEnvironment] Invalid cost: %d (Cost: %0.3f, Weight: %0.3f)\n", icost, cost, a->getWeight());
		}
                CostV->push_back(icost); 

                ActionV->push_back(a);
            }
        }
    }
}

void SBPLBasePlannerEnvironment::GetPreds(int TargetStateID, std::vector<int>* PredIDV, std::vector<int>* CostV){
    RAVELOG_ERROR("[SBPLBasePlanningEnvironment] GetPreds is not implemented.\n");
    throw new SBPL_Exception();
}

void SBPLBasePlannerEnvironment::SetAllActionsandAllOutcomes(CMDPSTATE* state){

    RAVELOG_ERROR("[SBPLBasePlanningEnvironment] SetAllActionsandAllOutcomes is not implemented.\n");
    throw new SBPL_Exception();
}
 
void SBPLBasePlannerEnvironment::SetAllPreds(CMDPSTATE* state){

    RAVELOG_ERROR("[SBPLBasePlanningEnvironment] SetAllPreds is not implemented.\n");
    throw new SBPL_Exception();
}

int SBPLBasePlannerEnvironment::SizeofCreatedEnv(){
    return (int) StateId2CoordTable.size();
}

void SBPLBasePlannerEnvironment::PrintState(int stateID, bool bVerbose, FILE* fOut){

    if( stateID > StateId2CoordTable.size() ){
        RAVELOG_ERROR("[SBPLBasePlanningEnvironment] Invalid state id: %d.\n", stateID);
        throw new SBPL_Exception();
    }

    GridCoordinate gc = StateId2CoordTable[stateID];

    if(fOut == NULL){
        fOut = stdout;
    }

    SBPL_FPRINTF(fOut, "Grid: X=%d, Y=%d, Z=%d, Phi=%d, Theta=%d, Psi=%d, Mode=%d", gc.x, gc.y, gc.z, gc.phi, gc.theta, gc.psi, gc.mode);
    if(bVerbose){
        WorldCoordinate wc = GridCoordinateToWorldCoordinate(gc);
        SBPL_FPRINTF(fOut, " World: X=%0.3f, Y=%0.3f, Z=%0.3f, Phi=%0.3f, Theta=%0.3f, Psi=%0.3f\n, Mode=%d", wc.x, wc.y,wc.z, wc.phi, wc.theta, wc.psi, gc.mode);
    }else{
        SBPL_FPRINTF(fOut, "\n");
    }
}

void SBPLBasePlannerEnvironment::PrintEnv_Config(FILE* fOut){

    RAVELOG_ERROR("[SBPLBasePlanningEnvironment] PrintEnv_Config is not implemented.\n");
    throw new SBPL_Exception();
}

WorldCoordinate SBPLBasePlannerEnvironment::GridCoordinateToWorldCoordinate(const GridCoordinate &gcoord) const {

    WorldCoordinate retCoord;
    
    retCoord.x = gcoord.x*_cellsize + (_cellsize/2.0) + _extents.xmin;
    retCoord.y = gcoord.y*_cellsize + (_cellsize/2.0) + _extents.ymin;
    retCoord.z = gcoord.z*_cellsize + (_cellsize/2.0) + _extents.zmin;
    retCoord.phi = gcoord.phi*_anglesize;
    retCoord.theta = gcoord.theta*_anglesize;
    retCoord.psi = gcoord.psi*_anglesize;
    retCoord.mode=gcoord.mode;

    return retCoord;
}
 
GridCoordinate SBPLBasePlannerEnvironment::WorldCoordinateToGridCoordinate(const WorldCoordinate &wcoord) const {

    GridCoordinate retCoord;
    double x = wcoord.x - _extents.xmin;
    if( x < 0 ){ x = 0.0; }
    retCoord.x = (int)(x/_cellsize);
    
    double y = wcoord.y - _extents.ymin;
    if( y < 0 ){ y = 0.0; }
    retCoord.y = (int)(y/_cellsize);

    double z = wcoord.z - _extents.zmin;
    if( z < 0 ){ z = 0.0; }
    retCoord.z = (int)(z/_cellsize);
    
    double phi = wcoord.phi;
   // std::cout << "test : " << phi << " "<<bmc::pi<double>()<<std::endl;

    while(phi < 0){
        phi += 2.0*bmc::pi<double>();
    }

    while(phi >= 2.0*bmc::pi<double>()){
        phi -= 2.0*bmc::pi<double>();
    }

    retCoord.phi = (int)(phi/_anglesize + 0.5);
    // Catch the angle wrap-around
    if(retCoord.phi == _numangles)
        retCoord.phi = 0;

    double theta = wcoord.theta;
    while(theta < 0){
        theta += 2.0*bmc::pi<double>();
    }

    while(theta >= 2.0*bmc::pi<double>()){
        theta -= 2.0*bmc::pi<double>();
    }

    retCoord.theta = (int)(theta/_anglesize + 0.5);
	// Catch the angle wrap-around
	if(retCoord.theta == _numangles)
		retCoord.theta = 0;

    double psi = wcoord.psi;
    while(psi < 0){
        psi += 2.0*bmc::pi<double>();
    }

    while(psi >= 2.0*bmc::pi<double>()){
        psi -= 2.0*bmc::pi<double>();
    }

    retCoord.psi = (int)(psi/_anglesize + 0.5);
    // Catch the angle wrap-around
    if(retCoord.psi == _numangles)
        retCoord.psi = 0;

    retCoord.mode=wcoord.mode;

    return retCoord;
}

GridCoordinate SBPLBasePlannerEnvironment::StateIndexToGridCoordinate(unsigned int stateidx) const{

    if( stateidx >= StateIndex2StateIdTable.size() ){
        RAVELOG_ERROR("[SBPLBasePlannerEnvironment] The state index %d is invalid.\n", stateidx);
        throw new SBPL_Exception();
    }
    
    std::map<int, int>::const_iterator it = StateIndex2StateIdTable.find(stateidx);
    if( it == StateIndex2StateIdTable.end() ){
        RAVELOG_ERROR("[SBPLBasePlannerEnvironment] The state index %d maps to an uninitialized state id.\n", stateidx);
        throw new SBPL_Exception();
    }

    return StateId2CoordTable[it->second];
}

int SBPLBasePlannerEnvironment::GridCoordinateToStateIndex(const GridCoordinate &coord) const{

    int retIdx = INVALID_INDEX;


    //check validity
    if(coord.x < 0 || coord.x >= _gridwidth || 
       coord.y < 0 || coord.y >= _gridheight ||
       coord.z < 0 || coord.z >= _griddepth ||
       coord.phi < 0 || coord.phi >= _numangles ||
       coord.theta < 0 || coord.theta >= _numangles ||
       coord.psi < 0 || coord.psi >= _numangles ||
       coord.mode < 1 || coord.mode > 5 ){
        return retIdx;
    }
     
    // retIdx = coord.theta + coord.y * _numangles + coord.x * _numangles * _gridheight;
    retIdx = coord.mode +
            coord.phi * _nummodes +
            coord.theta * _nummodes * _numangles +
            coord.psi * _nummodes * _numangles * _numangles +
            coord.x * _nummodes * _numangles * _numangles * _numangles +
            coord.y * _nummodes * _numangles * _numangles * _numangles * _gridwidth +
            coord.z * _nummodes * _numangles * _numangles * _numangles * _gridwidth * _gridheight;

    return retIdx;
}

int SBPLBasePlannerEnvironment::CreateState(const GridCoordinate &gc) {

    // get the index
    int state_idx = GridCoordinateToStateIndex(gc);

    if(state_idx == INVALID_INDEX){
        return UNINITIALIZED_ID;
    }

    // generate a new id and add the node
    int state_id = StateId2CoordTable.size();
    StateId2CoordTable.push_back(gc);
    StateIndex2StateIdTable[state_idx] = state_id;

    int *entry = new int[1];
    StateID2IndexMapping.push_back(entry);
    StateID2IndexMapping[state_id][0] = -1; // TODO: What is this? copied from SBPL code.

    return state_id;
}

bool SBPLBasePlannerEnvironment::IsValidStateId(const int &state_id) const {
    
    if( state_id < 0 || state_id >= StateId2CoordTable.size() ){
        return false;
    }else{
        return true;
    }
}


double SBPLBasePlannerEnvironment::ComputeCost(const GridCoordinate &c1, const GridCoordinate &c2) const 
{
    int xdiff = c1.x - c2.x;
    int ydiff = c1.y - c2.y;
    int zdiff = c1.z - c2.z;

    int phidiff1 = c2.phi - c1.phi;
    if(phidiff1 < 0) phidiff1 += _numangles;
    int phidiff2 = c1.phi - c2.phi;
    if(phidiff2 < 0) phidiff2 += _numangles;
    int phidiff = std::min(phidiff1, phidiff2);

    int thetadiff1 = c2.theta - c1.theta;
    if(thetadiff1 < 0) thetadiff1 += _numangles;
    int thetadiff2 = c1.theta - c2.theta;
    if(thetadiff2 < 0) thetadiff2 += _numangles;
    int thetadiff = std::min(thetadiff1, thetadiff2);

    int psidiff1 = c2.psi - c1.psi;
    if(psidiff1 < 0) psidiff1 += _numangles;
    int psidiff2 = c1.psi - c2.psi;
    if(psidiff2 < 0) psidiff2 += _numangles;
    int psidiff = std::min(psidiff1, psidiff2);

    int mode_cost=0;
    if ( c1.mode != c2.mode ) {
        mode_cost=1;
    }

    double cost = _lweight*xdiff*xdiff + _lweight*ydiff*ydiff + _lweight*zdiff*zdiff + 
                    _tweight*thetadiff*thetadiff + _tweight*psidiff*psidiff + _tweight*phidiff*phidiff +
                    _mweight*mode_cost ;
    return cost;
}
