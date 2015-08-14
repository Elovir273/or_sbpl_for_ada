#include <or_sbpl_for_ada/SBPLBasePlanner7d.h>
#include <or_sbpl_for_ada/SBPLBasePlannerTypes7d.h>
#include <or_sbpl_for_ada/Action7d.h> 
#include <or_sbpl_for_ada/YamlUtils2.h>

#include <boost/unordered_map.hpp>
#include <boost/foreach.hpp>
#include <boost/make_shared.hpp>

#include <sbpl/planners/adplanner.h>
#include <sbpl/planners/planner.h>
#include <sbpl/utils/utils.h>

#include <yaml-cpp/yaml.h>
#include <fstream>

using namespace or_sbpl_for_ada;

SBPLBasePlanner::SBPLBasePlanner(OpenRAVE::EnvironmentBasePtr penv) :
// Here you modify the epsilon values you want. 
OpenRAVE::PlannerBase(penv), _orenv(penv), _initialized(false), _maxtime(5), _path_cost(-1.0),
_epsinit(3), _epsdec(1.0), _return_first(false), _n_modes(0) {

    // allow to get results back in sbpl_ams, because you can't add public function
    RegisterCommand("GetPathsCosts", boost::bind(&SBPLBasePlanner::GetPathsCosts, this, _1, _2),
        "Get the costs of the plans");
    RegisterCommand("GetCartPath", boost::bind(&SBPLBasePlanner::GetCartPath, this, _1, _2),
        "Get cartesian path");
    RegisterCommand("GetListActions", boost::bind(&SBPLBasePlanner::GetListActions, this, _1, _2),
        "Get the list of actions");
}

SBPLBasePlanner::~SBPLBasePlanner() {
}

bool SBPLBasePlanner::InitPlan(OpenRAVE::RobotBasePtr robot, PlannerParametersConstPtr params) {

    _robot = robot;
    _params = params;
    _env = boost::make_shared<SBPLBasePlannerEnvironment>(robot);

    // Parse the extra parameters
    std::stringstream extra_stream;
    extra_stream << params->_sExtraParameters;

    double _maxtime = 1.0;
    double linear_weight;
    double angle_weight;
    double mode_weight;
    double cellsize = 0.0;

    int numangles = 0;
    int nummodes = 3;  
    int n_axes = 0;

    EnvironmentExtents extents;    
    ActionList actions;


    YAML::Node doc_param = YAML::Load(extra_stream);
    n_axes = doc_param["n_axes"].as<int>();
    _n_axes = n_axes;


    if (_n_axes != 2 && _n_axes != 3 ) {
        RAVELOG_ERROR("[SBPLBasePlanner] n_axes != 3 & != 2 : n_axes =  %d", n_axes);
    }
    if ( n_axes == 2 ) { _n_modes = 3; }
    if ( n_axes == 3 ) { _n_modes = 2; }

        //Use new yaml, >= 0.5
    
    std::string s = ros::package::getPath("or_sbpl_for_ada");
    std::string s2, s3, s4, s6;

    if ( _n_axes == 2 ) {
        s2 = "/yaml/actions2DOFs_s2.yaml";
        s3 = "/yaml/actions2DOFs_s3.yaml";
        s4 = "/yaml/actions2DOFs_s4.yaml";
        s6 = "/yaml/actions2DOFs_s6.yaml";
    }
    else {
        s2= "/yaml/actions3DOFs_s2.yaml";
        s4= "/yaml/actions3DOFs_s4.yaml";
        s6= "/yaml/actions3DOFs_s6.yaml";
    }

    std::cout <<"loading"<<std::endl;
    std::vector<OpenRAVE::dReal> start_pos(6);
    OpenRAVE::RobotBase::ManipulatorPtr manip=_robot->GetActiveManipulator();
    OpenRAVE::RaveGetAffineDOFValuesFromTransform(start_pos.begin(), _robot->GetLink("mico_end_effector")->GetTransform(), OpenRAVE::DOF_Transform);
    std::vector<OpenRAVE::dReal> goal_vals;
    goal_vals = _params->vgoalconfig;

    // it's in meter, so this much for an upper limit is fine
    double min_distance=999999;
    double distance = 0;
    if (goal_vals.size()%7 != 0) {
        RAVELOG_ERROR("[SBPLBasePlanner] wrong goal size : %d", goal_vals.size());
    }
    for (int k=0; k<goal_vals.size()/7;k++) {
        double d1 = abs((start_pos[0]-goal_vals[7*k])*(start_pos[0]-goal_vals[7*k])*1000);
        double d2 = abs((start_pos[1]-goal_vals[7*k+1])*(start_pos[1]-goal_vals[7*k+1])*1000);
        double d3 = abs((start_pos[2]-goal_vals[7*k+2])*(start_pos[2]-goal_vals[7*k+2])*1000);
        distance = sqrtf(d1 + d2 + d3 );
        if (distance < min_distance ) {min_distance=distance; }
    }

    // adaptive steps. This can be tune
    std::string sf;

    if (min_distance > 7 ) { sf = s + s6; }
    else {
        if ( distance > 4 ) {sf = s + s4;}
        else { sf = s + s2; }
    }

    // Getting back the info from the yaml file
    std::cout << sf << std::endl;
    YAML::Node doc = YAML::LoadFile(sf);
    cellsize = doc["cellsize"].as<double>();
    linear_weight = doc["linear_weight"].as<double>();
    mode_weight = doc["mode_weight"].as<double>();
    doc["extents"] >> extents;
    angle_weight = doc["angle_weight"].as<double>();
    numangles = doc["numangles"].as<int>();
    nummodes = doc["nummodes"].as<int>();
    doc["actions"] >> actions;
    _maxtime = doc["timelimit"].as<double>();


    RAVELOG_INFO("[SBPLBasePlanner] n_axes : %d\n", n_axes);
    RAVELOG_INFO("[SBPLBasePlanner] extents : %0.3f\n", extents.xmin);
    RAVELOG_INFO("[SBPLBasePlanner] angle weight: %0.3f\n", angle_weight);
    RAVELOG_INFO("[SBPLBasePlanner] mode weight: %0.3f\n", mode_weight);
    RAVELOG_INFO("[SBPLBasePlanner] Cellsize: %0.3f\n", cellsize);
    RAVELOG_INFO("[SBPLBasePlanner] Num angles: %d\n", numangles);
    RAVELOG_INFO("[SBPLBasePlanner] Time limit: %0.3f\n", _maxtime);
    RAVELOG_INFO("[SBPLBasePlanner] Return first: %s\n", (_return_first ? "True" : "False") );

    _env->Initialize(cellsize, extents, numangles, actions, linear_weight, angle_weight, mode_weight, nummodes); //env qu'on def comme on veut, ici 7d
    _planner = boost::make_shared<ADPlanner>(_env.get(), true);

    return true;
}

bool SBPLBasePlanner::InitPlan(OpenRAVE::RobotBasePtr robot, std::istream& input) {
}



OpenRAVE::PlannerStatus SBPLBasePlanner::PlanPath(OpenRAVE::TrajectoryBasePtr ptraj) {

    RAVELOG_INFO("[SBPLBasePlanner] Time limit: %0.3f\n", _maxtime);
    RAVELOG_INFO("[SBPLBasePlanner] Begin PlanPath\n");

    OpenRAVE::PlannerStatus planner_status;
    // Set the goals values
    planner_status = init_plan();

    ReplanParams rparams(_maxtime);
    rparams.initial_eps = _epsinit;
    rparams.dec_eps = _epsdec;
    rparams.return_first_solution = _return_first;
    rparams.max_time = _maxtime;
    std::vector<int> plan;
    std::vector<OpenRAVE::dReal> start_pos(6);

    // get the start values from the end effector tranform, after applying tranforms
    get_start_val( start_pos );
    print_start_cart(start_pos);

    std::vector<float> mode_cost;

    // get the best mode
    planner_status = best_mode( mode_cost, rparams, ptraj, plan, start_pos);

    // if it fails, you need to tell it to the python files. 
    if ( planner_status == OpenRAVE::PS_Failed ) {
        for (int i=0;i<_n_modes;i++) {
            _cost.push_back(-1);
        }
    }
    else {
        for (int i=0;i<_n_modes;i++) {
            _cost.push_back(mode_cost[i]);
        }
    }
    return planner_status;
}


OpenRAVE::PlannerStatus SBPLBasePlanner::best_mode( std::vector<float> &mode_cost, ReplanParams rparams, 
    OpenRAVE::TrajectoryBasePtr ptraj, std::vector<int>& plan2, std::vector<OpenRAVE::dReal> start_pos ) {

if ( goal_achieved(start_pos) == true) {
    for (int i=0;i<_n_modes;i++) { mode_cost.push_back(0); }
        OpenRAVE::ConfigurationSpecification config_spec = OpenRAVE::RaveGetAffineConfigurationSpecification(OpenRAVE::DOF_Transform,_robot, "linear");
        config_spec.AddDerivativeGroups(1, true);  //velocity group, add delta time group
        ptraj->Init(config_spec);
        return OpenRAVE::PS_HasSolution;
       }
else {
    try{
    int start_id;
    int solved;
    int mode_min = 0;
    int mode_max = 0;
    if ( _n_modes == 3 ) {
        mode_min = 1;
        mode_max = 3;
    }
    else {
       if ( _n_modes == 2 ) {
        mode_min = 5;
        mode_max = 6;
    } 
    else {
        RAVELOG_ERROR("[SBPLBasePlanner] [best_mode] Wrong _n_modes\n");
        return OpenRAVE::PS_Failed;
        }
    }

    // Loop for each mode
    for (int compteur_mode=mode_min; compteur_mode <= mode_max; compteur_mode++) {

        std::vector<int> plan;
        start_id = _env->SetStart(start_pos[0], start_pos[1], start_pos[2],start_pos[3], start_pos[4], start_pos[5], compteur_mode);
        if( start_id < 0 || _planner->set_start(start_id) == 0){
            RAVELOG_ERROR("[SBPLBasePlanner] [best_mode] Failed to set start state\n");
            return OpenRAVE::PS_Failed;
        }

        solved = _planner->replan(&plan, rparams);
        RAVELOG_INFO("[SBPLBasePlanner] Solved? %d\n", solved);

        if ( solved==0) {
            RAVELOG_ERROR("[SBPLBasePlanner] SBPL unable to find solution in allocated time\n");
            std::cout <<"too short on time"<<std::endl;
            //we dont really have solution, but we know it
            return OpenRAVE::PS_Failed;
        }
        if( solved==1 ){
        OpenRAVE::ConfigurationSpecification config_spec = OpenRAVE::RaveGetAffineConfigurationSpecification(OpenRAVE::DOF_Transform,
               _robot, "linear");
        config_spec.AddDerivativeGroups(1, true);  //velocity group, add delta time group
        ptraj->Init(config_spec);
        std::vector<PlannedWaypointPtr> xyzA_path;

        // just give back the last path found
        _env->ConvertStateIDPathIntoWaypointPath(plan, xyzA_path, _path_cost, _cart_path, _list_actions);         
        mode_cost.push_back(_path_cost);
        }
    }
    return OpenRAVE::PS_HasSolution;
    }
    catch( SBPL_Exception e ){
     RAVELOG_ERROR("[SBPLBasePlanner] SBPL encountered fatal exception while searching the best mode\n");
     return OpenRAVE::PS_Failed;
     }
    }
}

/* Setup the goal point for the plan */
OpenRAVE::PlannerStatus SBPLBasePlanner::init_plan( ) {

    // No use here; might be useful if you're reusing the same planner with different start positions a lot of time
    // _planner->force_planning_from_scratch_and_free_memory();

    try{
        std::vector<OpenRAVE::dReal> goal_vals;
        goal_vals = _params->vgoalconfig;
        std::vector<int> goals_id = _env->SetGoal(goal_vals);

        bool bset_goal=true;
        for (int i=0;i<goals_id.size();i++) {
            if (_planner->set_goal(goals_id[i]) == 0 )
                bset_goal=false;
        }

        if( goals_id.size() == 0 || bset_goal==false) {
            RAVELOG_ERROR("[SBPLBasePlanner] Failed to set goal state\n");
            return OpenRAVE::PS_Failed;
        }

    } catch( SBPL_Exception e ){
        RAVELOG_ERROR("[SBPLBasePlanner] SBPL encountered fatal exception while setting the goal state\n");
        return OpenRAVE::PS_Failed;
    }
}


bool SBPLBasePlanner::get_start_val( std::vector<OpenRAVE::dReal>& start_pos ) {

    OpenRAVE::RaveGetAffineDOFValuesFromTransform(start_pos.begin(), 
        _robot->GetLink("mico_end_effector")->GetTransform(), OpenRAVE::DOF_Transform);

    OpenRAVE::RaveTransform< OpenRAVE::dReal > ee_trans = _robot->GetLink("mico_end_effector")->GetTransform();
    OpenRAVE::RaveTransform< OpenRAVE::dReal > centered_wrist_trans;
    // Translation to the center of the global frame
    centered_wrist_trans = ee_trans;
    centered_wrist_trans.trans.x = 0;
    centered_wrist_trans.trans.y = 0;
    centered_wrist_trans.trans.z = 0;

    // Definition of the reference
    OpenRAVE::geometry::RaveVector< OpenRAVE::dReal > target_rotation = OpenRAVE::geometry::RaveVector< OpenRAVE::dReal >(0.604918, 0.583349, 0.398589, 0.367293);
    OpenRAVE::geometry::RaveVector< OpenRAVE::dReal > target_translation = OpenRAVE::geometry::RaveVector< OpenRAVE::dReal >(0.,0.,0.);
    OpenRAVE::RaveTransform< OpenRAVE::dReal > target_transform = OpenRAVE::RaveTransform< OpenRAVE::dReal >(target_rotation, target_translation);

    // Inverse
    centered_wrist_trans.rot.y = -centered_wrist_trans.rot.y;
    centered_wrist_trans.rot.z = -centered_wrist_trans.rot.z;
    centered_wrist_trans.rot.w = -centered_wrist_trans.rot.w;

    // Comparison of the angles between the centered ee tranform and the reference transform
    OpenRAVE::RaveTransform< OpenRAVE::dReal > result_trans;
    result_trans = OpenRAVE::RaveTransform< OpenRAVE::dReal >( centered_wrist_trans * target_transform  );

    double q0 = result_trans.rot.x;
    double q1 = result_trans.rot.y;
    double q2 = result_trans.rot.z;
    double q3 = result_trans.rot.w;

    double phi = atan2(2*(q0*q1 + q2*q3), 1-2*(q1*q1 + q2*q2));
    double theta = asin(2*(q0*q2 - q3*q1));
    double psi = atan2(2*(q0*q3 + q1*q2), 1-2*(q2*q2 + q3*q3));

    // Apply a rotation of -Psi to the quaternions
    OpenRAVE::geometry::RaveVector< OpenRAVE::dReal > quat_rot_psi = OpenRAVE::geometry::RaveVector< OpenRAVE::dReal >(sin( psi / 2.0 ),0,0,cos( psi / 2.0 ));
    // Compare to the reference transform
    OpenRAVE::geometry::RaveVector< OpenRAVE::dReal > quat_rot_result_trans = quat_mult(quat_rot_psi, result_trans.rot);

    q0 = quat_rot_result_trans[0];
    q1 = quat_rot_result_trans[1];
    q2 = quat_rot_result_trans[2];
    q3 = quat_rot_result_trans[3];

    double phi_rot = atan2(2*(q0*q1 + q2*q3), 1-2*(q1*q1 + q2*q2));
    double theta_rot = asin(2*(q0*q2 - q3*q1));
    double psi_rot = atan2(2*(q0*q3 + q1*q2), 1-2*(q2*q2 + q3*q3));

    // Done ! We finally got what we want : 
    // Wrist rotation in the centered frame
    // Wrist oritentation in the hand frame
    start_pos[3] = phi_rot;
    start_pos[4] = theta_rot;
    start_pos[5] = psi;

    return true;
}

// Muliplication of 2 quaterniosn
OpenRAVE::geometry::RaveVector< OpenRAVE::dReal > SBPLBasePlanner::quat_mult(OpenRAVE::geometry::RaveVector< OpenRAVE::dReal > q1, OpenRAVE::geometry::RaveVector< OpenRAVE::dReal > q2) {
    OpenRAVE::geometry::RaveVector< OpenRAVE::dReal > mult;
    mult[0] = q1[3]*q2[0] - q1[2]*q2[1] + q1[1]*q2[2] + q1[0]*q2[3];
    mult[1] = q1[2]*q2[0] + q1[3]*q2[1] - q1[0]*q2[2] + q1[1]*q2[3];
    mult[2] = -q1[1]*q2[0] + q1[0]*q2[1] + q1[3]*q2[2] + q1[2]*q2[3];
    mult[3] = -q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] + q1[3]*q2[3];
    return mult;
}

// Check if we are actually at the goal
bool SBPLBasePlanner::goal_achieved( std::vector<OpenRAVE::dReal> start_pos) {
    std::vector<OpenRAVE::dReal> goal_vals;
    goal_vals = _params->vgoalconfig;

    // values depend of the smalest grid size
    float eps1 = 0.015;
    float eps2 = 0.45;
    for (int i = 0; i<goal_vals.size()/7; i++) {
        if ( (fabs(goal_vals[0]-start_pos[0]) < eps1) && (fabs(goal_vals[1]-start_pos[1]) < eps1) && (fabs(goal_vals[2]-start_pos[2]) < eps1) &&
            (fabs(goal_vals[3]-start_pos[3]) < eps2) && (fabs(goal_vals[4]-start_pos[4]) < eps2) && (fabs(goal_vals[5]-start_pos[5]) < eps2) ) {
            std::printf("Goal achieved");
        return true;
    }
}
return false;
}


void SBPLBasePlanner::AddWaypoint(OpenRAVE::TrajectoryBasePtr ptraj, const OpenRAVE::ConfigurationSpecification &config_spec, 
  const double &x, const double &y, const double &z, const double &phi, const double &theta, const double &psi, const int &mode) const {

    //Not working yet

    // Create a trajectory point
    std::vector<double> point;
    point.resize(config_spec.GetDOF());

    // Get group offsets
    OpenRAVE::ConfigurationSpecification::Group dt_group = config_spec.GetGroupFromName("deltatime");
    int time_group = dt_group.offset;
    OpenRAVE::ConfigurationSpecification::Group affine_group = config_spec.GetGroupFromName("affine_transform");
    int affine_offset = affine_group.offset;

    // Manually set the timing
    int idx = ptraj->GetNumWaypoints();
    point[time_group] = idx;

    // Set the affine values

    double Mphi[3][3];
    double Mtheta[3][3];
    double Mpsi[3][3];

    Mphi[0][0]=1.;
    Mphi[0][1]=0.;
    Mphi[0][2]=0.;
    Mphi[1][0]=0.;
    Mphi[1][1]=OpenRAVE::RaveCos(phi);
    Mphi[1][2]=-OpenRAVE::RaveSin(phi);
    Mphi[3][0]=0.;
    Mphi[3][1]=OpenRAVE::RaveSin(phi);
    Mphi[3][2]=OpenRAVE::RaveCos(phi);

    Mtheta[0][0]=OpenRAVE::RaveCos(theta);
    Mtheta[0][1]=0.;
    Mtheta[0][2]=OpenRAVE::RaveSin(theta);
    Mtheta[1][0]=0.;
    Mtheta[1][1]=1.;
    Mtheta[1][2]=0.;
    Mtheta[3][0]=-OpenRAVE::RaveSin(theta);
    Mtheta[3][1]=0.;
    Mtheta[3][2]=OpenRAVE::RaveCos(theta);

    Mpsi[0][0]=OpenRAVE::RaveCos(psi);
    Mpsi[0][1]=-OpenRAVE::RaveSin(psi);
    Mpsi[0][2]=0.;
    Mpsi[1][0]=OpenRAVE::RaveSin(psi);
    Mpsi[1][1]=OpenRAVE::RaveCos(psi);
    Mpsi[1][2]=0.;
    Mpsi[3][0]=0.;
    Mpsi[3][1]=0.;
    Mpsi[3][2]=1.;

    double res_temp[3][3];
    double res[3][3];
    multiply(Mpsi,Mtheta,res);
    multiply(res_temp,Mphi,res);

    OpenRAVE::RaveTransformMatrix<double> R;
    R.rotfrommat(res[0][0], res[0][1], res[0][2],
     res[1][0], res[1][1] ,res[1][2] ,
     res[2][0],res[2][1] ,res[2][2] );

    OpenRAVE::RaveVector<double> trans(x, y, z); 
    OpenRAVE::RaveTransform<double> transform(OpenRAVE::geometry::quatFromMatrix(R), trans);
    
    OpenRAVE::RaveGetAffineDOFValuesFromTransform(point.begin() + affine_offset, 
      transform, 
      OpenRAVE::DOF_Transform);

    // Insert the point
    ptraj->Insert(idx, point, true);
    
}

// state 0 : defined by hand, state 1 and next : the simulate position
bool SBPLBasePlanner::GetCartPath(std::ostream &out, std::istream &in){

    RAVELOG_INFO("[SBPLBasePlanner] Using GetCartPath\n");

    YAML::Emitter emitter;

    emitter << YAML::BeginSeq;
    for(int i=0;i<_cart_path.size();i++) {
      emitter << YAML::BeginMap;
      emitter << YAML::Key << "state";
      emitter << YAML::Value << i;
      emitter << YAML::Key << "position";
      emitter << YAML::Value << YAML::BeginSeq << 
      _cart_path[i].x << _cart_path[i].y <<_cart_path[i].z << 
      _cart_path[i].phi <<_cart_path[i].theta << _cart_path[i].psi <<
      _cart_path[i].mode << YAML::EndSeq;
      emitter << YAML::EndMap;
  }
  emitter << YAML::EndSeq;

  out << emitter.c_str();
  return true;
}

// step x mean it's the action to go from state x to state x+1
bool SBPLBasePlanner::GetListActions(std::ostream &out, std::istream &in){

    RAVELOG_INFO("[SBPLBasePlanner] Using GetListActions\n");

    YAML::Emitter emitter;

    emitter << YAML::BeginSeq;
        for(int i=0;i<_cart_path.size()-2;i++) { // -1 because one less action than number of states
          emitter << YAML::BeginMap;
          emitter << YAML::Key << "step";
          emitter << YAML::Value << i+1;
          emitter << YAML::Key << "action";
          emitter << YAML::Value << YAML::BeginSeq << 
          _list_actions[i].x << _list_actions[i].y <<_list_actions[i].z << 
          _list_actions[i].phi <<_list_actions[i].theta << _list_actions[i].psi <<
          _list_actions[i].mode << YAML::EndSeq;
          emitter << YAML::EndMap;
      }
      emitter << YAML::EndSeq;

      out << emitter.c_str();
      return true;
  }

  void SBPLBasePlanner::print_start_DOF() {
    std::vector<OpenRAVE::dReal> v;
    _robot->GetDOFValues(v);

    std::cout << "_robot->GetDOFValues() : " ;
    for (int temp1=0;temp1<v.size();temp1++) {
        std::cout << v[temp1] << " ";
    }
}

void SBPLBasePlanner::print_start_cart(std::vector<OpenRAVE::dReal> start_pos) {
    std::cout << "start cart values : " ;
    for (int i=0;i<start_pos.size();i++) {
        std::cout << start_pos[i] << " ";
    }
    std::cout <<std::endl;
}
bool SBPLBasePlanner::GetPathsCosts(std::ostream &out, std::istream &in){

    RAVELOG_INFO("[SBPLBasePlanner] Using GetPathsCost\n");
    YAML::Emitter emitter;
    emitter << YAML::BeginSeq;
    for (int i=0; i<_cost.size();i++ ) {
        emitter << _cost[i];
    }
    emitter << YAML::EndSeq;
    out << emitter.c_str();
    return true;
}