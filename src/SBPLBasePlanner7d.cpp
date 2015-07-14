#include <or_sbpl_for_ada/SBPLBasePlanner7d.h>
#include <or_sbpl_for_ada/SBPLBasePlannerTypes7d.h>
#include <or_sbpl_for_ada/Action7d.h> 
#include <or_sbpl_for_ada/YamlUtils2.h>

#include <boost/unordered_map.hpp>
#include <boost/foreach.hpp>
#include <boost/make_shared.hpp>
// #include <sbpl/planners/araplanner.h>
#include <sbpl/planners/adplanner.h>
#include <sbpl/planners/planner.h>
#include <sbpl/utils/utils.h>

#include <yaml-cpp/yaml.h>
#include <fstream>

using namespace or_sbpl_for_ada;

SBPLBasePlanner::SBPLBasePlanner(OpenRAVE::EnvironmentBasePtr penv) :
OpenRAVE::PlannerBase(penv), _orenv(penv), _initialized(false), _maxtime(1), _path_cost(-1.0),
_epsinit(100), _epsdec(1.0), _return_first(false) {

    RegisterCommand("GetPathCost", boost::bind(&SBPLBasePlanner::GetPathCost, this, _1, _2),
        "Get the cost of the plan");
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
    _env = boost::make_shared<SBPLBasePlannerEnvironment>(robot); //7d

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

#ifdef YAMLCPP_NEWAPI

    YAML::Node doc = YAML::Load(extra_stream);

    cellsize = doc["cellsize"].as<double>();
    linear_weight = doc["linear_weight"].as<double>();
    mode_weight = doc["mode_weight"].as<double>();
    doc["extents"] >> extents;
    angle_weight = doc["angle_weight"].as<double>();
    numangles = doc["numangles"].as<int>();
    nummodes = doc["nummodes"].as<int>();
    doc["actions"] >> actions;
    _maxtime = doc["timelimit"].as<double>();
    n_axes = doc["n_axes"].as<int>();
    _n_axes = n_axes;
#else

    // Parse the extra parameters as yaml
    YAML::Parser parser(extra_stream);
    YAML::Node doc;
    parser.GetNextDocument(doc);

    RAVELOG_INFO("[SBPLBasePlanner] Parsing\n");
    doc["n_axes"] >> n_axes;
    doc["linear_weight"] >> linear_weight;
    doc["angle_weight"] >> angle_weight;
    doc["mode_weight"] >> mode_weight;
    doc["extents"] >> extents;
    doc["cellsize"] >> cellsize;
    doc["numangles"] >> numangles;
    doc["nummodes"] >> nummodes;
    doc["actions"] >> actions;
    doc["timelimit"] >> _maxtime;
    _n_axes = n_axes;

    if (YAML::Node const *init_eps = doc.FindValue("initial_eps")) {
        *init_eps >> _epsinit;
        RAVELOG_INFO("[SBPLBasePlanner] Initial epsilon: %0.3f\n", _epsinit);
    }


    if (YAML::Node const *dec_eps = doc.FindValue("dec_eps")) {
        *dec_eps >> _epsdec;
        RAVELOG_INFO("[SBPLBasePlanner] Epsilon decrement: %0.3f\n", _epsdec);
    }

    if (YAML::Node const *return_first = doc.FindValue("return_first")) {
        int rfirst;
        *return_first >> rfirst;
        _return_first = (rfirst == 1);
    }
#endif

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
   // std::cout << "_robot->GetTransform() : " << _robot->GetTransform() << std::endl;

    OpenRAVE::PlannerStatus planner_status;
    planner_status = init_plan();

    ReplanParams rparams(_maxtime);
    rparams.initial_eps = _epsinit;
    rparams.dec_eps = _epsdec;
    rparams.return_first_solution = _return_first;
    rparams.max_time = _maxtime;

    // start_listener();

//while(true) {
for ( int temp=0; temp < 20; temp++) {
    std::vector<OpenRAVE::dReal> start_vals(6);

    OpenRAVE::RobotBase::ManipulatorPtr manip=_robot->GetActiveManipulator();

    OpenRAVE::RaveGetAffineDOFValuesFromTransform(start_vals.begin(),
        manip->GetEndEffectorTransform(), OpenRAVE::DOF_Transform);

    //print_start_DOF();
    print_start_cart(start_vals);

    std::vector<float> mode_cost;
    planner_status = best_mode(start_vals, mode_cost, rparams);
    }
    return planner_status;
}


OpenRAVE::PlannerStatus SBPLBasePlanner::best_mode( std::vector<OpenRAVE::dReal> start_vals, std::vector<float> &mode_cost, ReplanParams rparams ) {
try{
    int start_id;
    int solved;
    for (int compteur_mode=1;compteur_mode<4;compteur_mode++) {
      //  std::cout << "compteur : "<<compteur_mode<<std::endl;
        std::vector<int> plan;
        start_id = _env->SetStart(start_vals[0], start_vals[1], start_vals[2],start_vals[3], start_vals[4], start_vals[5], compteur_mode);
        
        if( start_id < 0 || _planner->set_start(start_id) == 0){
            RAVELOG_ERROR("[SBPLBasePlanner] [best_mode] Failed to set start state\n");
            return OpenRAVE::PS_Failed;
        }
        solved = _planner->replan(&plan, rparams);
   //  std::cout << "solved ? "<< solved2 << std::endl;
        if ( solved==0) {
            RAVELOG_ERROR("[SBPLBasePlanner] SBPL unable to find solution in allocated time\n");
            std::cout << "Compteur_mode : "<<compteur_mode<<std::endl;
            return OpenRAVE::PS_Failed;
        }
        mode_cost.push_back(plan.size());
     //   std::cout <<"cost : "<<plan.size()<<std::endl;
    }
    return OpenRAVE::PS_HasSolution;
    } catch( SBPL_Exception e ){
        RAVELOG_ERROR("[SBPLBasePlanner] SBPL encountered fatal exception while searching the best mode\n");
        return OpenRAVE::PS_Failed;
    }
}

/* Setup the goal point for the plan */
OpenRAVE::PlannerStatus SBPLBasePlanner::init_plan( ) {

    // _planner->force_planning_from_scratch_and_free_memory();

    try{

        std::vector<OpenRAVE::dReal> goal_vals;
        goal_vals = _params->vgoalconfig;
        /*
        if(goal_vals.size()%7 != 0){
            RAVELOG_ERROR("[SBPLBasePlanner] Unable to extract goal of appropriate size.\n");
            return OpenRAVE::PS_Failed;
        }
        */
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








/*
 * Creates a waypoint and adds it to the trajectory
 * 
 * @param ptraj The trajectory to append the waypoint to
 * @param config_spec The configuration specification for the trajectory
 * @param t The delta time value
 * @param x The x location
 * @param y The y location
 * @param theta The orientation
 */
 void SBPLBasePlanner::AddWaypoint(OpenRAVE::TrajectoryBasePtr ptraj, const OpenRAVE::ConfigurationSpecification &config_spec, 
  const double &x, const double &y, const double &z, const double &phi, const double &theta, const double &psi, const int &mode) const {

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

bool SBPLBasePlanner::GetPathCost(std::ostream &out, std::istream &in){

    RAVELOG_INFO("[SBPLBasePlanner] Using GetPathCost\n");
    YAML::Emitter emitter;
    emitter << YAML::BeginSeq;
    emitter << _path_cost;
    emitter << YAML::EndSeq;
    out << emitter.c_str();
    return true;
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

void SBPLBasePlanner::print_start_cart(std::vector<OpenRAVE::dReal> start_vals) {
    std::cout <<std::endl;
    std::cout << "start cart values : " ;
    for (int temp2=0;temp2<start_vals.size();temp2++) {
        std::cout << start_vals[temp2] << " ";
    }
    std::cout <<std::endl;
}

void SBPLBasePlanner::chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

void SBPLBasePlanner::start_listener()
{
  // ros::init(argc, argv, "listener_start_pos");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("start_pos", 10, &SBPLBasePlanner::chatterCallback, this);
  ros::spin();
}