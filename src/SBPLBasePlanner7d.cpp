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
OpenRAVE::PlannerBase(penv), _orenv(penv), _initialized(false), _maxtime(3), _path_cost(-1.0),
_epsinit(3), _epsdec(1.0), _return_first(false) {

    _cost[0]=0;
    _cost[1]=0;
    _cost[2]=0;

    RegisterCommand("GetPathCost", boost::bind(&SBPLBasePlanner::GetPathCost, this, _1, _2),
        "Get the cost of the plan");
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
    _maxtime_temp = _maxtime;
    return true;
}

bool SBPLBasePlanner::InitPlan(OpenRAVE::RobotBasePtr robot, std::istream& input) {

}



OpenRAVE::PlannerStatus SBPLBasePlanner::PlanPath(OpenRAVE::TrajectoryBasePtr ptraj) {

    RAVELOG_INFO("[SBPLBasePlanner] Time limit: %0.3f\n", _maxtime);
    RAVELOG_INFO("[SBPLBasePlanner] Begin PlanPath\n");
   // std::cout << "_robot->GetTransform() : " << _robot->GetTransform() << std::endl;

    _planner->force_planning_from_scratch_and_free_memory();

    OpenRAVE::PlannerStatus planner_status;

    _maxtime = _maxtime_temp; // bug : _maxtime from InitPlan not remembered. Temporary fix
    planner_status = init_plan();

    ReplanParams rparams(_maxtime);
    rparams.initial_eps = _epsinit;
    rparams.dec_eps = _epsdec;
    rparams.return_first_solution = _return_first;
    rparams.max_time = _maxtime;
    
    std::vector<int> plan;

    std::vector<OpenRAVE::dReal> start_pos(6);
/*
So it's the transform that you are sending that is wrong. Try replacing
manip->GetEndEffectorTransform()
with
_robot->GetLink("mico_end_effector")->GetTransform()'
*/
    OpenRAVE::RobotBase::ManipulatorPtr manip=_robot->GetActiveManipulator();
    OpenRAVE::RaveGetAffineDOFValuesFromTransform(start_pos.begin(), _robot->GetLink("mico_end_effector")->GetTransform(), OpenRAVE::DOF_Transform);
    //OpenRAVE::RaveGetAffineDOFValuesFromTransform(start_pos.begin(), manip->GetEndEffectorTransform(), OpenRAVE::DOF_Transform);

    //print_start_DOF();    
    //print_start_cart(start_pos);

    std::vector<float> mode_cost;
    
    planner_status = best_mode( mode_cost, rparams, ptraj, plan, start_pos);

  //  std::cout <<"Planned for the start pos. mode cost : "
  //      << mode_cost[0]<<" "<<mode_cost[1]<<" "<<mode_cost[2]<<std::endl;

    _cost[0]=mode_cost[0];
    _cost[1]=mode_cost[1];
    _cost[2]=mode_cost[2];

    return planner_status;
}


OpenRAVE::PlannerStatus SBPLBasePlanner::best_mode( std::vector<float> &mode_cost, ReplanParams rparams, 
    OpenRAVE::TrajectoryBasePtr ptraj, std::vector<int>& plan2, std::vector<OpenRAVE::dReal> start_pos ) {

    try{
        int start_id;
        int solved;
        for (int compteur_mode=1;compteur_mode<4;compteur_mode++) {
       // print_start_cart();
      //  std::cout << "compteur : "<<compteur_mode<<std::endl;
        //plan.clear();
            std::vector<int> plan;

            start_id = _env->SetStart(start_pos[0], start_pos[1], start_pos[2],start_pos[3], start_pos[4], start_pos[5], compteur_mode);

            if( start_id < 0 || _planner->set_start(start_id) == 0){
                RAVELOG_ERROR("[SBPLBasePlanner] [best_mode] Failed to set start state\n");
                return OpenRAVE::PS_Failed;
            }

    //    std::cout << "start planning" << std::endl;
            solved = _planner->replan(&plan, rparams);
    //    std::cout << "end planning" << std::endl;


            RAVELOG_INFO("[SBPLBasePlanner] Solved? %d\n", solved);

        //std::cout<<"compteur : "<<compteur_mode <<" ;  plan size : "<<plan.size()<<std::endl; 
     // std::cout << "solved ? "<< solved << std::endl;

            if ( solved==0) {
                RAVELOG_ERROR("[SBPLBasePlanner] SBPL unable to find solution in allocated time\n");
        //    std::cout << "Compteur_mode : "<<compteur_mode<<std::endl;
                return OpenRAVE::PS_Failed;
            }


            if( solved==1 ){

                OpenRAVE::ConfigurationSpecification config_spec = OpenRAVE::RaveGetAffineConfigurationSpecification(OpenRAVE::DOF_Transform,
                 _robot, "linear");
        config_spec.AddDerivativeGroups(1, true);  //velocity group, add delta time group
        ptraj->Init(config_spec);
        std::vector<PlannedWaypointPtr> xyzA_path;

        _env->ConvertStateIDPathIntoWaypointPath(plan, xyzA_path, _path_cost, _cart_path, _list_actions);         
        mode_cost.push_back(_path_cost);
    }

}

return OpenRAVE::PS_HasSolution;
}catch( SBPL_Exception e ){
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

bool SBPLBasePlanner::GetPathsCosts(std::ostream &out, std::istream &in){

    RAVELOG_INFO("[SBPLBasePlanner] Using GetPathsCost\n");

    YAML::Emitter emitter;

    emitter << YAML::BeginSeq;


    emitter << _cost[0] << _cost[1] << _cost[2];


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
    std::cout <<std::endl;
    std::cout << "start cart values : " ;
    for (int temp2=0;temp2<start_pos.size();temp2++) {
        std::cout << start_pos[temp2] << " ";
    }
    std::cout <<std::endl;
}

/*
void SBPLBasePlanner::chatterCallback(const std_msgs::String::ConstPtr& msg)
{

    // ROS_INFO("I heard: [%s]", msg->data.c_str());
    _start_pos.clear();
    std::istringstream iss(msg->data.c_str());
    std::copy(std::istream_iterator<float>(iss), std::istream_iterator<float>(), std::back_inserter(_start_pos));
    std::cout <<"listened to start pos. size : "<< _start_pos.size() << std::endl;
}

void SBPLBasePlanner::start_listener()
{
  // ros::init(argc, argv, "listener_start_pos");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("start_pos", 10, &SBPLBasePlanner::chatterCallback, this);
  ros::spin();
}
*/

ros::Publisher SBPLBasePlanner::init_path_cost_publisher()
{
  ros::NodeHandle n;
  ros::Publisher path_cost_pub = n.advertise<std_msgs::String>("path_cost", 10);

  return path_cost_pub;
}

std_msgs::String SBPLBasePlanner::floatToStringToPub( std::vector<float> mode_cost ) {

    std::string cost_string;
    for ( int i=0;i<mode_cost.size();i++) {
        cost_string = cost_string + boost::to_string(mode_cost[i]) + " ";
    }

    std_msgs::String msg;
    std::stringstream ss;
    ss << cost_string;
    msg.data = ss.str();
    return msg;
}