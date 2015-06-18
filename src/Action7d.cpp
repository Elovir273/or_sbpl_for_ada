#include <or_sbpl_for_ada/Action7d.h>
#include <or_sbpl_for_ada/SBPLBasePlannerTypes7d.h>
#include <boost/foreach.hpp>
#include <boost/make_shared.hpp>

using namespace or_sbpl_for_ada;

Action7d::Action7d() {
    setWeight(1.0);
}

Action7d::Action7d(const std::vector<WorldCoordinate> &pts, const double &weight) :
    _pts(pts) {
    setName("7d");
    setWeight(weight);
}

bool Action7d::apply(const WorldCoordinate &wc, const OpenRAVE::RobotBasePtr &robot, WorldCoordinate &final_wc) const {
    
    bool valid = false;
    std::vector<WorldCoordinate> intermediates = applyWithIntermediates(wc, robot);
    if(intermediates.size() > 0){
        final_wc = intermediates.back();
        valid = true;
    }

    return valid;
}

std::vector<WorldCoordinate> Action7d::applyWithIntermediates(const WorldCoordinate &wc,
                                                                  const OpenRAVE::RobotBasePtr &robot) const {
std::vector<WorldCoordinate> intermediates;

    // Create a robot state saver
    OpenRAVE::RobotBase::RobotStateSaver rStateSaver(robot);

    // Initialize the list of intermediate points
    
    bool valid = true;
    WorldCoordinate next_pos=_pts[0];
    WorldCoordinate wc_current(wc);

/*
        if ( next_pos.mode != wc.mode ) {
            if ( next_pos.x==0 && next_pos.y==0 && next_pos.z==0 && next_pos.theta==0 && next_pos.phi==0 && next_pos.psi == 0 ) {
                wc_current.mode=next_pos.mode;
            }
            else {
            intermediates.clear();
            return intermediates;
            }
        }
        else {
*/
        wc_current.x += next_pos.x;
        wc_current.y += next_pos.y;
        wc_current.z += next_pos.z;
        wc_current.phi += next_pos.phi;
        wc_current.theta += next_pos.theta;
        wc_current.psi += next_pos.psi;
        wc_current.mode = next_pos.mode;

        OpenRAVE::Transform transform = wc_current.toTransform();
        robot->SetTransform(transform);
        
        // Check for collision and break out if needed
        bool incollision = robot->GetEnv()->CheckCollision(robot); // env reel 
        valid = !incollision;
        
        if(valid){
            intermediates.push_back(wc_current);
        }
        else{
            intermediates.clear();
            rStateSaver.Restore();
            return intermediates;
        }
    //}
    
    // Restore state
    rStateSaver.Restore();

    return intermediates;

}
