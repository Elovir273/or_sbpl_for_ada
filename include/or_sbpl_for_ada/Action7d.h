#ifndef SBPL_ACTION7d_H_
#define SBPL_ACTION7d_H_

#include <or_sbpl_for_ada/SBPLBasePlannerTypes7d.h>
#include <boost/unordered_map.hpp>
#include "yaml-cpp/yaml.h"

namespace or_sbpl_for_ada {

    /**
     * An action that is parameterized as a cached list of points that make up the action
     */
    class Action7d : public Action {

    public: 
	/**
	 * Constructor.
	 */
        Action7d();

        /**
         * Constructor. 
         *
         * @param pts The list of points that comprise this action
	 * @param weight The weight to apply to the action cost
         */
        Action7d(const std::vector<WorldCoordinate> &pts, const double &weight);

        /**
         * Applies the action, checking for collision at each cached point
         *
         * @param wc The pose to start action propagation from
         * @param robot The robot to apply the twist to
         * @param final_wc The world coordinate that the robot reaches by applying the action
         * @return True if the action was successful. False otherwise.
         */
        virtual bool apply(const WorldCoordinate &wc, const OpenRAVE::RobotBasePtr &robot, WorldCoordinate &final_wc) const;
 
        /**
         * Applies the action
         *
         * @param wc The pose to start action propagation from
         * @param robot The robot to apply the twist to
         * @return A list of all valid poses visited during execution of the action
         */
        virtual std::vector<WorldCoordinate> applyWithIntermediates(const WorldCoordinate &wc,
                                                                    const OpenRAVE::RobotBasePtr &robot) const;
	
	/**
	 * @return The list of points in the action
	 */
	virtual std::vector<WorldCoordinate> getPoints() const { return _pts; }

    private:
        std::vector<WorldCoordinate> _pts;

    };
    typedef boost::shared_ptr<Action7d> Action7dPtr;
}

#endif
