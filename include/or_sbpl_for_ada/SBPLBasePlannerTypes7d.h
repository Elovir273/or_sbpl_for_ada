#ifndef SBPL_BASE_PLANNER_TYPES_H_
#define SBPL_BASE_PLANNER_TYPES_H_

#include <iostream>
#include <boost/format.hpp>
#include <boost/shared_ptr.hpp>
#include <yaml-cpp/yaml.h>
#include <openrave/openrave.h>

namespace or_sbpl_for_ada {

    /**
     * Describes a coordinate in world frame
     */
    class WorldCoordinate {
    public:
	/**
	 * Constructor.  The coordinate values are defaulted to zero.
	 */
    WorldCoordinate() : x(0.0), y(0.0), z(0.0), phi(0.0), theta(0.0), psi(0.0), mode(1) {};

	/**
	 * Constructor
	 *
	 * @param x The x value of the coordinate
	 * @param y The y value of the coordinate
	 * @param theta The theta value of the coordinate
	 */
    WorldCoordinate(const double &x, const double &y, const double &z, const double &phi,const double &theta, const double &psi,  const int mode) : x(x), y(y), z(z), phi(phi), theta(theta),  psi(psi), mode(mode) {};
	
	/**
	 * Copy constructor
	 *
	 * @param wc The coordinate to copy
	 */
    WorldCoordinate(const WorldCoordinate &wc) : x(wc.x), y(wc.y), z(wc.z), phi(wc.phi), theta(wc.theta), psi(wc.psi), mode(wc.mode) {}

	/**
	 * @return The string representation of the coordinate
	 */
        std::string toString() const { return (boost::format("[ %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %d]") % x % y % z % phi % theta % psi % mode).str(); }

        /**
         * Converts this world coordinate to a transform for the robot
         *
         * @return The associated transform
         */
        OpenRAVE::Transform toTransform() const;

	/**
	 * The x value of the coordinate
	 */
        double x; 

	/**
	 * The y value of the coordinate
	 */
        double y;

    /**
	 * The z value of the coordinate
	 */
        double z; 

	/**
	 * The phi value of the coordinate
	 */
        double phi;

	/**
	 * The theta value of the coordinate
	 */
        double theta;

	/**
	 * The psi value of the coordinate
	 */
        double psi;

	/**
	 * The mode value of the coordinate
	 */
        int mode;

	/**
	 * Output stream overload
	 *
	 * @param out The output stream
	 * @param wc The world coordinate to write to the stream
	 */
        friend std::ostream& operator<< (std::ostream &out, WorldCoordinate &wc){
            out << wc.toString();
            return out;
        }
    };

    /**
     * Describes a coordinate in grid space
     */
    class GridCoordinate {
    public:

	/**
	 * Constructor. The coordinate values are defaulted to zero
	 */
    GridCoordinate() : x(0), y(0), z(0), phi(0),theta(0),  psi(0), mode(1) {};
	
	/**
	 * Constructor.
	 *
	 * @param x The x value of the coordinate
	 * @param y The y value of the coordinate
	 * @param theta The theta value of the coordinate
	 */
    GridCoordinate(const int &x, const int &y, const int &z, const int &phi, const int &theta,  const int &psi, const int mode) : x(x), y(y), z(z), phi(phi), theta(theta),  psi(psi), mode(mode) {};

	/**
	 * Copy constructor.
	 *
	 * @param gc The coordinate to copy
	 */
    GridCoordinate(const GridCoordinate &gc) : x(gc.x), y(gc.y), z(gc.z), phi(gc.phi), theta(gc.theta),  psi(gc.psi), mode(gc.mode) {}

	/**
	 * @return The string representation of the coordinate
	 */
        std::string toString() const { return (boost::format("[ %d, %d, %d, %d, %d, %d, %d]") % x % y % z % phi % theta % psi % mode).str(); }

	/**
	 * The x value of the coordinate
	 */
        int x;
	
	/**
	 * The y value of the coordinate
	 */
        int y;

       /**
	 * The z value of the coordinate
	 */
        int z;

	/**
	 * The theta value of the coordinate
	 */
        int phi;
        
	/**
	 * The theta value of the coordinate
	 */
        int theta;

	/**
	 * The theta value of the coordinate
	 */
        int psi;

	/**
	 * The theta value of the coordinate
	 */
        int mode;

	/**
	 * Output stream overload
	 *
	 * @param out The output stream
	 * @param gc The grid coordinate to write to the stream
	 */
        friend std::ostream& operator<< (std::ostream &out, GridCoordinate &gc){
            out << gc.toString();
            return out;
        }
    };

    /**
     * A class to represent an action for the lattice
     */

    class Action {
    public:
	/**
	 * Apply this action
	 *
	 * @param wc The coordinate to begin applying the action from
	 * @param robot The robot to use for collision checking
	 * @param final_wc The final coordinate after the full action is applied (set by this function)
	 * @return True if the action was successfully applied
	 */
        virtual bool apply(const WorldCoordinate &wc, const OpenRAVE::RobotBasePtr &robot, WorldCoordinate &final_wc) const = 0;
	
	/**
	 * Apply this action
	 *
	 * @param wc The coordinate to begin applying the action from
	 * @param robot The robot to use for collision checking
	 * @return A list of all intermediate poses visited by this action
	 */
        virtual std::vector<WorldCoordinate> applyWithIntermediates(const WorldCoordinate &wc, const OpenRAVE::RobotBasePtr &robot) const = 0;

	/**
	 * @return The name for this action
	 */
        std::string getName() const { return _name; }
	
	/**
	 * @param name The name for this action
	 */
        void setName(const std::string &name) { _name = name; }

	/**
	 * @return The weight to apply to the cost of this action
	 */
	double getWeight() const { return _weight; }

	/**
	 * @param weight The weight to apply to the cost of this action
	 */
	void setWeight(const double &weight) { _weight = weight; }

    protected:
	/**
	 * The name of this action
	 */
        std::string _name;
	
	/**
	 * The weight to apply to the cost of this action
	 */
	double _weight;
    };

    typedef boost::shared_ptr<Action> ActionPtr;


    /**
     * A class to represent the bounds of the search space in world frame
     */
    class EnvironmentExtents {
    public:
	/**
	 * Constructor. All bounds are defaulted to zero
	 */
    EnvironmentExtents() : xmin(0.0), xmax(0.0), ymin(0.0), ymax(0.0), zmin(0.0), zmax(0.0) {}

	/**
	 * Constructor
	 *
	 * @param xmin The minimum x value
	 * @param xmax The maximum x value
	 * @param ymin The minimum y value
	 * @param ymax The maximum y value
 	 * @param zmin The minimum z value
	 * @param zmax The maximum z value
	 */
    EnvironmentExtents(const double &xmin, const double &xmax, const double &ymin, const double &ymax, const double &zmin, const double &zmax) :
        xmin(xmin), xmax(xmax), ymin(ymin), ymax(ymax), zmin(zmin), zmax(zmax) {};

	/**
	 * The minimum x value
	 */
        double xmin;

	/**
	 * The maximum x value
	 */
        double xmax;
	/**
	 * The minimum y value
	 */
        double ymin;

	/**
	 * The maximum y value
	 */
        double ymax;

	/**
	 * The minimum z value
	 */
        double zmin;

	/**
	 * The maximum z value
	 */
        double zmax;

    };

    /**
     * A waypoint that pairs a start point and an action
     */
    class PlannedWaypoint {
    public:
	/**
	 * Constructor
	 *
	 * @param coord The coordinate of this waypoint
	 * @param action The action to apply to get to this waypoint
	 */
    PlannedWaypoint(const WorldCoordinate &coord, const ActionPtr& action) : coord(coord), action(action) {}

	/**
	 * The coordinate for this waypoint
	 */
	WorldCoordinate coord;

	/**
	 * The action to apply to get to this waypoint
	 */
	ActionPtr action;
    };

    typedef boost::shared_ptr<PlannedWaypoint> PlannedWaypointPtr;

    typedef std::map<unsigned int, std::vector<ActionPtr> > ActionList;

    
}
	void multiply(double a[3][3], double b[3][3], double c[3][3]);
	
#endif
