#include <or_sbpl_for_ada/SBPLBasePlannerTypes7d.h>
#include <or_sbpl_for_ada/YamlUtils2.h>
#include <or_sbpl_for_ada/SBPLBasePlanner7d.h>
#include <or_sbpl_for_ada/Action7d.h>
#include <fstream>
#include "yaml-cpp/yaml.h"
#include <boost/unordered_map.hpp>
#include <boost/foreach.hpp>

int main(int argc, char** argv){

#ifdef YAMLCPP_NEWAPI
    YAML::Node doc = YAML::LoadFile("actions.yaml");
#else
    std::ifstream in_file("actions.yaml");
    YAML::Parser parser(in_file);
    
    YAML::Node doc;
    parser.GetNextDocument(doc);
#endif
    
    std::cout << " step 1 done " << std::endl;

    double cellsize = 0.0;
#ifdef YAMLCPP_NEWAPI
    std::cout << " start cell " << std::endl;
    cellsize = doc["cellsize"].as<double>();
    std::cout << " end cell" << std::endl;
#else
    doc["cellsize"] >> cellsize;
#endif
    std::cout << "Cellsize: " << cellsize << std::endl;

    int numangles = 0;
#ifdef YAMLCPP_NEWAPI
    numangles = doc["numangles"].as<int>();
#else
    doc["numangles"] >> numangles;
#endif
    std::cout << "Num angles: " << numangles << std::endl;

    int nummodes = 0;
#ifdef YAMLCPP_NEWAPI
    nummodes = doc["nummodes"].as<int>();
#else
    doc["nummodes"] >> nummodes;
#endif
    std::cout << "Num mode: " << nummodes << std::endl;
    

        double angle_weight = 0.0;
#ifdef YAMLCPP_NEWAPI
    angle_weight = doc["angle_weight"].as<double>();
#else
    doc["angle_weight"] >> angle_weight;
#endif
    std::cout << "angle weight: " << angle_weight << std::endl;

    double linear_weight = 0.0;
#ifdef YAMLCPP_NEWAPI
    linear_weight = doc["linear_weight"].as<double>();
#else
    doc["linear_weight"] >> linear_weight;
#endif
    std::cout << "Linear weight: " << linear_weight << std::endl;

    double mode_weight = 0.0;
#ifdef YAMLCPP_NEWAPI
    mode_weight = doc["mode_weight"].as<double>();
#else
    doc["mode_weight"] >> mode_weight;
#endif
    std::cout << "mode weight: " << mode_weight << std::endl;


    or_sbpl_for_ada::ActionList actions;
    doc["actions"] >> actions;
    std::cout << "Num actions: " << actions.size() << std::endl;

    or_sbpl_for_ada::EnvironmentExtents extents;
    doc["extents"] >> extents;
     std::cout << "extents: xmin : " << extents.xmin <<
                           " xmax : " << extents.xmax <<
                           " ymin : " << extents.ymin <<
                           " ymax : " << extents.ymax <<
                           " zmin : " << extents.zmin <<
                           " zmax : " << extents.zmax << std::endl;

    BOOST_FOREACH(or_sbpl_for_ada::ActionList::value_type &alist, actions){
    BOOST_FOREACH(or_sbpl_for_ada::ActionPtr a, alist.second){
        or_sbpl_for_ada::Action7dPtr ca = boost::dynamic_pointer_cast<or_sbpl_for_ada::Action7d>(a);
        std::cout << "Added action with weight: " << ca->getWeight() << std::endl;
        std::cout << "Poses: " << std::endl;
        std::vector<or_sbpl_for_ada::WorldCoordinate> pts = ca->getPoints();
        for(unsigned int i=0; i < pts.size(); i++){
        or_sbpl_for_ada::WorldCoordinate wc = pts[i];
        std::cout << "\t" << wc << std::endl;
        }
    }
    }
}
