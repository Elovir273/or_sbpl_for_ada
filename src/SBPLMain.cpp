#include <openrave/plugin.h>
#include <or_sbpl_for_ada/SBPLBasePlanner7d.h>

using namespace OpenRAVE;

InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
    if (type == PT_Planner && interfacename == "sbpl")
    {
        return InterfaceBasePtr(new or_sbpl_for_ada::SBPLBasePlanner(penv));
    }
    return InterfaceBasePtr();
}

void GetPluginAttributesValidated(PLUGININFO& info)
{
    info.interfacenames[PT_Planner].push_back("sbpl");
}

RAVE_PLUGIN_API void DestroyPlugin()
{
    RAVELOG_INFO("destroying plugin\n");
}
