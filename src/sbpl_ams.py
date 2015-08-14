#!/usr/bin/env python

# Copyright (c) 2013, Carnegie Mellon University
# All rights reserved.
# Authors: Evan Shapiro <eashapir@andrew.cmu.edu>
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
# - Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
# - Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# - Neither the name of Carnegie Mellon University nor the names of its
#   contributors may be used to endorse or promote products derived from this
#   software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# file should ultimately be in prpy/planning

from base import BasePlanner, PlanningError, PlanningMethod, UnsupportedPlanningError
import openravepy
import yaml
import time


class SBPLPlanner(BasePlanner):
    def __init__(self):
        super(SBPLPlanner, self).__init__()
        
        try:
            #need SBPLMG if package renamed
            self.planner = openravepy.RaveCreatePlanner(self.env, 'SBPL')
        except openravepy.openrave_exception:
            raise UnsupportedPlanningError('Unable to create SBPL module')

    def setupEnv(self, env):
        self.env = env
        try:
            self.problem = openravepy.RaveCreateProblem(self.env, 'SBPL')
        except openravepy.openrave_exception:
            raise UnsupportedPlanningError('Unable to create SBPL module.')

    def __str__(self):
        return 'SBPL'

    def SetPlannerParameters(self, params_yaml):
        self.planner_params = params_yaml

    @PlanningMethod
    def PlanToBasePose(self, robot, goal_pose, timelimit=60.0, return_first=False, **kw_args):
        """
        Plan to a base pose using SBPL
        @param robot
        @param goal_pose desired base pose
        @param timelimit timeout in seconds
        @param return_first return the first path found (if true, the planner will run until a path is found, ignoring the time limit)
        """
     
        params = openravepy.Planner.PlannerParameters()

        from openravepy import DOFAffine
        robot.SetActiveDOFs([], DOFAffine.Transform )
        params.SetRobotActiveJoints(robot)

        config_spec = openravepy.RaveGetAffineConfigurationSpecification(DOFAffine.Transform, robot)
        #params.SetConfigurationSpecification(self.env, config_spec) # This breaks

        params.SetGoalConfig(goal_pose) 

        # Setup default extra parameters
        extra_params = self.planner_params

        extra_params["n_axes"] = kw_args.get('_n_axes')

        if return_first:
            extra_params["return_first"] = 1
        else:
            extra_params["return_first"] = 0

        extra_params["initial_eps"] = 1.0

        for key, value in kw_args.iteritems():
            extra_params[key] = value

        params.SetExtraParameters(str(extra_params))
        traj = openravepy.RaveCreateTrajectory(self.env, '')

        try:
            self.planner.InitPlan(robot, params)
            status = self.planner.PlanPath(traj,releasegil=True)

        except Exception as e:
            raise PlanningError('Planning failed with error: {0:s}'.format(e))

        from openravepy import PlannerStatus
        if status not in [ PlannerStatus.HasSolution, PlannerStatus.InterruptedWithSolution ]:
            raise PlanningError('Planner returned with status {0:s}.'.format(str(status)))
        return traj
        

    def GetLastPlanPathsCosts(self):
        """
        Return the path cost
        """
        yaml_paths_costs = self.planner.SendCommand("GetPathsCosts")
        paths_costs = yaml.load(yaml_paths_costs)
        return paths_costs

    def GetLastPlanCartesianPath(self):
        """
        Return a dictionary of key/value pairs containing the cartesian state at each step
        """
        yaml_cart_path = self.planner.SendCommand("GetCartPath")
        cart_path = yaml.load(yaml_cart_path)
        return cart_path    

    def GetLastPlanListActions(self):
        """
        Return a dictionary of key/value pairs containing the action made at each step
        """
        yaml_list_action = self.planner.SendCommand("GetListActions")
        list_action = yaml.load(yaml_list_action)
        return list_action