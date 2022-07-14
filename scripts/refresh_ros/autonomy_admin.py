#!/usr/bin/python

import rospy
from .ReFRESH_ros_utils import *
from .ReFRESH_ros import *
import rospkg
import xml.etree.ElementTree as ET
import owlready2 as owl

class ModuleOntologyEngine():
    def __init__(self, launcher, onto):
        # xml parse onto
        xmlStruct = ET.parse(onto)
        self.moduleInstances = []
        for item in xmlStruct.findall('register_module'):
            pkgName = item.get('package')
            libName = item.get('module_name')
            try:
                rospkg.RosPack().get_path(pkgName)
            except Exception as e:
                # this returns with exception
                print(e)
                raise("ERROR: Package not found, invoked in file: "+onto)
            try:
                # bad practice... can be cleaned up if using ROS2 + lifecycle
                mod = __import__(libName, fromlist=[pkgName])
                self.moduleInstances.append(mod())
            except Exception as e:
                print(e)
                raise("ERROR: Module "+libName+" not found or ill-formed in package: "+pkgName)
        self.moduleManager = Manager(launcher, self.moduleInstances, "ReFRESH_ROS_Module_Manager")
        # TODO:
        # start ontology reasoner by loading ontology into owlready2
        # start service that listens to high-level action demands
        # start service that listens to Estimation requests
        # start publisher that publishes overall module-level performance Evaluation

    def resolveHighLevelDemands(self):
        pass

    def synthHighLevelES(self):
        pass

    def synthHighLevelEV(self):
        pass

    def shutdown(self):
        pass

    def run(self, blocking=True):
        pass

if __name__ == "__main__":
    # main thread for runtime self adaptation autonomy
    core = Launcher("RunTimeInfrastructure")
    # attach module ontology engine with core
    moduleOntoMan = ModuleOntologyEngine(core, onto=rospy.get_param("~module_description"))
    # start monitoring utilities
    #telemetryThread = core.launch(...)
    # start task behavior tree level with core
    behaviorThread = core.launch(Ftype.NODE, "refresh_ros",
                                "ReFRESH_ROS_TaskBT_Engine", "ReFRESH_ROS_TaskBT_Engine",
                                "_mission_file:="+rospy.get_param("~initial_mission",""))
    # run core
    moduleOntoMan.run(blocking = True)
    # cleanup: stop behavior tree
    core.stop(behaviorThread)
    # cleanup: stop monitoring
    #core.stop(telemetryThread)
    # cleanup: stop module ontology engine
    moduleOntoMan.shutdown()
    # cleanup: stop core thread
    core.shutdown()
