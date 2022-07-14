#!/usr/bin/python

import rospy
from .ReFRESH_ros_utils import *
from .ReFRESH_ros import *
import rospkg
import owlready2 as owl

class ModuleOntologyEngine():
    def __init__(self, launcher, onto):
        # xml parse onto
        # for items in spawned_modules_in_onto:
        #   if rospkg.find(item.package):
        #       try:
        #           # bad practice... can be cleaned up if using ROS2 + lifecycle
        #           from item.package import module_script
        #           continue
        #       except script_not_found:
        #           throw("ill-formed xml: module not found")
        #   throw("ill-formed xml: package not found")
        # self.moduleManager = Manager(launcher, ...)
        # start ontology reasoner by loading ontology into owlready2
        # start service that listens to high-level action demands
        # start service that listens to Estimation requests
        # start publisher that publishes overall module-level performance Evaluation
        pass

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
    #behaviorThread = core.launch(Ftype.NODE, ...)
    # run core
    moduleOntoMan.run(blocking = True)
    # cleanup: stop behavior tree
    #core.stop(behaviorThread)
    # cleanup: stop monitoring
    #core.stop(telemetryThread)
    # cleanup: stop module ontology engine
    moduleOntoMan.shutdown()
    # cleanup: stop core thread
    core.shutdown()
