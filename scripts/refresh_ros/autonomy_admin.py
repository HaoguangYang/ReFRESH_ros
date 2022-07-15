#!/usr/bin/python

import rospy
from .ReFRESH_ros_utils import *
from .ReFRESH_ros import *
import rospkg
#import xml.etree.ElementTree as ET
import owlready2 as owl
from refresh_ros.msg import HighLevelRequestAction
from refresh_ros.srv import ModuleEstimate
import actionlib

class ModuleOntologyEngine():
    def __init__(self, launcher):
        # xml parse onto
        # why not use yaml and conform to the rosparam scheme?
        #xmlStruct = ET.parse(config)
        mList = rospy.get_param("~register_module",[])
        self.moduleInstances = []
        for item in mList:
            pkgName = item['package']
            libName = item['module_name']
            try:
                rospkg.RosPack().get_path(pkgName)
            except Exception as e:
                # this returns with exception
                print(e)
                raise("ERROR: Package "+pkgName+" not found")
            try:
                # bad practice... can be cleaned up if using ROS2 + lifecycle
                mod = __import__(libName, fromlist=[pkgName])
                self.moduleInstances.append(mod())
            except Exception as e:
                print(e)
                raise("ERROR: Module "+libName+" not found or ill-formed in package: "+pkgName)
        # ReFRESH manager for all descending modules
        self.moduleManager = Manager(launcher, self.moduleInstances, "ReFRESH_ROS_Module_Manager")
        # start ontology reasoner by loading ontology into owlready2
        onto_file = rospy.get_param("~runtime_ontology")
        if os.path.exists(onto_file):
            self.onto = owl.get_ontology(onto_file).load()
        else:
            raise("ERROR: Runtime ontology "+onto_file+" not found")
        # start action server that listens to high-level action demands, resolve demands with ontology,
        # and feeds back real-time action-level performance and resource evaluation.
        self.highLevelActionServer = actionlib.SimpleActionServer("refresh_ros/high_level_action",
            HighLevelRequestAction, exec_cb = self.resolveAndRunHighLevelAction, auto_start=False)
        # start service that listens to Estimation requests
        self.highLevelEstimationServer = rospy.Service("refresh_ros/high_level_action_estimate",
            ModuleEstimate, self.synthHighLevelES)

    def resolveAndRunHighLevelAction(self, goal):
        # TODO: runtime evaluation is provided as the feedback.
        pass

    def synthHighLevelES(self):
        pass

    def synthHighLevelEV(self):
        pass

    def shutdown(self):
        self.moduleManager.shutdown()

    def run(self, blocking=True):
        self.moduleManager.run(blocking = blocking)

if __name__ == "__main__":
    # main thread for runtime self adaptation autonomy
    core = Launcher("RunTimeInfrastructure")
    # attach module ontology engine with core
    moduleOntoMan = ModuleOntologyEngine(core)#, config=rospy.get_param("~module_description"))
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
