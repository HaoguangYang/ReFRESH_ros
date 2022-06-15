#!/usr/bin/python

import rospy
from .ReFRESH_ros_utils import *
from .ReFRESH_ros import *

class ModuleOntologyEngine(Manager):
    pass

if __name__ == "__main__":
    # main thread for runtime self adaptation autonomy
    core = Launcher("RunTimeInfrastructure")
    # attach module ontology engine with core
    # start monitoring utilities
    # start task behavior tree level with core
    # run core
    # cleanup: stop behavior tree
    # cleanup: stop monitoring
    # cleanup: stop module ontology engine
    # cleanup: stop core thread
    