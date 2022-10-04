#!/usr/bin/python

import rospy
from refresh_ros.ReFRESH_ros_utils import *
from refresh_ros.ReFRESH_ros import *
import rospkg
import owlready2 as owl
from refresh_ros.msg import HighLevelRequestAction, HighLevelRequestGoal, ModuleEvaluate
from refresh_ros.srv import ModuleEstimate, ModuleEstimateResponse
import actionlib

class ModuleOntologyEngine(Manager):
    # TODO: need to conform to inheritance format
    def __init__(self, launcher:Launcher, managedModules:dict=[], name:str="ReFRESH_ROS_Module_Manager",
                    freq:float=5.0, minReconfigInterval:float= 1.0):
        # use yaml and conform to the rosparam scheme
        mList = rospy.get_param("~register_module",[])
        """
        YAML format rosparam:
        register_module:
          - package: xxx
            module_name: yyy
          - package: zzz
            module_name: www
          ...
        """
        self.moduleInstances = []
        for item in mList:
            pkgName = item['package']
            libName = item['module_name']
            try:
                rospkg.RosPack().get_path(pkgName)
            except Exception as e:
                # this returns with exception
                print(e)
                raise Exception("ERROR: Package " + pkgName + " not found")
            try:
                # bad practice... can be cleaned up if using ROS2 + lifecycle
                mod = __import__(libName, fromlist = [pkgName])
                self.moduleInstances.append(mod())
            except Exception as e:
                print(e)
                raise Exception("ERROR: Module " + libName + " not found or ill-formed in package: " + pkgName)
        # ReFRESH manager for all descending modules
        super().__init__(launcher, self.moduleInstances + managedModules, name, freq, minReconfigInterval)

        # start ontology reasoner by loading ontology into owlready2
        onto_file = rospy.get_param("~runtime_ontology")
        if os.path.exists(onto_file):
            self.onto = owl.get_ontology(onto_file).load()
        else:
            raise Exception("ERROR: Runtime ontology " + onto_file + " not found")
        # start action server that listens to high-level action demands, resolve demands with ontology,
        # and feeds back real-time action-level performance and resource evaluation.
        self.highLevelActionServer = actionlib.SimpleActionServer("refresh_ros/high_level_action",
            HighLevelRequestAction, execute_cb = self.resolveAndRunHighLevelAction, auto_start = False)
        # start service that listens to Estimation requests
        self.highLevelEstimationServer = rospy.Service("refresh_ros/high_level_action_estimate",
            ModuleEstimate, self.synthHighLevelES)
        
        # basicDecider is overriden as ontologyDecider
        self.Decider = ModuleComponent(Ftype.TIMER, (freq, self.ontologyDecider), {})

    """
    Override the original preemption logic here since an ontology will be serving the inter-module relation
    """
    def moduleIsPreemptible(self, module:ReFrESH_Module):
        return 0, tuple()

    def moduleCanPreempt(self, module:ReFrESH_Module):
        return 0, tuple()

    def ontologyDecider(self, event):
        # TODO: determine module-level faults
        # TODO: back-trace function
        # TODO: ontologyResolveFunction(...)
        pass

    def resolveAndRunHighLevelAction(self, goal : HighLevelRequestGoal):
        # requestedBehavior = goal.action_request
        # requestedBehaviorArgs = goal.arguments
        # TODO: consult with the ontology on the component synthesis of requestedBehavior(requestedBehaviorArgs)
        """
        requestedBehavior is an instance of class action_request and parameter property arguments,
        arguments encapsulated in a YAML structure (?) to provide multi-field properties.
        a requestedBehavior dedicates a motor group for execution.
        a motor group is dedicated to at most 1 requestedBehavior.
        a synchronizer communicates between at least 2 motor groups.
        a requestedBehavior has exactly 1 goal.
        a requestedBehavior has at least 1 constraint.
        a goal requires at least 1 function.
        a function is composed by at least 1 (other (sub-)functions or modules).
        a constraint is disassembled into subconstraints based on function composition.
        subconstraints belongs to subfunctions.
        """
        # composition = self.ontologyResolveAction(requestedBehavior, requestedBehaviorArgs)
        # self.requestOn(composition-onDict)
        # self.activeConfig = composition
        # while loop
        #   TODO: runtime evaluation is provided as the feedback.
        #   ... = self.synthHighLevelCost(self.activeConfig)
        pass

    def synthHighLevelES(self, req : ModuleEstimate._request_class):
        # requestedBehavior = req.action_request
        # requestedBehaviorArgs = req.arguments
        # TODO: consult with the ontology on the estimated performance of requestedBehavior(requestedBehaviorArgs)
        # composition = self.ontologyResolveAction(requestedBehavior, requestedBehaviorArgs)
        # ... = self.synthHighLevelCost(composition)
        return ModuleEstimateResponse(estimate = ModuleEvaluate(
                performanceCost = 0.0,
                resourceCost = 0.0),
                explanation = "")

    def ontologyResolveAction(self, action, args):
        # for function in hiearachy:
        #   module = ontologyResolveFunction(function.function, function.args)
        pass

    def ontologyResolveFunction(self, function, args):
        pass

    def ontologySynthHighLevelCost(self, config):
        # TODO: collect cost of interconnected modules
        # TODO: ontoSynthActionCost(self.config)
        # N.B. config contais structure of the module-level composition,
        # therefore synthesis will directly reuse that info.
        # performance cost: 0 -- success; 0..1 -- active execution; >=1 -- failure
        # resource cost: 0..1 -- feasible; >=1 -- infeasible
        # return (...)
        pass

    # def shutdown(self):
    #     self.moduleManager.shutdown()

    # def run(self, blocking=True):
    #     self.moduleManager.run(blocking = blocking)

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
                                args="_mission_file:=" + rospy.get_param("~initial_mission","''"))
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
