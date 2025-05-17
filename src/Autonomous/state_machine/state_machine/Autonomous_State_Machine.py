import rclpy
from rclpy.node import Node
from multiprocessing import get_context
import keyboard # type: ignore
from random import sample
from pyparsing import removeQuotes
from statemachine import StateMachine, State
import statemachine
from statemachine.contrib.diagram import DotGraphMachine
from typing import Dict, List, Optional, Tuple
import time
from decimal import Decimal, getcontext
import os, json
from rclpy.executors import MultiThreadedExecutor

import statemachine.exceptions
from rclpy.action import ActionClient
from custom_msgs_srvs.action import Navigation
from custom_msgs_srvs.srv import LED


# All print statements are only for debugging the code

class AutonomousStateMachine(StateMachine):
    # Define states
    Start = State(initial=True)
    Navigation = State()
    Check_Point = State()
    SearchTag = State()
    SearchObject = State()
    DriveToTag = State()
    BlinkLights = State()
    UserInput = State()
    BackonPath = State()
    danceOff = State()
    Stop = State(final=True)

    # Define events for transitions
    startNav = Start.to(Navigation) # Load the first coordinate and path
    navigate = Navigation.to(Navigation, cond="!reachedTargetPoint")
    reachTarget = Navigation.to(Check_Point, cond = "reachedTargetPoint") # Need to find a way to keep track of goal points as something else, and not just part of path
    runUnstuck = Navigation.to(BackonPath, cond="roverStuck")
    backToNav = BackonPath.to(Navigation, cond = "backtrack")
    tryUnstuck = BackonPath.to(danceOff, cond = "!backtrack")
    keepBanging = danceOff.to(danceOff, cond="haveTime and isStuck")
    unstuck = danceOff.to(Navigation, cond="haveTime and !isStuck")
    moveOn = danceOff.to(Navigation, cond="!haveTime and isStuck") # Add a instance var here for checking in Navigation
    GNSS = Check_Point.to(BlinkLights)
    lookForTag = Check_Point.to(SearchTag, cond = "checkAruco")
    failTag = SearchTag.to(SearchTag)
    lookForObj = Check_Point.to(SearchObject, cond = "checkObj")
    failObj = SearchObject.to(SearchObject)
    retryOp = SearchTag.to(Navigation)
    retryOp2 = SearchObject.to(Navigation)
    DriveToAruco = SearchTag.to(DriveToTag, cond = "isAruco")
    DriveToObj = SearchObject.to(DriveToTag, cond="isObj")
    success = DriveToTag.to(BlinkLights, cond="successCondition")
    failure = DriveToTag.to(Navigation, cond="!successCondition")
    keepGoing = BlinkLights.to(UserInput)
    continueMission = UserInput.to(Navigation)
    abortMission = UserInput.to(Stop)
    """
    This is a class representing an autonomous state machine for the rover. 
    It consists of 10 states, 19 transitions linked with n(replace later) events.
    """

    # Initalize the state machine
    def __init__(self, model):
        super().__init__(model=model)
        
        self.calls = []
        # self.count_Fails_Tag_Search = 0
        # self.count_Fails_Obj_Search = 0
        self.failTagTime = 0
        self.failTagTotal = 0
        self.failObjTime = 0
        self.failObjTotal = 0
        self.currPoint = None
        self.heading = None
        self.flagStuck = False
        self.lastTraversedPoint = ""
        self.tagFound = False
        self.objFound = False
        self.nextCommand = True
        self.GPSPrecision = 7
        self.pathBacktrack = []
        self._success_search = False

        # self.currTime = time.localtime
        self.timeElapsed = 0

        # need to parametrize once we have the parameters
        self.pointDict = {"GNSS" : (38.44079858,-110.7782071,1377.88), "Aruco" : (53.1231312, 12.592134123, 1123.94), "object" : (123.123123, 53.24123, 400.41)} #pointDictInput, currently values only put for testing
        self.path = []#pathInput
        

    # Define the State actions
    @Start.enter
    def loadPoint(self):
        self.led_cli = self.model.create_client(LED, 'change_LED')
        self.led_req = LED.Request()
        
        self.model.get_logger().info("Updated Picture")
        imgPath = "/home/wiscrobo/workspace/WRoverSoftware/src/Autonomous/state_machine/state_machine/autonomous_state_machine.png"
        graph = DotGraphMachine(AutonomousStateMachine)
        dot = graph()
        dot.write_png(imgPath)

        # Read target points 
        file_path = os.path.abspath("src/Autonomous/state_machine/state_machine/points.json")
        with open(file_path) as f:
            js = json.load(f)
        
        self.targets = js["targets"]

        self.target_indx = 0
        self.target_gps = self.targets[self.target_indx][:2]
        self.target_type = self.targets[self.target_indx][2]
        print("TargetGPS:" + str(self.target_gps))
        print("TargetType:" + str(self.target_type))
        self.entered_nav = False
        self._reachedTargetPoint = False
        self.nav_action_client = ActionClient(self.model, Navigation, 'navigate')
        self.blinkLightColor("RED")
        self.startNav()


    # LOGIC FOR ENTERING NAVIGATION
        # Createa a callback that calls navigation
    @Navigation.enter
    def driveTrainPath(self):
        """
        This calls the drive function, ideally the drive function will be integrated with the obstacle avoidance
        """
        print("ENTERED NAVIGATION")
        if(not self.entered_nav):
            self.entered_nav = True
            goal_msg = Navigation.Goal()
            goal_msg.points = self.target_gps

            self.nav_action_client.wait_for_server()

            self._send_goal_future = self.nav_action_client.send_goal_async(goal_msg, feedback_callback=self.navigate_feedback_callback)

            self._send_goal_future.add_done_callback(self.navigate_goal_response_callback)
        # Obstacle avoidance and driving behavior assumed to be handled by callbacks or another thread
        print("driveTrain: Navigation goal sent.")

    def navigate_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.model.get_logger().info('Goal rejected :(')
            return

        self.model.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.navigate_get_result_callback)
    
    def navigate_get_result_callback(self, future):
        result = future.result().result
        if result.reach_target:
            self.model.get_logger().info("Navigation goal succeeded!")
            self._reachedTargetPoint = True
            self.reachTarget()
        else:
            self.model.get_logger().warn("Navigation goal failed.")
            self.reachTarget()

    def navigate_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.model.get_logger().info(
        f"Distance Away: {feedback.distance_away:.2f}\n"
        f"Current GPS: {list(feedback.current_gps)}\n"
        f"Target GPS: {list(feedback.target_gps)}\n"
        )
    

    @Navigation.exit
    def exitNav(self):
        """
        Uses the rover's GPS to get its current point, and heading
        """
        self.target_indx += 1
        self.target_gps = self.targets[self.target_indx][:2]
        self.target_type = self.targets[self.target_indx][2]

    # Checks what kind of point we have reached.
    @Check_Point.enter
    def checkPointType(self):
        """
        Accesses current point
        """
        if self.target_type == 0:
            self.GNSS()
        elif self.target_type == 1:
            self.SearchTag()
        else:
            self.SearchObject()

    @SearchTag.enter
    def startFailTimeAruco(self):
        self.failTagTime = time.time()

    @SearchTag.enter
    def pathFindTag(self, roverCanSeeTag):
        """Tries to search and see if the Aruco Tag is in the rover's FOV, if it is then we must exit. Essentially a scouting alg"""
        self.tagFound = roverCanSeeTag # This parameter will not exist in actuality, tag and object detection sync

    @SearchTag.exit
    def countFailTimeAruco(self):
        timeTakenTag = time.time()-self.failTagTime
        self.failTagTotal += timeTakenTag
        #print(self.failTagTotal)

    @SearchObject.enter
    def startFailTimeObj(self):
        self.failObjTime = time.time()

    @SearchObject.enter
    def pathFindObject(self, roverCanSeeObj):
        """Tries to search and see if the object is visible by turning camera"""
        self.objFound = roverCanSeeObj # This parameter will not exist in actuality.
    
    @SearchObject.exit
    def countFailTimeObj(self):
        timeTakenObj = time.time() - self.failObjTime
        self.failObjTotal += timeTakenObj
        #print(self.failObjTotal)
    
    @DriveToTag.enter
    def driveToTag(self):
        """Will Call the drive algorithm"""
        pass

    @BlinkLights.enter
    def blinkLights(self):
        """Makes the lights green or smtg, depending on reqs"""
        print("Lights are blinking green yay") # replace with actual code to make lights blink
        self.blinkLightColor("GREEN")
        
    
    @UserInput.enter
    def awaitNextCommand(self):
        """Press spacebar, if space key pressed, returns true. If backspace pressed returns false, waits till one or the other key is pressed"""
        print("Press SPACE to continue mission and BACKSPACE to abort") # This is not for debugging, do not remove
        awaitFlag = True
        while awaitFlag:
            key = keyboard.read_key()
            if key == 'space':
                self.nextCommand = True
                awaitFlag = False
            if key == 'backspace':
                self.nextCommand = False
                awaitFlag = False

    @BackonPath.enter
    def tryAndRouteToPath(self, roverGPSReading):
        """This function will try to retrace our steps backwards to get back on the path.
        This means that in our drive function we will essentially create a small path on every entry and exit of Navigation keep appending
        This will be a stack, and then we traverse in reverse order if we get stuck. Will have to be done in ROS2, will do a demo here"""
        for _ in len(self.pathBacktrack):
            self.pathBacktrack.pop()
        
            # Try and drive to this point using our drive function
            # after this attempt, check if we have moved back!
        if(self.currPoint != roverGPSReading):
            self.flagStuck = True
        else:
            self.flagStuck = False

    @danceOff.enter
    def arbitraryMovementsToFreeRover(self):
        """This function will essentially wiggle the rover, now I don't know how to make the rover move, but that's what it should do"""
        pass
    # Define the conditions for transitions. For passing in parameters we use sm.send with event name and param=param as arguments
    @danceOff.enter
    def startTrackTime(self):
        self.timeElapsed = self.timeElapsed + time.localtime

    @danceOff.exit
    def trackTime(self):
        self.timeElapsed = time.localtime - self.timeElapsed # keeps track of seconds
    # Checks if the rover is still moving keeps looping the Navigation state (Gonna take Euclidean from target point and check)


    def roverStuck(self):
        """I feel like this would be better to do as a state machine implementation rather than a condition, 
           as we have a continuous value feed tape for determining if the rover is stuck. So removing this condition"""
        pass

    def checkAruco(self):
        if(self.lastTraversedPoint=="Aruco"):
            return True
        return False

    def checkObj(self):
        if(self.lastTraversedPoint=="object"):
            return True
        return False
    
    def isGNSS(self):
        if(self.lastTraversedPoint=="GNSS"):
            return True
        return False
    
    def isAruco(self):
        if(self.tagFound == True):
            return True
        return False

    def isObj(self):
        if(self.objFound == True):
            return True
        return False
    
    def successCondition(self):
        return self._success_search 
    
    def reachedTargetPoint(self):
        return self._reachedTargetPoint
    
    def backtrack(self, roverGPSReading):
        getcontext().prec = self.GPSPrecision #GPS units of precision for recognizing as a different point, can be changed,
        tupCurrPoint = (Decimal(self.currPoint[0]), Decimal(self.currPoint[1]), Decimal(self.currPoint[2]))
        tupRoverPoint = (Decimal(roverGPSReading[0]), Decimal(roverGPSReading[1]), Decimal(roverGPSReading[2]))

        if(tupCurrPoint != tupRoverPoint):
            return True
        else:
            return False

    def haveTime(self):
        """This function checks if we have enough time left in our Unstuck algorithm"""
        if(self.timeElapsed <= 120):
            return True
        else:
            return False

    def isStuck(self):
        if(self.flagStuck == True):
            return True
        return False
    
    # General helper functions
    def haveTimeSearch(self):
        if (self.failTagTotal or self.failObjTotal) < 100:
            return True
        return False
    
    def blinkLightColor(self,color):
        while not self.led_cli.wait_for_service(timeout_sec=1.0):
            self.model.get_logger().info('service not available, waiting again...')

        if color == "RED":
            self.led_req.red = 255
            self.led_req.green = 0
            self.led_req.blue = 0
        elif color == "GREEN":
            self.led_req.red = 0
            self.led_req.green = 255
            self.led_req.blue = 0
        elif color == "BLUE":
            self.led_req.red = 0
            self.led_req.green = 0
            self.led_req.blue = 255
        else:
            self.led_req.red = 0
            self.led_req.green = 0
            self.led_req.blue = 0
        return self.led_cli.call_async(self.led_req)



def main(args=None):

    # # This is not necessary everytime, was only to draw up the machine
    
    #sm = AutonomousStateMachine()
    pass
if __name__ == "__main__":
    main()