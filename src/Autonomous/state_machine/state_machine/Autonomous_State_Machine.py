import rclpy
from rclpy.node import Node
from multiprocessing import get_context
import keyboard 
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
from custom_msgs_srvs.action import ObjectDetection
from custom_msgs_srvs.srv import LED
import tkinter as tk
from std_msgs.msg import String


# All print statements are only for debugging the code

class AutonomousStateMachine(StateMachine):
    # Define states
    Start = State(initial=True)
    Navigation = State()
    Check_Point = State()
    Search = State()
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
    lookForTag = Check_Point.to(Search, cond = "checkAruco")
    failTag = Search.to(Search)
    retryOp = Search.to(Navigation)
    DriveToAruco = Search.to(DriveToTag, cond = "isAruco")
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
        self.command_timeout = 15
        self.model.declare_parameter('led_mode', "mock")
        self.led_mode = self.model.get_parameter('led_mode').get_parameter_value().string_value
        self.model.get_logger().info('LED Mode: ' + str(self.led_mode))
        self.led_cli = self.model.create_client(LED, 'change_LED')
        self.led_req = LED.Request()
        
        self.model.get_logger().info("Updated Picture")
        imgPath = "/home/balabalu/WRoverSoftware/src/Autonomous/state_machine/state_machine/autonomous_state_machine.png"
        graph = DotGraphMachine(AutonomousStateMachine)
        dot = graph()
        dot.write_png(imgPath)

        #Wait for user input 
        self.command_received = False
        self.subscription = self.model.create_subscription(
            String,
            '/user_command',
            self.handle_user_command,
            10
        )

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
        self.object_detection_action_client = ActionClient(self.model, ObjectDetection, 'object_detection')
        self.blinkLightColor("RED")
        self.startNav()


    # LOGIC FOR ENTERING NAVIGATION
        # Createa a callback that calls navigation
    @Navigation.enter
    def driveTrainPath(self):
        """
        This calls the drive function, ideally the drive function will be integrated with the obstacle avoidance
        """
        self.model.get_logger().info("ENTERED NAVIGATION")
        
        goal_msg = Navigation.Goal()
        goal_msg.points = self.target_gps

        self._send_goal_future = self.nav_action_client.send_goal_async(goal_msg, feedback_callback=self.navigate_feedback_callback)

        self._send_goal_future.add_done_callback(self.navigate_goal_response_callback)
        # Obstacle avoidance and driving behavior assumed to be handled by callbacks or another thread
        self.model.get_logger().info("driveTrain: Navigation goal sent.")

    def navigate_goal_response_callback(self, future):
        self.model.get_logger().info("response from action client")
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
            self._reachedTargetPoint = False
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
        else:
            self.Search()

    @Search.enter
    def pathFindTag(self):
        """
        This calls the drive function, ideally the drive function will be integrated with the obstacle avoidance
        """
        self.model.get_logger().info("ENTERED SEARCH FOR TAG OR OBJECT")
        
        goal_msg = ObjectDetection.Goal()
        goal_msg.type = self.target_type

        self._send_goal_future = self.object_detection_action_client.send_goal_async(goal_msg, feedback_callback=self.obj_det_feedback_callback)

        self._send_goal_future.add_done_callback(self.obj_det_goal_response_callback)
        # Obstacle avoidance and driving behavior assumed to be handled by callbacks or another thread
        self.model.get_logger().info("driveTrain: Navigation goal sent.")

    def obj_det_goal_response_callback(self, future):
        self.model.get_logger().info("response from action client object/detection")
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.model.get_logger().info('Goal rejected :(')
            return

        self.model.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.obj_det_get_result_callback)

    def obj_det_get_result_callback(self, future):
        result = future.result().result
        if result.reached_tag:
            self.model.get_logger().info("Object/tag arrival succeeded!")
            self._reachedTargetPoint = True
            self.DriveToAruco()
        else:
            self.model.get_logger().warn("Object/tag arrival failed!")
            self.Search()

    def obj_det_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.model.get_logger().info(
        f"Have we found object/tag: {feedback.found_tag}\n"
        )
    
    @DriveToTag.enter
    def driveToTag(self):
        """All implemented on previous one (no search algorithm has been implemented)"""
        self.success()

    @BlinkLights.enter
    def blinkLights(self):
        """Makes the lights green or smtg, depending on reqs"""
        print("Lights are blinking green yay") # replace with actual code to make lights blink
        self.blinkLightColor("GREEN")
        self.keepGoing()
        
    
    @UserInput.enter
    def awaitNextCommand(self):
        """
        Waits for user to press SPACE to continue the mission or BACKSPACE to abort.
        Returns True if continuing, False if aborting.
        """
        self.model.get_logger().info("Paste this into another command window to continue: \n ros2 topic pub --once /user_command std_msgs/msg/String \"data: 'continue'\"")
        self.command_start_time = time.time()
        self.timer = self.model.create_timer(1.0, self.check_command)

    def check_command(self):
        if self.command_received:
            self.model.get_logger().info("Command Received: True")
            self.timer.cancel()
            self.continueMission()
        elif time.time() - self.command_start_time > self.command_timeout:
            self.command_received = True
            self.model.get_logger().info("Timeout reached. Automatically continuing mission.")

    @UserInput.exit
    def resetCommandReceived(self):
        self.command_received = False
        
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
        if self.led_mode == "real":
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
        else:
            if color == "RED":
                self.model.get_logger().info('Blinked LED Red')

            elif color == "GREEN":
                self.model.get_logger().info('Blinked LED Green')

            elif color == "BLUE":
                self.model.get_logger().info('Blinked LED Blue')
            else:
                self.model.get_logger().info('Turned off LED')
    
    def handle_user_command(self, msg):
        if msg.data == "continue":
            self.command_received = True
            self.model.get_logger().info("Set command received to true")

def main(args=None):

    # # This is not necessary everytime, was only to draw up the machine
    
    #sm = AutonomousStateMachine()
    pass
if __name__ == "__main__":
    main()