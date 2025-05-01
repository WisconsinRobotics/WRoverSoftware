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

import statemachine.exceptions

# All print statements are only for debugging the code

class AutonomousStateMachine(StateMachine):

    """
    This is a class representing an autonomous state machine for the rover. 
    It consists of 10 states, 19 transitions linked with n(replace later) events.
    """

    # Initalize the state machine
    def __init__(self):
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

        # self.currTime = time.localtime
        self.timeElapsed = 0

        # need to parametrize once we have the parameters
        self.pointDict = {"GNSS" : (38.44079858,-110.7782071,1377.88), "Aruco" : (53.1231312, 12.592134123, 1123.94), "object" : (123.123123, 53.24123, 400.41)} #pointDictInput, currently values only put for testing
        self.path = []#pathInput
        super().__init__()


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
    GNSS = Check_Point.to(BlinkLights, cond="isGNSS")
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

    # Define the actions

    # Define the State actions

    # Drives to points, think the loop will be done inside the state, so no 
    @Navigation.enter
    def driveTrainPath(self):
        """
        This calls the drive function, ideally the drive function will be integrated with the obstacle avoidance
        """
        # Call drive here, and associated helper functions like obstacle avoidance simultaneously. This will move the rover VVIMP
        pass
    
    @Navigation.enter
    def followNextPath(self):
        """
        This is the decided for if we abandon trying to reach the point, and move on to the next point
        """
        if self.flagStuck == True:
            # Here, we give up on going to this target point, and move to the next
            pass #Implement this
        #Otherwise we do nothing
        pass

    @Navigation.exit
    def getCurrPointandHeading(self, roverGPSReading, roverHeading):
        """
        Uses the rover's GPS to get its current point, and heading
        """

        self.currPoint = roverGPSReading # Replace with the GPS reading coming from the Rover
        self.pathBacktrack.append(roverGPSReading)
        self.heading = roverHeading # Replace with direction angle of rover(don't know what will be) and don't know if needed


    # Checks what kind of point we have reached.
    @Check_Point.enter
    def checkPointType(self):
        """

        Accesses current point
        """
        for key, value in self.pointDict.items():
            # TODO: Implement estimation functionality for comparing equality for the current point +/- 2 metres
            if value == self.currPoint:
                self.lastTraversedPoint = key
                break


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
        print(self.failTagTotal)

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
        print(self.failObjTotal)
    
    @DriveToTag.enter
    def driveToTag(self):
        """Will Call the drive algorithm"""
        pass

    @BlinkLights.enter
    def blinkLights(self):
        """Makes the lights green or smtg, depending on reqs"""
        print("Lights are blinking green yay") # replace with actual code to make lights blink
    @BlinkLights.exit
    def revertLights(self):
        """Makes the lights go back"""
        print("Lights are not blinking, sad") # replace with actual code to make lights stop blinking
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

    def reachedTargetPoint(self):
        # TODO: Implement estimation functionality +/- 2 metres of target point 
        print(self.currPoint)
        if(self.currPoint in self.pointDict.values()):
            return True
        return False

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
    
    def successCondition(self, roverGPSReading):
        self.currPoint = roverGPSReading
        if(self.reachedTargetPoint(self) == "True"):
            return True
        return False
    
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



def main():

    sm = AutonomousStateMachine()

    #These are input values from the rover GPS               VVIMP READ BELOW DO NOT MISS
    testsampleRoverGPS1 = (38.44079858,-110.7782071,1377.88) # This needs to be a continuous input stream for the code below to work.
    testsampleRoverGPS2 = (53.1231312, 12.592134123, 1123.94)
    testsampleRoverGPS3 = (123.123123, 53.24123, 400.41)
    testsampleRoverGPS4 = (456.123, 1234.523, 23.256)
    # Please note that we will only have 1 roverGPS reading which will get updated every single time
    sampleRoverHeading = 15

    # This is not necessary everytime, was only to draw up the machine
    imgPath = "C:\\Users\\Aditya\\RoboticsStateMachine\\autonomous_state_machine.png"
    graph = DotGraphMachine(AutonomousStateMachine)
    dot = graph()
    dot.write_png(imgPath)

    sm.send("startNav")

    navFlag = True

    while navFlag:
        try:
            sm.send("reachTarget", roverGPSReading=testsampleRoverGPS2, roverHeading = sampleRoverHeading)

            if(sm.lastTraversedPoint == "GNSS"):

                sm.send("GNSS") # Will enter blink lights here
                sm.send("keepGoing")
                if(sm.nextCommand == False):
                    sm.send("abortMission")
                    navFlag = False
                else:
                    sm.send("continueMission")
                    # Must load in next point, and get rid of this point in traversal -- think it will be done in drive?
                    """My plan for the whole navigating thing is also, to go to the closest point, then remove that point from dictionary.
                     We will sort that before entering here, and then go to next closest point which is in dict, remove and so on
                     on continue mission, we will do the point removal from the dictionary. Access the key of last traversed, 
                     access the point in that list of tuples, and remove from list, should not be too hard to implement."""

            elif(sm.lastTraversedPoint == "Aruco"):
                sm.send("lookForTag", roverCanSeeTag = True)
                if (sm.haveTimeSearch == True):
                    sm.send("failTag")
                else:
                    (sm.send("retryOp"))



            elif(sm.lastTraversedPoint == "Object"):
                sm.send("lookforObj")



            navFlag = False # See what to do here
        except statemachine.exceptions.TransitionNotAllowed:
            sm.send("navigate", roverGPSReading=testsampleRoverGPS2, roverHeading = sampleRoverHeading)

    print(sm.current_state)
if __name__ == "__main__":
    main()