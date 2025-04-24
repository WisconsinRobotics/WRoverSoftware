from multiprocessing import get_context
from pyparsing import removeQuotes
from statemachine import StateMachine, State
from statemachine.contrib.diagram import DotGraphMachine
from typing import Dict, List, Optional, Tuple
import time
from decimal import Decimal, getcontext


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
        self.failObjTime = 0
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

        self.pointDict = {} #pointDictInput
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
    start = Start.to(Navigation, cond="evaluateInput") # Load the first coordinate and path
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
    failTag = SearchTag.to(SearchTag, cond = "haveTimeSearch")
    lookForObj = Check_Point.to(SearchObject, cond = "checkObj")
    failObj = SearchObject.to(SearchObject, cond = "haveTimeSearch")
    retryOp = SearchTag.to(Navigation, cond="!haveTimeSearch")
    retryOp2 = SearchObject.to(Navigation, cond="!haveTimeSearch")
    DriveToAruco = SearchTag.to(DriveToTag, cond = "isAruco")
    DriveToObj = SearchObject.to(DriveToTag, cond="isObj")
    success = DriveToTag.to(BlinkLights, cond="successCondition")
    failure = DriveToTag.to(Navigation, cond="!successCondition")
    keepGoing = BlinkLights.to(UserInput)
    continueMission = UserInput.to(Navigation, cond="evaluateInput")
    abortMission = UserInput.to(Stop, cond="!evaluateInput")

    # Define the actions

    # Define the State actions

    # Gets the point each time we move back to Navigation state
    @Navigation.enter
    def driveTrainPath(self):
        """
        This calls the drive function, ideally the drive function will be integrated with the obstacle avoidance
        """
        # Call drive here, and associated helper functions like obstacle avoidance simultaneously
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
        for key, value in self.pointDict.items:
            # TODO: Implement estimation functionality for the current point +/- 2 metres
            if value == self.currPoint:
                self.lastTraversedPoint = key
        

    @SearchTag.enter
    def pathFindTag(self, roverCanSeeTag):
        """Tries to search and see if the Aruco Tag is visible by turning camera"""
        if(roverCanSeeTag is True): 
            self.tagFound = True
        else: 
            self.tagFound = False
        

    @SearchTag.enter
    def startFailTimeAruco(self):
        self.failTagTime = self.failTagTime + time.localtime

    @SearchTag.exit
    def countFailTimeAruco(self):
        self.failTagTime = time.localtime-self.failTagTime

    @SearchObject.enter
    def pathFindObject(self, roverCanSeeObj):
        """Tries to search and see if the object is visible by turning camera"""
        if(roverCanSeeObj is True): return True
        return False
        
    @SearchObject.enter
    def startFailTimeObj(self):
        self.failObjTime = self.failObjTime + time.localtime

    @SearchObject.exit
    def countFailTimeObj(self):
        self.failObjTime = time.localtime - self.failObjTime
    
    @DriveToTag.enter
    def driveTrain(self):
        """Will Call the drive algorithm being worked on by Brady, Nick and Ryan"""
        pass
    @BlinkLights.enter
    def blinkLights(self):
        """Makes the lights green or smtg, depending on reqs"""
        pass
    @BlinkLights.exit
    def revertLights(self):
        """Makes the lights go back"""
        pass
    @UserInput.enter
    def awaitNextCommand(self):
        """How are we planning to do this? Should I do command line input here?"""
        pass

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

    def haveTimeSearch(self):
        if (self.failTagTime or self.failObjTime) < 100:
            return True
        return False
    
    def successCondition(self, roverGPSReading):
        self.currPoint = roverGPSReading
        if(self.reachedTargetPoint(self) == "True"):
            return True
        return False
    
    def evaluateInput(self):
        if(self.awaitNextCommand == True):
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

def main():

    sm = AutonomousStateMachine()

    imgPath = "C:\\Users\\Aditya\\RoboticsStateMachine\\autonomous_state_machine.png"
    graph = DotGraphMachine(AutonomousStateMachine)
    dot = graph()
    dot.write_png(imgPath)

if __name__ == "__main__":
    main()