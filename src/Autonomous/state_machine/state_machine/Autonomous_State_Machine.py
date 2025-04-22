from statemachine import StateMachine, State
from statemachine.contrib.diagram import DotGraphMachine
from typing import Dict, List, Optional, Tuple
import time

class AutonomousStateMachine(StateMachine):

    """
    This is a class representing an autonomous state machine for the rover. 
    It consists of 10 states, 19 transitions linked with n(replace later) events.
    """

    # Initalize the state machine
    def __init__(self):
        self.calls = []
        super().__init__()
        count_Fails_Tag_Search = 0
        count_Fails_Obj_Search = 0
        flagStuck = False

        currTime = time.time
        timeElapsed = 0

        pointDict = {}
        path = []


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
    start = Start.to(Navigation) # Load the first coordinate and path
    reachTarget = Navigation.to(Check_Point, cond = "travelling") # Need to find a way to keep track of goal points as something else, and not just part of path
    runUnstuck = Navigation.to(BackonPath, cond = "roverStuck")
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
    retryOp = SearchTag.to(Navigation, cond="!checkAruco")
    retryOp2 = SearchObject.to(Navigation, cond="!checkObj")
    DriveToAruco = SearchTag.to(DriveToTag, cond = "isAruco")
    DriveToObj = SearchObject.to(DriveToTag, cond="isObj")
    success = DriveToTag.to(BlinkLights)
    keepGoing = BlinkLights.to(UserInput)
    continueMission = UserInput.to(Navigation, cond="evaluateInput")
    abortMission = UserInput.to(Stop, cond="!evaluateInput")

    # Define the actions

    # Define the State actions

    # Gets the point each time we move back to Navigation state
    @Navigation.enter
    def getCurrPointandHeading(self):
        pass

    @Navigation.enter
    def followNextPath(self):
        """
        This is the decided for if we abandon trying to reach the point, and move on to the next point
        """
        pass

    # Checks what kind of point we have reached.
    @Check_Point.enter
    def checkPointType(self, currPoint, targetPoints):
        pass

    @SearchTag.enter
    def countFails(self):
        pass

    @SearchObject.enter
    def countAttempts(self):
        pass

    # Define the Transition actions
    
    # This is the code which will need to be called, to make the transition from Start state to Navigation state.
    @start.on
    def loadPath(self):
        """
        Tries to load path into the rover.
        """
        pass

    
    # Define the conditions for transitions

    # Checks if point has been reached (Gonna take Euclidean from target point and check)
    def travelling(self, currPoint, targetPoint):
        pass

    def roverStuck(self, currPoint, timeElapsed):
        pass

    def checkAruco(self, currPoint):
        pass

    def checkObj(self, currPoint):
        pass
    
    def isGNSS(self):
        pass
    
    def isAruco(self, roverCanSeeTag):
        pass

    def isObj(self, roverCanSeeObj):
        pass
    
    def evaluateInput(self):
        pass
    
    def backtrack(self):
        pass

    def haveTime(self):
        pass

    def isStuck(self):
        pass

    def evaluateInput(self):
        """
        This one just checks if user wants us to keep going or stop
        """
        pass

def main():

    sm = AutonomousStateMachine()

    imgPath = "C:\\Users\\Aditya\\RoboticsStateMachine\\autonomous_state_machine.png"
    graph = DotGraphMachine(AutonomousStateMachine)
    dot = graph()
    dot.write_png(imgPath)

if __name__ == "__main__":
    main()