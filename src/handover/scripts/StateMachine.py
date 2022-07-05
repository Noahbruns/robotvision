import rospy

class StateMachine():
    def __init__(self, init):
        self.log("StateMachine started")
        self.state = init
        self.change = True

    def setState(self, state):
        self.state = state
        self.change = True
        self.log("State set to: " + str(self.state))
    
    def isChange(self):
        if self.change:
            self.change = False
            return True
        else:
            return False

    def getState(self):
        return self.state

    def isState(self, state):
        return self.state == state

    def log(self, text):
        rospy.loginfo(text)