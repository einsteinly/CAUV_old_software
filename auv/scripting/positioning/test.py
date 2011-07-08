from AI_location import aiPositionProvider

import time

class positionProvider(aiPositionProvider):
    
    def __init__(self, opts, args):
        # do something with options if you want
        error("This is just a test positioner")
        
    def timeout():
        # gracefully handle timeout
        pass

    def fixPosition():
        # the main function for generating a position fix
        time.sleep(10)

    def isFinished():
        # return True when we're done, False otherwise
        return True
        
    def getPosition():
        # return the current (or latest) position
        return (1, 2)    
