import settings

from math import tan, radians

class DownwardCamera():
    @staticmethod
    def getObjectDepthWidth(auvdepth1, width1, auvdepth2, width2):
        """
        Estimate the depth and width of an object based on two sightings.
        Note that in use it is probably better to take averages at 2 depths and pass the averages
        """
        image_width = 2*tan(radians(settings.DOWNWARD_CAMERA_FOV)/2)
        depth = (auvdepth1*width1-auvdepth2*width2)/(width1-width2)
        width = (auvdepth1-auvdepth2)*width1*width2*image_width/(width1-width2)
        return depth, width
