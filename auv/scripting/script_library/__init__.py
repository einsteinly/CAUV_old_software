import avoid_collision
import breach
import circle_buoy
import follow_cam
import follow_pipe
import line_search
import location_search
import sonar_collision_avoider
import spiral
import start
import surface
import track_asv
import track_wall
import waypoint_demo

index = [k for k,v in locals().items() if hasattr(v, "Script")]