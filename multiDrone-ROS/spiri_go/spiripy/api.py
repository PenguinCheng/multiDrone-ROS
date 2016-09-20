import rospy
import roslib
import time
roslib.load_manifest('spiri_go')
import actionlib
# Load specific service and action server definitions
from spiri_go.msg import *  # imports actions too
from spiri_go.srv import *
# Import the exceptions defined here
from spiripy import spiri_except


class SpiriGo(object):
    """A top level python API to command Spiri to perform autonomous actions.

    The idea for this class is to supply methods for users to perform autonomous
    actions via simple python scripts.

    It interfaces with the SpiriGo ROS node via services and actions, not publishing
    and subscribing. This guarantees the promptness of information.

    Attributes:

    """

    def __init__(self):
        rospy.init_node('spiri_go_api', anonymous=True)

    def wait_for_ros(self, timeout=-1):
        """ Waits for the ros interfaces to come online

        Args:
            timeout (number): How long to wait until this should be considered
            a failure. If timeout is less than 0, will never stop trying

        Raises:
            SpiriGoConnectionError: If the connection times out.
        """
        try:
            rospy.wait_for_service('spiri_state', timeout)
        except:
            raise spiri_except.SpiriGoConnectionError("Timed out connecting to ROS")

    def wait_for_fcu(self, timeout=-1):
        """Waits for mavros to connect to the flight controller.

        Args:
            timeout (number): How long to wait until this should be considered
            a failure. If timeout is less than 0, will never stop trying

        Raises:
            SpiriGoConnectionError: If the connection times out.
        """
        start_time = time.time()
        state = self.get_state()

        while not state.connected:
            time.sleep(0.5)
            state = self.get_state()
            duration = time.time() - start_time
            # print "waiting for fuc: " + str(duration) + "/" + str(timeout)
            # check if this should time out
            if timeout > 0 and duration > timeout:
                raise spiri_except.SpiriGoConnectionError("Timed out connecting to the FCU")

    def get_state(self):
        """mavros_msgs/State: returns state of the copter."""
        state_service_client = get_service_client('spiri_state', LastState)
        return state_service_client()

    def get_local_position(self):
        """geometry_msgs/Point Message: returns the 3D position vector in meters."""
        pos_service_client = get_service_client('spiri_local_position', LocalPosition)
        return pos_service_client()

    def takeoff(self, height=4):
        """Commands the copter to takeoff autonomously.

        Note:
            The copter will go to GUIDED mode, arm motors, and takeoff all in once shot.
            This is a ROS action and will be blocking until it completes.

        Args:
            height (int): The takeoff height, default is 4 meters.

        Returns:
            bool: True if successful, False otherwise.
        """

        # Connection to the server
        client = get_action_client('spiri_take_off', TakeoffAction)

        # Make a goal object for the copter to complete
        goal = TakeoffGoal()
        goal.height = height
        print "Sending takoff command"
        client.send_goal(goal)
        client.wait_for_result()

        return True

    def land(self):
        """Command the copter to land in place.

        Note:
            The copter will go to LAND mode.
            The method will block until it completes.

        Args:

        Returns:
            bool: True if successful, False otherwise.
        """
        client = get_action_client('spiri_land_here', LandHereAction)
        goal = LandHereGoal()
        goal.height = 0
        print "Sending land here command"
        client.send_goal(goal)
        client.wait_for_result()

        return True


def get_action_client(name, action):
    """Gets the client and ensures that the server is running

    Args:
        name: a string, the server's name (eg. 'spiri_take_off').
        action: an object, the type of server (eg. TakeoffAction).

    Returns:
        The action client object.
    """
    client = actionlib.SimpleActionClient(name, action)
    server_present = client.wait_for_server(rospy.Duration(1))
    if server_present:
        return client
    else:
        raise spiri_except.SpiriGoConnectionError("Could not create action client " + name)


def get_service_client(name, service):
    """Ensure that the service exists.

    Args:
        service: an object, the name of the service.

    Returns:
        The service client object.
    """
    try:
        rospy.wait_for_service(name, 1)
    except:
        raise spiri_except.SpiriGoConnectionError("Could not create service client " + name)
    return rospy.ServiceProxy(name, service)
