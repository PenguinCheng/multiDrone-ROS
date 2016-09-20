import unittest
from spiripy import api, spiri_except
import os, subprocess
# Load specific service and action server definitions
from spiri_go.msg import *
from spiri_go.srv import *

# The setup and teardown of these is very slow (almost a minute worst case),
# therefore it is recomended to use large monolithic testcases to reduce
# overhead. Generally though, do not test something here if it can be
# tested just as well without the simulator. Note that mavros is on for
# all test suits. This is the only one that uses the sitl sim though.
class TestSpiriApiWithSim(unittest.TestCase):
    def setUp(self):
        # launch the simulator
        home = os.path.expanduser("~")
        ardu_path = os.path.join(home, "ardupilot", "ArduCopter")
        #os.chdir(ardu_path)
        self.sim = subprocess.Popen(
            "sim_vehicle.sh --daemon",
            cwd=ardu_path,
            preexec_fn=os.setsid,
            shell=True,
            #stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE
        )
        # launch ROS
        self.ros = subprocess.Popen(
            ["roslaunch", "spiri_go", "sitl.launch"],
            preexec_fn=os.setsid,
            #stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE
        )
        self.spiri_go = api.SpiriGo()

    def tearDown(self):
        # kill ros
        os.killpg(os.getpgid(self.ros.pid), 15)
        os.waitpid(self.ros.pid, os.WUNTRACED)
        # kill the simulator
        os.killpg(os.getpgid(self.sim.pid), 15)
        os.waitpid(self.sim.pid, os.WUNTRACED)
        # wait for the processes to end

    def test(self):
        try:
            print 'waiting 20 seconds for ROS'
            self.spiri_go.wait_for_ros(20)
        except:
            # print an explanation, then reraise the error
            print "Can't connect to ROS. Check your system setup."
            raise

        try:
            print 'waiting 20 seconds for the flight controller'
            self.spiri_go.wait_for_fcu(20)
        except:
            # print an explanation, then reraise the error
            print "Can't connect to simulator. Check your system setup."
            raise

        print 'Test Takeoff'
        to_height = 5
        self.spiri_go.takeoff(to_height)
        # This blocks until takeoff is over
        local = self.spiri_go.get_local_position()
        # ensure that you are close enough to the takeoff goal
        self.assertGreater(local.z, to_height*0.96)
        self.assertLess(local.z, to_height*1.04)

        # Test Landing
        self.spiri_go.land()
        # block while waiting to land
        local = self.spiri_go.get_local_position()
        # a assume it will be at least this low after it's landed
        self.assertLess(local.z, 0.1)


if __name__ == "__main__":
    unittest.main()
