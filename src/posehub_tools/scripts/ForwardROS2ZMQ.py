import rospy
import zmq
import json
from geometry_msgs.msg import PoseStamped # Change this to your actual message type
global msg1,msg2,msg3
msg1 = "0,0,0,0,0,0,0,1,0"
msg2 = "0,0,0,0,0,0,0,1,0"
msg3 = "0,0,0,0,0,0,0,1,0"
# Setup ZMQ
ZMQ_ADDRESS = "tcp://*:5601"  # Or "tcp://127.0.0.1:5555" for local only

# === Prepare ZMQ Publisher ===
context = zmq.Context()
publisher = context.socket(zmq.PUB)
publisher.bind(ZMQ_ADDRESS)
publisher.setsockopt(zmq.SNDHWM, 50)
zmq_topics = ["Probe","StaticRef","Anatomy"]

def pose_to_string_list(pose_stamped):
    pos = pose_stamped.pose.position
    ori = pose_stamped.pose.orientation
    stamp = pose_stamped.header.stamp

    valid = "0" if (stamp.secs < 1) else "1"

    return str(pos.x) + "," + str(pos.y)+ "," + str(pos.z)+ "," + str(ori.x)+ "," +str(ori.y)+ "," +str(ori.z)+ "," +str(ori.w) + "," + valid

def ref_callback(msg):
    global msg1
    msg1 = pose_to_string_list(msg)

def panel_callback(msg):
    global msg2
    msg2 = pose_to_string_list(msg)
    
def ar_tool_callback(msg):
    global msg3
    msg3 = pose_to_string_list(msg)
    

if __name__ == '__main__':
    rospy.init_node('ros_to_zmq_forwarder', anonymous=True)
    rospy.Subscriber("/atracsys/Probe/measured_cp", PoseStamped, ar_tool_callback)  # Update topic and type
    rospy.Subscriber("/atracsys/Anatomy/measured_cp", PoseStamped, panel_callback)  # Update topic and type
    rospy.Subscriber("/atracsys/StaticRef/measured_cp", PoseStamped, ref_callback)  # Update topic and type
    rate = rospy.Rate(50)
    while not rospy.is_shutdown():
        publisher.send_multipart([b"StaticRef", msg1.encode()])
        publisher.send_multipart([b"Anatomy", msg2.encode()])
        publisher.send_multipart([b"Probe", msg3.encode()])
        rate.sleep()
rospy.spin()
publisher.close()
context.term()
