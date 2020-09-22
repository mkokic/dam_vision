import rospy
import argparse
import pcl
import ros_numpy
from skimage import data, color, img_as_ubyte
import numpy as np
import cv2
import struct
import ctypes
from sensor_msgs import point_cloud2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from std_msgs.msg import Header, Int64
from geometry_msgs.msg import Point
from gpd.msg import GraspConfigList, CloudIndexed
from hough_circle.msg import my_msg
from IPython import embed

class SelectGrasps(object):
    def __init__(self):
        self.parser = argparse.ArgumentParser()
        self.parser.add_argument('--mode', required=True, type=int, choices=[1, 2], help='initial or second grasp')

    def parse(self):
        return self.parser.parse_args()

def get_rgb(data):
    s = struct.pack('>f', data)
    i = struct.unpack('>l', s)[0]
    pack = ctypes.c_uint32(i).value
    r = int((pack & 0x00FF0000)>> 16)
    g = int((pack & 0x0000FF00)>> 8)
    b = int(pack & 0x000000FF)
    return [r, g, b]

depth_image = []
def depthCallback(msg):
    global depth_image
    if len(depth_image) == 0:
        depth_image.append(np.nan_to_num(bridge.imgmsg_to_cv2(msg, 'passthrough')))

K = []
def cameraCallback(msg):
    global K
    if len(K) == 0:
        K.append(np.matrix(msg.K, dtype='float64'))

bridge = CvBridge()
def imageCallback(msg):
    global cv_image
    cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (0, 0, 212), (131, 255, 255))
    global blur
    blur = cv2.GaussianBlur(mask, (9, 9), 2, 2)
    drawImg = cv2.cvtColor(blur, cv2.COLOR_GRAY2BGR)
    image_pub = rospy.Publisher("/filtered_image",Image, queue_size=1)
    pub = image_pub.publish(bridge.cv2_to_imgmsg(drawImg, 'bgr8'))

rim_points = []
def rimCallback(msg):
    global rim_points
    rim_points.append([msg.points[0].x, msg.points[0].y])

# Create a ROS node.
rospy.init_node('select_grasp')
camera_info = rospy.Subscriber('/camera/rgb/camera_info',CameraInfo, cameraCallback)
depth_sub = rospy.Subscriber('/camera/depth_registered/image_raw', Image, depthCallback)
rospy.sleep(1.0)
image_sub = rospy.Subscriber('/camera/rgb/image_raw',Image,imageCallback)
rospy.sleep(5.0)

# blur instead of cv_image
ind = np.where((blur > 0))
x, y = ind[0], ind[1]

cloud = []
if len(cloud) == 0:
    for v, u in zip(x, y):
        color = cv_image[v, u]
        Z = depth_image[0][v, u]
        if Z == 0: continue
        X = (u - np.array(K[0])[0, 2]) * Z / np.array(K[0])[0, 0]
        Y = (v - np.array(K[0])[0, 5]) * Z / np.array(K[0])[0, 0]
        a = 255
        rgb = struct.unpack('I', struct.pack('BBBB', color[2], color[1], color[0], a))[0]
        cloud.append([X, Y, Z, rgb])

# Wait for point cloud to arrive.
while len(cloud) == 0:
    rospy.sleep(0.01)

fields = [PointField('x', 0, PointField.FLOAT32, 1),
          PointField('y', 4, PointField.FLOAT32, 1),
          PointField('z', 8, PointField.FLOAT32, 1),
          PointField('rgba', 12, PointField.UINT32, 1),
          ]

rim_sub = rospy.Subscriber('/rim_points', my_msg, rimCallback)

print len(cloud)
print 'Publishing point cloud...'
pub = rospy.Publisher("my_cloud", PointCloud2, queue_size=2)
header = Header()
header.frame_id = "/camera_depth_optical_frame"
pc2 = point_cloud2.create_cloud(header, fields, cloud)
s = raw_input('Hit [ENTER] to publish')
pub.publish(pc2)
rospy.sleep(1.0)

def callback(msg):
    global grasps
    grasps = msg.grasps

grasps = [] # global variable to store grasps
# Subscribe to the ROS topic that contains the grasps.
grasps_sub = rospy.Subscriber('/detect_grasps/clustered_grasps', GraspConfigList, callback)
# Wait for grasps to arrive.
rate = rospy.Rate(1)
while not rospy.is_shutdown():
    if len(grasps) > 0:
        rospy.loginfo('Received %d grasps.', len(grasps))
        break

opt = SelectGrasps().parse()
if opt.mode == 1:
    grasps_filt = [g for g in grasps if g.approach.z > 0.8]
    #and np.abs(np.array(rim_points).reshape(-1, 2)[:, 0].mean() - g.surface.x * 1000.) < 50.
    print np.array(rim_points).reshape(-1, 2)[:, 0].mean() - 320.
    print grasps_filt
if opt.mode == 2:
    grasps_filt = [g for g in grasps if g.approach.x > 0.8 and (np.abs(g.surface.x * 1000. - np.array(rim_points).reshape(-1, 2)[:, 0].mean()) < 50.)]
    print grasps_filt
