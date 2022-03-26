from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from test_msgs.msg import LOCAL_POS_DATA, VariableFusionMap

class RUN():
    def __init__(self):
        self.img_h = 500
        self.img_w = 250
        self.grid_size = 20 #unit:cm
        self.dr_x = 0
        self.dr_y = 0
        self.dr_heading = 0
        self.last_dr_x = 0
        self.last_dr_y = 0
        self.last_dr_heading = 0
        self.count = 0
        self.is_first = True
        self.last_mask = np.zeros(125000, dtype=np.int16)
        self.input_img = np.zeros(125000, dtype=np.uint8)
        self.mask_pub = rospy.Publisher('/Mask_Map', VariableFusionMap, queue_size=10)
        self.subscriber = rospy.Subscriber('/BEV_IMG', VariableFusionMap, self.bev_cb)

    def TR_process(self):
       
    def frames_accumulation(self)

    def road_seg(self):
        
    def bev_cb(self, msg):
        start2 = time.time()
        self.input_img = np.frombuffer(msg.positive_map, dtype=np.uint8)
        dr_x, dr_y, dr_heading = msg.dr_x, msg.dr_y, msg.dr_heading
        self.count += 1

        if self.count == 1 and not (msg.positive_map is None):
            self.road_seg(dr_x, dr_y, dr_heading)
            self.count = 0
        else:
            self.count = 0
            print("No Data Input !!!")

        end2 = time.time()
        print("Process   Time: %.3f" % (end2 - start2))

    def pub_mask(self):
        

def start():
    run = RUN()
    rospy.init_node('road_seg', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    start()
