#ROSでのオプティカルフローテストプログラム
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge #ROS2ImageとOpenCVをつなげるライブラリ
import cv2 #OpenCVライブラリ
import numpy as np

class RosOpticalFlow(Node):
    def __init__(self):
        super().__init__("realsense_subscriber_python")
        self.declare_parameter('image_topic_name', '/camera/camera/color/image_raw') #パラメータの宣言,launchでパラメータの設定変えた場合はトピック名を変える必要あり
        image_topic_name = self.get_parameter('image_topic_name').get_parameter_value().string_value #宣言したパラメータからパラメータ内の値を取得し、string型に変換

        video_qos = rclpy.qos.QoSProfile(depth=10) #QOSの設定,要は通信品質のやつ
        video_qos.reliability = rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT #BEST_EFFORTはやり取りするデータに欠損などが発生しても送る。初期設定だとRELIABLEになっていてデータが送られないらしい

        self.sub_img = self.create_subscription(#Image型,topic名image_topicnameのtopicを受け取る。通信品質は上記のvideo_qosに従う
            Image,
            image_topic_name,
            self.on_image_subscribed,
            video_qos
        )
        self.publisher = self.create_publisher(#publisher設定.OpenCV画像をROStiopicに変更して送信する
            Image,
            'opti',
            10
        )
        self.publisher_2 = self.create_publisher(
            Image,
            'opti_afmask',
            10
        )
        #チェックポイントとyamlの指定
        self.prev_gray = None  # 前のグレースケール画像を保存する変数


        self.sub_img
    
    def on_image_subscribed(self, img):#コールバック関数

        img_np = CvBridge().imgmsg_to_cv2(img)#realsenseから受け取った画像をOpenCVで使う画像に変換
        img_np = cv2.cvtColor(img_np, cv2.COLOR_BGR2RGB)#カラー画像に変換
        gray = cv2.cvtColor(img_np, cv2.COLOR_BGR2GRAY)#カラー画像をグレースケール画像へ変換

        

        if self.prev_gray is not None:
            # 密なオプティカルフローを計算
            flow = cv2.calcOpticalFlowFarneback(
                self.prev_gray, gray, None, 0.5, 3, 15, 3, 5, 1.2, 0)

            # フローの描画（HSV形式に変換して視覚化）
            hsv = np.zeros_like(img_np)
            hsv[..., 1] = 255
            mag, ang = cv2.cartToPolar(flow[..., 0], flow[..., 1])
            hsv[..., 0] = ang * 180 / np.pi / 2
            hsv[..., 2] = cv2.normalize(mag, None, 0, 255, cv2.NORM_MINMAX)

            hsv_c = hsv.copy()

            mask_green = np.array([35, 20, 20]) 
            mask_blue =  np.array([120, 255, 255])

            mask = cv2.inRange(hsv, mask_green, mask_blue)

            hsv[mask != 0] = [0, 0, 0]


            flow_rgb = cv2.cvtColor(hsv_c, cv2.COLOR_HSV2BGR)
            flow_rgb_mask = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)

            


            result_msg = CvBridge().cv2_to_imgmsg(flow_rgb, 'passthrough')
            result2_msg = CvBridge().cv2_to_imgmsg(flow_rgb_mask, 'passthrough')
            
            self.publisher.publish(result_msg)
            self.publisher_2.publish(result2_msg)
            
            cv2.waitKey(1)

        # 現在のフレームを次のフレームとの比較のために保存
        self.prev_gray = gray


def main(args=None):
    try:
        rclpy.init(args=args)#初期化
        rclpy.spin(RosOpticalFlow())#ノードを起動
    
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()#Ctrl+cで終了
        

if __name__ == "__main__":#Pythonスクリプトが直接実行された場合にのみにmain関数を実行
    main()