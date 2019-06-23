#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>

//クラス定義(どのような構造か定義)-----------------------------------------------------
class odom_publisher
{
  public:
  odom_publisher();
  
  private:
  
  //コールバック関数定義
  //void cb_odom_publisher(const sensor_msgs::LaserScan::ConstPtr &data);
  //void cb_image(const sensor_msgs::CompressedImage::ConstPtr &data);

  //ノードハンドラ作成
  ros::NodeHandle nh;

  //使用変数定義
  //使用するTopicの定義
  //ros::Subscriber sub_scan;
  //ros::Subscriber sub_image;
  ros::Publisher pub_map_odom;
  //transform用の変数
  //

  //一定間隔ごとにデバックするための関数
  void pub_send_odom(const ros::TimerEvent&);

  //時間の関数作成
  ros::Timer timer;

  tf::TransformListener listener;


};

//コンストラクタ定義(初期化時に必ず呼び出される部分)---------------------------------------
odom_publisher::odom_publisher(){
    //sub_scan = nh.subscribe("/scan", 5, &odom_publisher::cb_odom_publisher,this);
    //sub_image = nh.subscribe("/kinect2/sd/image_depth/compressed",5,&odom_publisher::cb_image,this);

    pub_map_odom = nh.advertise<nav_msgs::Odometry>("/odom", 1000);

    timer = nh.createTimer(ros::Duration(0.1), &odom_publisher::pub_send_odom,this);
}

//関数定義-----------------------------------------------------------------------
void odom_publisher::pub_send_odom(const ros::TimerEvent&){
    //変換した位置を取得変数
    tf::StampedTransform transform;
    //publish用のodom変数
    nav_msgs::Odometry pub_odom;

    ros::Time now = ros::Time::now();
    
    try{
        listener.waitForTransform("/map","/base_link",now,ros::Duration(1.0));
        listener.lookupTransform("/map","/base_link",now,transform);
    }catch(tf::TransformException &ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

    //framの取得
    pub_odom.header.frame_id = "odom";
    //位置の取得
    pub_odom.pose.pose.position.x = transform.getOrigin().x();
    pub_odom.pose.pose.position.y = transform.getOrigin().y();
    //姿勢の取得
    pub_odom.pose.pose.orientation.x = transform.getRotation().getX();
    pub_odom.pose.pose.orientation.y = transform.getRotation().getY();
    pub_odom.pose.pose.orientation.z = transform.getRotation().getZ();
    pub_odom.pose.pose.orientation.w = transform.getRotation().getW();
    //時間の取得
    //pub_odom.header.stamp = now;

    pub_map_odom.publish(pub_odom);
}


/*
void odom_publisher::cb_odom_publisher(const sensor_msgs::LaserScan::ConstPtr &data){
    //一度すべてコピー
  sensor_msgs::LaserScan send_data = *data;
  //データ数を数える
  int data_number =data->ranges.size();
  for(int i=0;i<data_number;i++){
      //nanかどうかの判定
      if(std::isnan(send_data.ranges[i])){
          send_data.ranges[i] = 0;
      }
      //infかどうかの判定
      if(std::isinf(send_data.ranges[i])){
          send_data.ranges[i] = 0;
      }

  }
  pub_odom_publisher.publish(send_data);
}

void odom_publisher::cb_image(const sensor_msgs::CompressedImage::ConstPtr &data){
    int size=0;
    for(int i=0;i<data->data.size();i++){
        if(data->data[i] == 255){
            i++;
        }
        size++;
    }
    //int size = data->data.size();
    ROS_INFO("new%d",size);
}
*/

//実行されるメイン関数---------------------------------------------------------------
int main(int argc, char** argv)
{
	ros::init(argc, argv, "odom_publisher");
	odom_publisher odom_publisher;
	ros::spin();
	return 0;
}
