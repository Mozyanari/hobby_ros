#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/transform_listener.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>

//クラス定義(どのような構造か定義)-----------------------------------------------------
class add_fram_id
{
  public:
  add_fram_id();
  private:

  //コールバック関数定義
  void cb_add_fram_id(const sensor_msgs::Imu::ConstPtr &data);

  //ノードハンドラ作成
  ros::NodeHandle nh;

  //使用変数定義
  //使用するTopicの定義
  ros::Subscriber sub_Imu;

  ros::Publisher  pub_Imu_fram;

};

//コンストラクタ定義(初期化時に必ず呼び出される部分)---------------------------------------
add_fram_id::add_fram_id(){
    sub_Imu = nh.subscribe("/imu/data", 5, &add_fram_id::cb_add_fram_id,this);

    pub_Imu_fram = nh.advertise<sensor_msgs::Imu>("/imu/frame_data", 1000);
}
//関数定義-----------------------------------------------------------------------
void add_fram_id::cb_add_fram_id(const sensor_msgs::Imu::ConstPtr &data){
  sensor_msgs::Imu send_data = *data;
  send_data.header.frame_id = "imu";
  pub_Imu_fram.publish(send_data);
}

//実行されるメイン関数---------------------------------------------------------------
int main(int argc, char** argv)
{
	ros::init(argc, argv, "add_fram_id");
	add_fram_id add_fram_id;
	ros::spin();
	return 0;
}
