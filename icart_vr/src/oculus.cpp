#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/transform_listener.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>


#define M_PI       3.14159265358979323846  /* pi */
//ロボットの最高速度
#define ROBOT_V 0.1
#define ROBOT_W 0.5

//クラス定義(どのような構造か定義)-----------------------------------------------------
class oculus
{
  //
  public:
  oculus();
  private:

  //コールバック関数定義
  void cb_oculus(const geometry_msgs::PoseStamped::ConstPtr &data);
  void cb_contoroller(const geometry_msgs::PoseStamped::ConstPtr &data);
  void cb_triger(const sensor_msgs::Joy::ConstPtr &data);
  void cb_wii(const sensor_msgs::Imu::ConstPtr &data);
  void cb_wii_button(const sensor_msgs::Joy::ConstPtr &data);

  //今のコントローラの角度
  double roll, pitch, yaw;

  //基準となるコントローラの角度
  double standard_roll, standard_pitch, standard_yaw;

  //icartに送信する速度
  geometry_msgs::Twist icart_speed;


  //ノードハンドラ作成
  ros::NodeHandle nh;

  //使用変数定義
  //使用するTopicの定義
  ros::Subscriber sub_oculus;
  ros::Subscriber sub_controller;
  ros::Subscriber sub_triger;
  ros::Subscriber sub_wii;
  ros::Subscriber sub_wii_button;

  ros::Publisher pub_head_data;
  ros::Publisher pub_wii;
  ros::Publisher pub_speed;

};

//コンストラクタ定義(初期化時に必ず呼び出される部分)---------------------------------------
oculus::oculus(){
    sub_oculus = nh.subscribe("/camera", 5, &oculus::cb_oculus,this);
    sub_controller = nh.subscribe("/contoroller", 5, &oculus::cb_contoroller,this);
    sub_triger = nh.subscribe("/triger", 5, &oculus::cb_triger,this);
    sub_wii = nh.subscribe("/imu/data_raw", 5, &oculus::cb_wii,this);
    sub_wii_button = nh.subscribe("/joy", 5, &oculus::cb_wii_button,this);

    pub_head_data = nh.advertise<geometry_msgs::Pose2D>("head_pose", 1000);
    pub_wii = nh.advertise<geometry_msgs::Pose2D>("wii_pose", 1000);
    pub_speed = nh.advertise<geometry_msgs::Twist>("/ypspur_ros/cmd_vel", 1000);
}
//関数定義-----------------------------------------------------------------------
//Oculus go本体の角度を取得
void oculus::cb_oculus(const geometry_msgs::PoseStamped::ConstPtr &data){
    geometry_msgs::PoseStamped a = *data;
    //double theta = tf::getYaw(a.pose.orientation);
    //ROS_INFO("theta=%f",theta);
    //出力値
    double head_roll;
    double head_pitch;
    double head_yaw;
    //入力値を一度変換
    tf::Quaternion quat(a.pose.orientation.x,a.pose.orientation.y,a.pose.orientation.z,a.pose.orientation.w);
    //roll,pitch,yawを計算
    tf::Matrix3x3(quat).getRPY(head_roll, head_pitch, head_yaw);
    //radからdegに変換
    head_roll = head_roll * (180.0 / M_PI);
    head_pitch = head_pitch * (180.0 / M_PI);
    head_yaw = head_yaw * (180.0 / M_PI);
    //Oculusgoの姿勢を送信
    geometry_msgs::Pose2D head_pose;
    head_pose.x = head_roll;
    head_pose.y = head_pitch;
    head_pose.theta = head_yaw;
    pub_head_data.publish(head_pose);
}
//Oculus goのコントローラの角度を取得
void oculus::cb_contoroller(const geometry_msgs::PoseStamped::ConstPtr &data){
  static double old_roll;
  static double old_pitch;
  static double old_yaw;

  tf::Quaternion quat(data->pose.orientation.x, data->pose.orientation.y ,data->pose.orientation.z, data->pose.orientation.w);
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  roll = roll * (180.0 / M_PI);
  pitch = pitch * (180.0 / M_PI);
  yaw = yaw * (180.0 / M_PI);
  ROS_INFO("roll=%f, pitch=%f, yaw=%f",roll,pitch,yaw);

  if((abs(old_roll-roll)>abs(old_pitch-pitch)) && (abs(old_roll-roll)>abs(old_yaw-yaw))){
    ROS_INFO("roll_rotation");
  }else if((abs(old_pitch-pitch)>abs(old_roll-roll)) && (abs(old_pitch-pitch)>abs(old_yaw-yaw))){
    ROS_INFO("pitch_rotation");
  }else if((abs(old_yaw-yaw)>abs(old_roll-roll)) && (abs(old_yaw-yaw)>abs(old_pitch-pitch))){
    ROS_INFO("yaw_rotation");
  }

  old_roll = roll;
  old_pitch = pitch;
  old_yaw = yaw;


  ROS_INFO("roll=%f, pitch=%f, yaw=%f",roll,pitch,yaw);
}

//コントローラのtrigerの状態を取得
void oculus::cb_triger(const sensor_msgs::Joy::ConstPtr &data){
  /*
  static int state = 0;
  static int old_state = 0;

  geometry_msgs::Twist robo_speed;

  //今のコントローラの状態を入力
  state = data->buttons[0];

  //今trigerが押されたか判定
  if((state == 1)&&(old_state == 0)){
    //今押された
    standard_roll = roll;
    standard_pitch = pitch;
    standard_yaw = yaw;
  }else if((state ==1)&&(old_state == 1)){
    //押されて続けてるからそれに合わせて速度
    double diff_roll = roll - standard_roll;
    double diff_pitch = pitch - standard_pitch;

    //速度を代入
    robo_speed.linear.x = -0.05*(diff_roll);
    robo_speed.angular.z = 0.05*(diff_pitch);

    //0.05以下なら0とみなす
    if(abs(robo_speed.linear.x)<0.05){
      robo_speed.linear.x=0;
    }
    if(abs(robo_speed.angular.z)<0.05){
      robo_speed.angular.z=0;
    }

    //バックしているときはwを逆にする
    if(robo_speed.linear.x < 0){
      robo_speed.angular.z = -(robo_speed.angular.z); 
    }

    //速度を送信
    pub_speed.publish(robo_speed);
  }else{
    //押されてないから停止
    robo_speed.linear.x = 0;
    robo_speed.angular.z = 0;

    //速度を送信
    pub_speed.publish(robo_speed);
  }

  //今の状態を保存
  old_state = state;
  */
}

//Wiiリモコンの状態を取得
void oculus::cb_wii(const sensor_msgs::Imu::ConstPtr &data){
  //回転速度を決定
  icart_speed.angular.z=(data->linear_acceleration.y/9.8)*ROBOT_W;
  
  

  

  


  /*
  sensor_msgs::Imu wii = *data;
  tf::Quaternion quat(wii.orientation.x, wii.orientation.y ,wii.orientation.z, wii.orientation.w);
  //出力値
  double wii_roll;
  double wii_pitch;
  double wii_yaw;
  tf::Matrix3x3(quat).getRPY(wii_roll, wii_pitch, wii_yaw);
  wii_roll = wii_roll * (180.0 / M_PI);
  wii_pitch = wii_pitch * (180.0 / M_PI);
  wii_yaw = wii_yaw * (180.0 / M_PI);
  ROS_INFO("wii_roll=%f, wii_pitch=%f, wii_yaw=%f",wii_roll,wii_pitch,wii_yaw);

  //Oculusgoの姿勢を送信
  geometry_msgs::Pose2D wii_pose;
  wii_pose.x = wii_roll;
  wii_pose.y = wii_pitch;
  wii_pose.theta = wii_yaw;
  pub_wii.publish(wii_pose);
  */



}
void oculus::cb_wii_button(const sensor_msgs::Joy::ConstPtr &data){
  if(data->buttons[1]){
    icart_speed.linear.x = ROBOT_V;
  }else if(data->buttons[3]){
    icart_speed.linear.x = -ROBOT_V;
    icart_speed.angular.z = -icart_speed.angular.z;
  }else{
    icart_speed.linear.x = 0;
    icart_speed.angular.z = 0;
  }

  //速度を送信
  pub_speed.publish(icart_speed);
}



//実行されるメイン関数---------------------------------------------------------------
int main(int argc, char** argv)
{
	ros::init(argc, argv, "oculus");
	oculus oculus;
	ros::spin();
	return 0;
}
