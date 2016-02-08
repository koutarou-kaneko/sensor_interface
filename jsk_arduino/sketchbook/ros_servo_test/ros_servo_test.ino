#include <Servo.h>
#include <ros.h>
//#include <std_msgs/Empty.h>
#include <std_msgs/UInt8.h>


//servo
Servo servo;//サーボのインスタンス

ros::NodeHandle  nh;

void servoCmdCb( const std_msgs::UInt8& msg){
    servo.write(msg.data);//0~180まで,サーボ出力
    
  // 180度が反時計回りの限界, 160度ぐらいにする
  // 90度が中間,ちょい下, だから100度ぐらいにする
  // 0度が時計回りの限界、7時より 
}

ros::Subscriber<std_msgs::UInt8> sub("servo_cmd", &servoCmdCb );

void setup(){
  //サーボの信号線を３番ピンに接続
  //（PWMピン以外のピンにも接続可）
  servo.attach(3);

  nh.initNode();
  nh.subscribe(sub);
  
  int deg = 10; //about 900us?
  servo.write(deg);//0~180まで,サーボ出力
}

void loop(){
  nh.spinOnce();
  delay(1);
}
  
  
