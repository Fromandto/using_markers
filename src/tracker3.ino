#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <Servo.h>  
#include <math.h>
#define ARM1LEN 0.140
#define ARM2LEN 0.152
Servo myservoA;  
Servo myservoB;
Servo myservoC;
Servo myservoD;
ros::NodeHandle nh;
int pos = 20;   
int i;
int A,B,C,D; 
std_msgs::String str_msg;
ros::Publisher chatter("/degree" , &str_msg);
char buffer[50] = {0};

void messageCb(const geometry_msgs::Pose &msg)
{
  //double desz = double(map(double(msg.position.x),-0.15,0.15,-0.08,0.08));
  double desz = (double(msg.position.x) + 0.15) / 0.3 * -0.16 + 0.08;
  //double desy = double(map(double(msg.position.y),-0.2,0.2,0.105,0.165));
  double desy = (double(msg.position.y) + 0.15) / 0.3 * -0.06 + 0.165;
  //double desx = double(map(double(msg.position.z),0.0,1.0,0.03,0.2));
  double desx = (double(msg.position.z) - 0.2) / 0.7 * -0.17 + 0.2;
      String("des:" + String(int(desz*100)) + " " + String(int(desy*100)) + " " + String(int(desx*100))).toCharArray(buffer , 50);
    str_msg.data = buffer;
    chatter.publish(&str_msg);
  digitalWrite(13, HIGH-digitalRead(13));   // blink the led
  double arm1 = ARM1LEN;
    double arm2 = ARM2LEN;
    double theta1, theta2;
    double sx = 0.0;double sy = 0.105; double sz = 0.0;
    double dis = sqrt((desx-sx)*(desx-sx)+(desy-sy)*(desy-sy)+(desz-sz)*(desz-sz));
    double disbase = sqrt((desx-sx)*(desx-sx)+(desz-sz)*(desz-sz));
    
    double thetaBase = 90.0;
    bool need = false;
    double cosc=0.0, angc=0.0, cosb=0.0, angb=0.0;
    if(dis < arm2-arm1 || dis > arm2+arm1) {
      //printf("to long or two short\n");
    } else {
      cosc = (dis*dis+arm1*arm1-arm2*arm2)/(2.0*dis*arm1);
      angc = acos(cosc);
      cosb = (0.0-dis*dis+arm1*arm1+arm2*arm2)/(2.0*arm2*arm1);
      angb = acos(cosb);
      
      if( desx != 0) {
        thetaBase = atan(desz/desx);
        if(desx < 0 && desz > 0) {
          thetaBase += PI;
        }
        if(desx < 0 && desz < 0) {
          thetaBase -= PI;
        }
      }
      theta1 = angc + atan((desy-0.105)/disbase);
      theta2 = angb + theta1 - PI;
    }
    B=int(30.0+theta1/PI*180.0);
    C=int(30.0-theta2/PI*180.0);
    A=int(90.0-thetaBase/PI*180.0);
    String(String(B) + " " + String(C) + " " + String(A)).toCharArray(buffer , 50);
    str_msg.data = buffer;
    chatter.publish(&str_msg);
}

ros::Subscriber<geometry_msgs::Pose> sub("/posedecov" , messageCb);


void setup()
{
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(chatter);
  A=90;
  B=100;
  C=100;
  myservoA.attach(9);  // 控制腰部（A）的端口是~9号
  myservoB.attach(10); // 控制大臂（B）的端口是~10号
  myservoC.attach(11); // 控制小臂（C）的端口是~11号
  myservoD.attach(6); // 控制腕部（D）的端口是~6号
  pinMode(8,INPUT);
  pinMode(12,OUTPUT);
  digitalWrite(13,LOW);
  digitalWrite(12,HIGH);
  pinMode(13,OUTPUT);
}

void loop()
{
  nh.spinOnce();
  delay(1);
  myservoB.write(B); //初始化 大臂在90度 （可修改角度）
  myservoC.write(C); //初始化小臂在65度 （可修改角度）
  myservoA.write(A); //初始化腰部在50度  （可修改角度）
}
