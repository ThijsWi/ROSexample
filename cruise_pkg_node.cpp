#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/BatteryState.h"
#include "sensor_msgs/Range.h"
#include "sensor_msgs/Imu.h"
#include "tf/tf.h"

ros::Publisher speed_pub;
geometry_msgs::Twist set_vel;

float rangesout = 0;
float diff = 0;
float diff0 = 0;
float diffout1 = 0;
float diffout2 = 0;

int delay = 200;
float dt = 0.2; // delay/1000 moet seconde zijn

float speed;
float speedset = 0.3;
float rotate;
float rotateset;
int teller = 0;
float Vx, Vy, Inc, spd, spd2;// spd;
float VelX, VelY;
float xPoseOud = 0, yPoseOud = 0;

float rfla = 0;
float rfra = 0;
float rfl1 = 0;
float rfr1 = 0;
float rfl2 = 0;
float rfr2 = 0;
float rfl = 0;
float rfr = 0;

float pose0;
float pose1;

void distfl_callback(const sensor_msgs::Range &range)
{
   rfl = range.range;
}

void distfr_callback(const sensor_msgs::Range &range)
{
   rfr = range.range;
}

void pose_callback(const geometry_msgs::PoseStamped &pose) {
pose0 = pose.pose.position.x;
pose1 = pose.pose.position.y;
}

void Realspd(float Vx, float Vy){
    float spd2;
    spd2 = pow(Vx, 2) + pow(Vy, 2);
    spd = pow(spd2, 0.5);
}

void posit(){
    VelX=(pose0-xPoseOud)/dt;
    VelY=(pose1-yPoseOud)/dt;
    Realspd(VelX, VelY);
}
void ranges(){
  if (rfl>1){
    rfl=1;
  }
  if (rfr>1){
    rfr=1;
  }
  rfla=(rfl+rfl1+rfl2)/3;
  rfra=(rfr+rfr1+rfr2)/3;
}

void speedinit(){
    diff= (rangesout - rfla)/dt;
    //diff=(diff0+diffout1+diffout2)/3;
    if (((rfla < 0.6) && (rfla > 0.25) && (speed > 0)) || ((rfra < 0.6) && (rfra > 0.25 ) && (speed > 0))){
        speed = spd - diff;
    }

    else if (((rfla < 0.25) || (rfra < 0.25))){
            speed = 0;
            rotate = 0;
    }
    else {
        speed = speedset;
        rotate = rotateset;
    }
}


void opslag(){
    rfl2=rfl1;
    rfl1=rfl;
    rfr2=rfr1;
    rfr1=rfr;
    diffout2=diffout1;
    diffout1=diff0;
    rangesout=rfla;
    xPoseOud=pose0;
    yPoseOud=pose1;
}

//PID

double _Kp = 0.1;
double _Ki = 0;
double _Kd = 0;
double _pre_error, _integral;
double _max = 1000;
double _min = -1000;

double calculate( double setpoint, double pv )
{
    // Calculate error
    double error = setpoint - pv;

    // Proportional term
    double Pout = _Kp * error;

    // Integral term
    _integral += error * dt;
    double Iout = _Ki * _integral;

    // Derivative term
    double derivative = (error - _pre_error) / dt;
    double Dout = _Kd * derivative;

    // Calculate total output
    double output = Pout + Iout + Dout;

    // Restrict to max/min
    if( output > _max )
        output = _max;
    else if( output < _min )
        output = _min;

    // Save error to previous error
    _pre_error = error;

    return output;
}

float SpdtoCon(float speed, float spd){
    double inc = calculate(speed, spd);
    float ftval = 1;
    float spdInc = ftval * inc; //need to finetude value ftval to accurately finetune to certain angle

    speed = speed + spdInc;
    return spdInc;
}
    void limit(){
            if (speed>speedset){
                speed=speedset;
            }
            if ((speedset>0) && (speed<0)){
                speed=0;
            }
            set_vel.linear.x = speed;
            set_vel.angular.z = rotate;

    }
int main(int argc, char **argv)
{
    ros::init(argc, argv,"action_controller");
    ros::NodeHandle n("~");
    ros::Subscriber distfl_sub = n.subscribe("/range/fl", 1, distfl_callback);
    ros::Subscriber distfr_sub = n.subscribe("/range/fr", 1, distfr_callback);
    ros::Subscriber pose_sub = n.subscribe("/pose", 2, pose_callback);



    //printf("start");
    //rosbot.initROSbot()//sensor_type);        //IMU Battery Monitor  Odometry Distance sensor wheel control
    ros::Rate loop_rate(5);
    speed_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    set_vel.linear.x = 0;
    set_vel.linear.y = 0;
    set_vel.linear.z = 0;
    set_vel.angular.x = 0;
    set_vel.angular.y = 0;
    set_vel.angular.z = 0;

    while(ros::ok()){
      //printf("\r\nsetvel %f",set_vel.linear.x);
      printf("\r\nspeed %f",speed);
      //printf("\r\ninc %f",Inc);
      //printf("\r\npose0 %f",pose0);
      //printf("\r\ndiff %f",diff);
      //printf("\r\ndiff0 %f",diff0);
      //printf("\r\nrfla %f",rfla);

        posit();
        ranges();
        speedinit();
        opslag();
        Inc=SpdtoCon(speed,spd);
        limit();

        //rosbot.setSpeed(speed,rotate);                    //problem
        speed_pub.publish(set_vel);
        ros::spinOnce();
				loop_rate.sleep();

    }

		return 0;
}
