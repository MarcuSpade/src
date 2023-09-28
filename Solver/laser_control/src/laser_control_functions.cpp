#include "laser_control/laser_control_class.h"

LaserController::LaserController()
{
    n.param("laser_control/acquisition_rate", acquisition_rate, 30);

    n.param<std::string>("laser_control/receive_vel_topic", receive_vel_topic, "/cmd_vel");
    n.param<std::string>("laser_control/publish_vel_topic", publish_vel_topic, "/solver_traction/cmd_vel");

    n.param<std::string>("laser_control/tof_sensor_fl_topic", tof_sensor_fl_topic, "/tof_fl");
    n.param<std::string>("laser_control/tof_sensor_fm_topic", tof_sensor_fm_topic, "/tof_fm");
    n.param<std::string>("laser_control/tof_sensor_fr_topic", tof_sensor_fr_topic, "/tof_fr");
    n.param<std::string>("laser_control/tof_sensor_bl_topic", tof_sensor_bl_topic, "/tof_bl");
    n.param<std::string>("laser_control/tof_sensor_bm_topic", tof_sensor_bm_topic, "/tof_bm");
    n.param<std::string>("laser_control/tof_sensor_br_topic", tof_sensor_br_topic, "/tof_br");

    float i {20.0};
    n.param("laser_control/frontHorizontal1", frontHorizontal1, 0);
    n.param("laser_control/frontVertical", frontVertical, 1);
    n.param("laser_control/frontHorizontal2", frontHorizontal2, 2);
    n.param("laser_control/backHorizontal1", backHorizontal1, 0);
    n.param("laser_control/backVertical", backVertical, 1);
    n.param("laser_control/backHorizontal2", backHorizontal2, 2);
    n.param("laser_control/stop_frontal", stop_frontal, i);
    n.param("laser_control/stop_frontal_fast", stop_frontal_fast, i);
    n.param("laser_control/vertical_degree", vertical_degree, i);
    n.param("laser_control/reduction_rate", reduction_rate, i);

    // Topics to publish

    pub_vel = n.advertise<geometry_msgs::Twist>(publish_vel_topic,acquisition_rate);
    pub_led = n.advertise<std_msgs::Float32>("led_frequency",acquisition_rate);

    // Topic to subscribe

    // sub_laserFront = n.subscribe("/laser_front", acquisition_rate, &LaserController::front_callback, this);
    // sub_laserBack = n.subscribe("/laser_back", acquisition_rate, &LaserController::back_callback, this);
    sub_sensor_fl = n.subscribe(tof_sensor_fl_topic, acquisition_rate, &LaserController::fl_callback, this);
    sub_sensor_fm = n.subscribe(tof_sensor_fm_topic, acquisition_rate, &LaserController::fm_callback, this);
    sub_sensor_fr = n.subscribe(tof_sensor_fr_topic, acquisition_rate, &LaserController::fr_callback, this);
    sub_sensor_bl = n.subscribe(tof_sensor_bl_topic, acquisition_rate, &LaserController::bl_callback, this);
    sub_sensor_bm = n.subscribe(tof_sensor_bm_topic, acquisition_rate, &LaserController::bm_callback, this);
    sub_sensor_br = n.subscribe(tof_sensor_br_topic, acquisition_rate, &LaserController::br_callback, this);
    sub_vel = n.subscribe(receive_vel_topic, acquisition_rate, &LaserController::vel_callback, this);

    speedIn.linear.x = 0.0;
    speedIn.angular.z = 0.0;
    speedAux.linear.x = 0.0;
    speedAux.angular.z = 0.0;
    ledsFrequency = 0.0;
}

LaserController::~LaserController(){}

// void LaserController::front_callback(const sensor_msgs::ChannelFloat32& msg1)
// {
    // laserMessage1 = msg1;
    // change_front = true;
// }

void LaserController::fl_callback(const std_msgs::Float32& msg1)
{
    fl_msg = msg1;
}
void LaserController::fm_callback(const std_msgs::Float32& msg1)
{
    fm_msg = msg1;
}
void LaserController::fr_callback(const std_msgs::Float32& msg1)
{
    fr_msg = msg1;
}
void LaserController::bl_callback(const std_msgs::Float32& msg1)
{
    bl_msg = msg1;
}
void LaserController::bm_callback(const std_msgs::Float32& msg1)
{
    bm_msg = msg1;
}
void LaserController::br_callback(const std_msgs::Float32& msg1)
{
    br_msg = msg1;
}

// void LaserController::back_callback(const sensor_msgs::ChannelFloat32& msg2)
// {
    // laserMessage2 = msg2;
    // change_back = true;
// }

void LaserController::vel_callback(const geometry_msgs::Twist& data)
{
    speedIn = data;
    this->setAngular();
    this->setLinear();
    this->publish_speed();
    this->setAux();
}

// bool LaserController::get_change_front()
// {
    // return change_front;
// }
// 
// bool LaserController::get_change_back()
// {
    // return change_back;
// }

void LaserController::setAngular()
{
    frontais[0] = fl_msg.data;
    frontais[1] = fm_msg.data;
    frontais[2] = fr_msg.data;
    traseiros[0] = bl_msg.data;
    traseiros[1] = bm_msg.data;
    traseiros[2] = br_msg.data;

    // traseiros = laserMessage2.values;
    // if (speedIn.angular.z > 0.0 && (frontais[frontHorizontal1] <10.0 || traseiros[backHorizontal1] <10.0 || frontais[frontHorizontal2] <10.0 || traseiros[backHorizontal2] <10.0 )) // 
    // {
    //     speed_angular_z = 0.0;
    // }
    //     else if(speedIn.angular.z < 0.0 && (frontais[frontHorizontal1] <10.0 || traseiros[backHorizontal1] <10.0 || frontais[frontHorizontal2] <10.0 || traseiros[backHorizontal2] <10.0)) // 
    // {
    //     speed_angular_z = 0.0;
    // }
    // else
    // {
    //     speed_angular_z = speedIn.angular.z;
    // }
    speed_angular_z = speedIn.angular.z;
    // if(speed_angular_z != 0.0 )
    // {
        // ledsFrequency = 500.0;
    // }
}

void LaserController::setLinear()
{ 
    // Condicao para que o robo pare caso identifique algum obstaculo

    // setStopDist();

    // Condicao para que o robo pare caso identifique algum obstaculo

    if (speedIn.linear.x > 0.0 && (frontais[frontHorizontal1] < stop_frontal || frontais[frontHorizontal2] < stop_frontal)) // 
    { speed_linear_x = 0.0;}
    else if (speedIn.linear.x < 0.0 && (traseiros[backHorizontal1] < stop_frontal || traseiros[backHorizontal2] < stop_frontal)) //  
    { speed_linear_x = 0.0;}
    else if (speedIn.linear.x > 0.03 && (frontais[frontHorizontal1] < stop_frontal_fast || frontais[frontHorizontal2] < stop_frontal_fast)) // 
    { if (speedIn.linear.x < (reduction_count*reduction_rate + 0.03)) speed_linear_x = 0.03;
      else speed_linear_x = speedIn.linear.x - reduction_count*reduction_rate ;
      reduction = true;
      reduction_count++; }
    else if (speedIn.linear.x < -0.03 && (traseiros[backHorizontal1] < stop_frontal_fast || traseiros[backHorizontal2] < stop_frontal_fast)) //  
    { if (speedIn.linear.x > (-reduction_count*reduction_rate - 0.03)) speed_linear_x = -0.03;
      else speed_linear_x = speedIn.linear.x + reduction_count*reduction_rate ;
      reduction = true;
      reduction_count++; }
    else
    { speed_linear_x = speedIn.linear.x; reduction = false; reduction_count = 0; }

    if (speedIn.linear.x > 0.0 && frontais[frontVertical] > vertical_degree) // 
    { speed_linear_x = 0.0;
    speedIn.linear.x = 0;} 
    else if (speedIn.linear.x < 0.0 && traseiros[backVertical] > vertical_degree ) //  
    { speed_linear_x = 0.0;
    speedIn.linear.x = 0;}
    // else
    // { speed_linear_x = speedIn.linear.x;}


    // Caso seja identicado um vao a frente, ele inverte o sentido de giro e altera a FLAG turn_back
    // if (frontais[frontVertical] > vertical_degree || traseiros[backVertical] > vertical_degree)
    // {
    //     speed_linear_x = speedIn.linear.x;
    //     turn_back = true;
    // }

    // Ao identificar que saiu do buraco, ele compara com a FLAG para ver se esta girando no sentido contrario para que o robo pare.
    // if (frontais[frontVertical] < vertical_degree && traseiros[backVertical] < vertical_degree && turn_back == true)
    // {
    //     speed_linear_x = 0;
    //     turn_back = false;
    //     speedIn.linear.x = 0;
    // }

    // A freqûencia de acionamento dos LEDs e alterada conforme identifica obstaculos
 
    if (speed_linear_x != 0 && frontais[frontHorizontal1] > 80.0 && traseiros[backHorizontal1] > 80 && frontais[frontHorizontal2] > 80.0  && traseiros[backHorizontal2] > 80 ) // Obstaculo acima de 60 cm ->   ->> 
    { ledsFrequency = 1000.0;}
    else if (speed_linear_x != 0 && (frontais[frontHorizontal1] > 30.0 || traseiros[backHorizontal1] > 30 || frontais[frontHorizontal2] > 30.0 || traseiros[backHorizontal2] > 30 )) // Obstaculo acima de 30 cm -> 
    { ledsFrequency = 500.0;}
    else if (speed_linear_x != speedIn.linear.x) // Obstaculo abaixo de 30 cm
    { ledsFrequency = 250.0;}
    else if (speedIn.linear.x == 0 && speedIn.angular.z == 0) // Robô parado
    { ledsFrequency = 0.0;}

    // Setado saida conforme calculado no método

    speedOut.linear.x = speed_linear_x;
    speedOut.angular.z = speed_angular_z;

}

float LaserController::getOutLinear()
{
    return speedOut.linear.x;
}

float LaserController::getOutAngular()
{
    return speedOut.angular.z;
}

float LaserController::getAuxLinear()
{
    return speedAux.linear.x;
}

float LaserController::getAuxAngular()
{
    return speedAux.angular.z;
}

void LaserController::publish_speed()
{
    pub_vel.publish(speedOut);
}

void LaserController::publish_frequency()
{
    ledsFrequency32.data = ledsFrequency;
    pub_led.publish(ledsFrequency32);
}

void LaserController::setAux()
{
    speedAux.linear.x = speedOut.linear.x;
    speedAux.angular.z = speedOut.angular.z;
}

void LaserController::setStopDist(){        
    //stop_frontal = (abs(speedIn.linear.x))*50+10;
}
