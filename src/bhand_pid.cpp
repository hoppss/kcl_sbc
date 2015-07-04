#include <kcl_sbc/bhand_pid.hpp>

BHand_PID::BHand_PID() : nh(){

    if (!nh.getParam("motor_index", motor_index)) ROS_ERROR("No Motor Index selected");
    if (!nh.getParam("control_topic", control_topic)) ROS_ERROR("No Control Topic selected");

    this->cur_joints.position.resize(30);

    this->joint_command.position.resize(30);
    }


    void BHand_PID::reconf_callback(kcl_sbc::PID_paramsConfig &config, uint32_t level) {

        this->P=config.p_gain;
        this->I=config.i_gain;
        this->D=config.d_gain;

        this->ref=config.f_ref;

        this->deadband=config.deadband/100.0;
        this->a_windup=config.a_windup/100.0;

        ROS_INFO("Reconfigure Request");


    }


    double BHand_PID::controller(double ref,double input){

            double e=ref-input;
            ROS_INFO_THROTTLE(0.2,"error: %f - %f = %f",ref,input,e);

            if(fabs(e/ref)>this->deadband) {
                if(fabs(e/ref)>this->a_windup ) ie=0; //anti-windup
                else 		{
                    this->ie=this->ie+e;
                }

                float de=e-this->prev;

                this->prev=e;

                return this->P*e+this->I*ie+this->D*de;
            }

            else {
                return 0;
            }
    }

    void BHand_PID::ft_cb1(const sr_grasp_msgs::KCL_ContactStateStamped &msg){
       command1=controller(this->ref, -msg.Fnormal);
       ROS_INFO_THROTTLE(0.5,"command1: %f %f %f ",cur_joints.position.at(this->motor_index),joint_command.position.at(this->motor_index),command1);
    }
    void BHand_PID::ft_cb2(const sr_grasp_msgs::KCL_ContactStateStamped &msg){
       command2=controller(this->ref, -msg.Fnormal);
       ROS_INFO_THROTTLE(0.5,"command2: %f %f %f ",cur_joints.position.at(this->motor_index),joint_command.position.at(this->motor_index),command2);
    }

    void BHand_PID::js_cb(const sensor_msgs::JointState &msg){
      this->cur_joints=msg;
    }





int main(int argc, char **argv) {
  ros::init(argc, argv, "bhand_pid");

  BHand_PID *bh = new BHand_PID();

  //ros::Subscriber js_sub = bh->nh.subscribe("/barrett_hand/joint_states", 1000, &BHand_PID::js_cb, bh);
  ros::Subscriber js_sub2 = bh->nh.subscribe("/joint_states", 1000, &BHand_PID::js_cb, bh);
  dynamic_reconfigure::Server<kcl_sbc::PID_paramsConfig> server;
  server.setCallback(boost::bind(&BHand_PID::reconf_callback, bh,_1, _2));
  ros::Subscriber ft_sub1 = bh->nh.subscribe("/finger1/ContactState", 1000, &BHand_PID::ft_cb1, bh);
  ros::Subscriber ft_sub2 = bh->nh.subscribe("/finger2/ContactState", 1000, &BHand_PID::ft_cb2, bh);

  bh->jcom_pub= bh->nh.advertise<sensor_msgs::JointState>("/bhand_node/command",10);
  bh->pos_con_pub= bh->nh.advertise<std_msgs::Float64>(bh->control_topic,10);
  bh->com_pub= bh->nh.advertise<std_msgs::Float64>("hand_command",10);

      
  ros::Rate rate(100);
  rate.sleep();
  ros::spinOnce();
  while(bh->cur_joints.velocity.size()==0){
      rate.sleep();
       ROS_INFO(".");
      ros::spinOnce();
}

  bh->joint_command=bh->cur_joints;
  bh->joint_command.position.at(6)=PI/2;
  bh->joint_command.position.at(7)=PI/2;
  bh->jcom_pub.publish(bh->joint_command);
  sleep(1);
  std_msgs::Float64 com_msg;
    bool POSITION=false;
  while(ros::ok()){
      ros::spinOnce();          
      rate.sleep();
      if(POSITION){
      bh->joint_command=bh->cur_joints;
      bh->joint_command.position.at(1)+=bh->command1;
      bh->joint_command.position.at(4)+=bh->command2;
      //ROS_INFO_STREAM(bh->joint_command.position.at(1) << "," << bh->joint_command.position.at(4));
      bh->jcom_pub.publish(bh->joint_command);
    }
      else{
          bh->joint_command=bh->cur_joints;
          std::fill(bh->joint_command.velocity.begin(), bh->joint_command.velocity.end(), 0);
          bh->joint_command.velocity.at(1)=bh->command1;
          bh->joint_command.velocity.at(2)=bh->command1;

          double spread_com=bh->controller(PI/2,bh->cur_joints.position.at(6));
          bh->joint_command.velocity.at(6)=spread_com;
          bh->joint_command.velocity.at(7)=spread_com;

          bh->jcom_pub.publish(bh->joint_command);

      }
      //com_msg.data=bh->joint_command.position.at(1);
      //bh->pos_con_pub.publish(com_msg);
      //bh->com_pub.publish(com_msg);


  }
  return 0;
}
