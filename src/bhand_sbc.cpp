#include <kcl_sbc/bhand_sbc.hpp>
BHand_SBC::BHand_SBC() : nh(){

    if (!nh.getParam("motor_index", motor_index)) ROS_ERROR("No Motor Index selected");
    if (!nh.getParam("control_topic", control_topic)) ROS_ERROR("No Control Topic selected");

    this->cur_joints.position.resize(30);

    this->joint_command.position.resize(30);
    t=0;
    Fhist.resize(SAMPLES);
    sq2=sqrt(2.0);
}


void BHand_SBC::reconf_callback(kcl_sbc::SBC_paramsConfig &config, uint32_t level) {
    this->beta=config.beta;
    this->delta=config.delta;
    this->eta=config.eta;
    this->f_ref=config.f_ref;
    this->multiplier=config.multiplier;
    ROS_INFO("Reconfigure Request");
}


double BHand_SBC::controller(double ref,double input){

    int current;
    if(!collected){
        current=t;
    }
    else{
        current=SAMPLES;
    }
    double prev = Fhist.at(t);
    SF=SF-prev+input; //subtract the replaced value and add the current in the vector
    Fhist.at(t)=input; //replace current value in history vector
    mF=SF/(double) current; // get the mean force
    double sF=0;
    for(int i=0;i<current;i++){
        sF+=(Fhist.at(i)-mF)*(Fhist.at(i)-mF);
    }
    sF=sqrt(sF/(double) (current-1));

    P_failure=0.5*(1-erf((input-(ref+beta))/(sq2*sF)));
    std_msgs::Float64 pfail;
    pfail.data=P_failure;
    prob_pub.publish(pfail);
    double command=eta*(P_failure-delta);


    t++;
    if(t>SAMPLES-1){
        collected=true;
        t=0;
    }
    return command*multiplier;
}

void BHand_SBC::ft_cb1(const sr_grasp_msgs::KCL_ContactStateStamped &msg){
    //double ft=sqrt(msg.tangential_force.x*msg.tangential_force.y+msg.tangential_force.y*msg.tangential_force.y+msg.tangential_force.z*msg.tangential_force.z);

    command1=controller(this->f_ref, -msg.Fnormal);
    ROS_INFO_THROTTLE(0.5,"command1: %f %f %f ",cur_joints.position.at(this->motor_index),joint_command.position.at(this->motor_index),command1);
}
void BHand_SBC::ft_cb2(const sr_grasp_msgs::KCL_ContactStateStamped &msg){    
    command2=controller(this->f_ref, -msg.Fnormal);
    ROS_INFO_THROTTLE(0.5,"command2: %f %f %f ",cur_joints.position.at(this->motor_index),joint_command.position.at(this->motor_index),command2);
}

void BHand_SBC::js_cb(const sensor_msgs::JointState &msg){
    this->cur_joints=msg;
}



double BHand_SBC::controller_p(double ref,double input){

        double e=ref-input;

            return 0.5*e;

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "bhand_sbc");

    BHand_SBC *bh = new BHand_SBC();

    //ros::Subscriber js_sub = bh->nh.subscribe("/barrett_hand/joint_states", 1000, &BHand_SBC::js_cb, bh);
    ros::Subscriber js_sub2 = bh->nh.subscribe("/joint_states", 1000, &BHand_SBC::js_cb, bh);
    dynamic_reconfigure::Server<kcl_sbc::SBC_paramsConfig> server;
    server.setCallback(boost::bind(&BHand_SBC::reconf_callback, bh,_1, _2));
    ros::Subscriber ft_sub1 = bh->nh.subscribe("/finger1/ContactState", 1000, &BHand_SBC::ft_cb1, bh);
    //ros::Subscriber ft_sub2 = bh->nh.subscribe("/finger2/ContactState", 1000, &BHand_SBC::ft_cb2, bh);

    bh->jcom_pub= bh->nh.advertise<sensor_msgs::JointState>("/bhand_node/command",10);
    bh->pos_con_pub= bh->nh.advertise<std_msgs::Float64>(bh->control_topic,10);
    bh->com_pub= bh->nh.advertise<std_msgs::Float64>("hand_command",10);
    bh->prob_pub= bh->nh.advertise<std_msgs::Float64>("prob_fail",10);



    ros::Rate rate(100);
    rate.sleep();
    ros::spinOnce();
    while(bh->cur_joints.velocity.size()==0){
        rate.sleep();
        ROS_INFO(". %d", bh->cur_joints.velocity.size());
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

            double spread_com=bh->controller_p(PI/2,bh->cur_joints.position.at(6));
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
