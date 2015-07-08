#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <time.h>
//#include <signal.h>
//#include <bhand.h>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/JointState.h>
#define PI 3.14159265
#include <bhand.h>
//#include <kclbarrett/API/BHandAppHelper.h>
//#include <kclbarrett/API/BHandCANDriver.h>
//#include <Windows.h>

class Hand {
public:

    BHand bh;
    // BHandCANDriver bhcan;

    Hand();
    
    void Error();
    void Initialize();
    void PrepareRealTime();
    int RunRealTime();
    bool grip_srv(std_srvs::Empty::Request& , std_srvs::Empty::Response& );
    bool rels_srv(std_srvs::Empty::Request& , std_srvs::Empty::Response& );
    bool reset_srv(std_srvs::Empty::Request& , std_srvs::Empty::Response& );
    
    void vUpdate(const std_msgs::Float32MultiArray::ConstPtr&);
    ros::NodeHandle n;
    
    
    int hpos[4];
    
    ros::ServiceServer srv,srv2,srv3;
    int result,option;
    int v[4];
    ros::Publisher fencoder_pub,fenctwist_pub,hjoints_pub;
    ros::Subscriber sub2;
    
    std_msgs::Float32MultiArray handarray;
    geometry_msgs::Twist enctwist;
    
    sensor_msgs::JointState hjoints;
    
private:
    
    
    
};

Hand::Hand(){
    ros::NodeHandle n;
    //  hjoints.set_name_size(8);
    /*    hjoints.set_position_size(8);
    hjoints.set_velocity_size(8);
    hjoints.set_effort_size(8);*/
    hjoints.name.push_back("BHand/FingerOne3/KnuckleThreeJoint");
    hjoints.name.push_back("BHand/FingerOne/KnuckleTwoJoint");
    hjoints.name.push_back("BHand/FingerOne/KnuckleOneJoint");
    hjoints.name.push_back("BHand/FingerTwo/KnuckleOneJoint");
    hjoints.name.push_back("BHand/FingerThree/KnuckleThreeJoint");
    hjoints.name.push_back("BHand/FingerThree/KnuckleTwoJoint");
    hjoints.name.push_back("BHand/FingerTwo/KnuckleTwoJoint");
    hjoints.name.push_back("BHand/FingerTwo/KnuckleThreeJoint");
    

    handarray.data.resize(4);
    fencoder_pub = n.advertise<std_msgs::Float32MultiArray>("fenc", 1000);
    fenctwist_pub = n.advertise<geometry_msgs::Twist>("fenctwist", 1000);
    hjoints_pub = n.advertise<sensor_msgs::JointState>("jenc", 1000);
    //  v={0,0,0,0};
    
}


void Hand::Error(){
    printf("ERROR: %d\n%s\n", result, bh.ErrorMessage(result));
    exit(0);
}



void Hand::Initialize() {
    // Set hardware description before initialization
    //int hwIndex = BHandHardware::getBHandHardwareIndex("BH8-262");

  //  int hwIndex = BHandHardware::getBHandHardwareIndex("BH8-280");
   // if (hwIndex < 0)
   // {
    //    printf("The API has not been compiled to include target hand.");
    //    Error();
   // }
   // bh.setHardwareDesc(hwIndex);
    //bool use280Config = (strcmp(bh.getHardwareDesc()->getModelNumber(), "BH8-280") == 0);

    bool use280Config = true;
    //printf("use280Config = %d", use280Config);
   // if (result = handInitWithMenu(&bh))
    //    Error();
    
    /*	int port;
        
        printf( "\n\n\r\t\tInitializing Software..." );
        printf( "\n\n\r\t\tPlease enter desired serial port> " );
        scanf("%d", &port);
        printf( "\n" );
       */
        if( result=bh.InitSoftware(1,THREAD_PRIORITY_TIME_CRITICAL) )   //allow use of any serial port
                Error();
                
        if( result=bh.ComSetTimeouts((DWORD)100,(DWORD)25000,(DWORD)100,(DWORD)5000,(DWORD)4000) )
                Error();
    
    // EH commented this out
    //if( result=bh.Baud(38400) )
    //	Error();
    
    //AC 010711 Removed FDZ and FIP setting for compliance w/ firmware v4.21 (version 4.21)
    
    //	if( result=bh.Command("SFSET FDZ 251") )
    //		Error();
    
    //	if( result=bh.Command("SFSET FIP 0") )
    //		Error();3
    
    //	if( result=bh.Command ( "SFSET MPE 1000" ) )  //AC 010711 spread MPE was 100 changed so
    //		Error();								 //program would crash less	(version 4.21)
    
    printf( "\r\t\tInitializing Hand...             " ); fflush(stdout);
    
    if( result=bh.InitHand("") )
        Error();
    
}


void Hand::PrepareRealTime() {
    
    /*
    bh.RTSetGain('1',200);
    bh.RTSetGain('2',200);
    bh.RTSetGain('3',200);
    bh.RTSetGain('4',200);
    printf("AAAAA\n");
    bh.PSet("FPG",1);
    bh.PSet("FDZ",0);
    bh.PSet("FTPG",1);
    bh.PSet("FIP",0);
    //	bh.RTUpdate(true,true);
*/
    option=0;
    printf("Enter option:\n1-spread 180º\n2-grab can\n3-spread grab can\n4-Enter manually\n0-leave it as it is\n");
    scanf("%d",&option);
    printf("option:%d\n",option);
    switch(option){
    case 1:
        //        bh.GoToDifferentPositions(12100,12000,0,1550);
        bh.GoToDifferentPositions(0,0,0,16000);
        printf("went there\n");
        //bh.GoToPosition("1",300000);
        break;
    case 2:
        bh.GoToDifferentPositions(6807,9092,9564,9);
        break;
    case 3:
        bh.GoToDifferentPositions(120000,120000,0,17000);
        break;
    case 4:
        int despos[4];
        for(int i=0;i<4;i++) {
            printf("enter pos nr. %d: ",i+1);
            scanf("%d",&despos[i]);
        }
        
        bh.GoToDifferentPositions(despos[0],despos[1],despos[2],despos[3]);
        break;
        default:
        break;
    }



    sleep(0.3);
    if( result=bh.RTSetFlags( "1234", 1, 1, 0, 0, 1, 0, 1, 0, 1 ))
        Error();


    bh.PSet("LFPPS", 1);



    int ppsprop,lcvprop,res1,res2;
    res1=bh.PGet("LFPPS",&ppsprop);
    res2=bh.PGet("LCVC",&lcvprop);
    printf("PROPERTIES (PPS) : %d %d (LCV) %d %d \n",res1,ppsprop,res2,lcvprop);


}



int Hand::RunRealTime()
{
    double var[4][3];
    int N=0, motor;
    clock_t time, tmstart;
    tmstart = clock();
    double mstime;
    
    int pps[24];
    int z=0;
    
    unsigned char posicao;
    // Start timer
    time = clock() - tmstart;
    mstime = 100.0*(double)time/(double)CLOCKS_PER_SEC;
    printf( "Press Any Key to Abort..." );
    
    bh.RTStart( "1234" );
    //bh.RTStart( "1234", BHMotorTorqueLimitProtect );
    float value;
    
    
    float kinratio[3];
    kinratio[0]=375;
    kinratio[1]=125;
    kinratio[2]=17.5;
    
    
    
    for (int i=0;i<8;i++){
        hjoints.position.push_back(0.0);
        hjoints.velocity.push_back(0.0);
        hjoints.effort.push_back(0.0);
    }
    
    
    printf("\nHere we go\n");
    
    for(int i=0;i<4;i++){
        v[i]=0;
    }
    
    ros::Rate loop_rate(30);
    
    while( ros::ok() )	{
        
        
        for (int motor=0;motor<4;motor++){
            //bh.RTSetTorque(motor+'1',v[motor]);
            bh.RTSetVelocity(motor+'1', v[motor]);
            handarray.data[motor]=(int) bh.RTGetPosition( motor+'1');
            //			hjoints.position[motor]=(int) bh.RTGetPosition( motor+'1');
            hjoints.velocity[motor]=(float) bh.RTGetVelocity(motor+'1');
            
            hpos[motor]=bh.RTGetPosition( motor+'1');

            if(z%20==0) {

                printf("%d (%d) ",hpos[motor], v[motor]);
            }
        }
        //	bh.RTUpdate(true,true);
        

        if(z%20==0) printf("\n");


   /*     if(z%20==0) {



            bh.RTGetPPS('3',pps,MAX_PPS_ELEMENTS);

            bh.RTUpdate(true,true);

           printf("PPS3: ");
            for(int i=0;i<MAX_PPS_ELEMENTS;i++){
                printf("%d ",pps[i]);
            }
            printf("\n");

            bh.RTGetPPS('4',pps,MAX_PPS_ELEMENTS);

            bh.RTUpdate(true,true);

            printf("PPS4: ");
            for(int i=0;i<20;i++){
                printf("%d ",pps[i]);
            }
            printf("\n");

        }
*/
        z++;

        hjoints.position[0]=(float)handarray.data[0]/kinratio[0]*PI/180;
        hjoints.position[1]=(float)handarray.data[0]/kinratio[1]*PI/180;
        hjoints.position[2]=(float)handarray.data[3]/kinratio[2]*PI/180;
        hjoints.position[3]=(float)handarray.data[3]/kinratio[2]*PI/180;
        hjoints.position[4]=(float)handarray.data[2]/kinratio[0]*PI/180;
        hjoints.position[5]=(float)handarray.data[2]/kinratio[1]*PI/180;
        hjoints.position[6]=(float)handarray.data[1]/kinratio[1]*PI/180;
        hjoints.position[7]=(float)handarray.data[1]/kinratio[0]*PI/180;




        hjoints.header.stamp=ros::Time::now();
        hjoints_pub.publish(hjoints);


        fencoder_pub.publish(handarray);

        bh.RTUpdate(true,true);


        ros::spinOnce();
        loop_rate.sleep();
    }


    bh.RTAbort();


    return 0;
}



bool Hand::reset_srv(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response){
    
    
    double pos[4];
    bool cont=true,cont2=true;
    printf("Resetting position\n");
    while(cont && cont2){
        for (int i=0;i<4;i++){
            pos[i]=bh.RTGetPosition(i+'1');
            bh.RTSetVelocity(i+'1',(100000-pos[i])/100);
        }
        if(fabs(pos[0]-100000)<100 ){
            bh.RTSetVelocity('1',0);
            cont=false;
        }
        if( fabs(pos[1]-100000)<100) {
            bh.RTSetVelocity('2',0);
            cont2=false;
        }
        bh.RTUpdate(true,true);
    }
    
    //bh.RTSetPosition('1',100000);
    //bh.RTSetPosition('2',100000);
    //    bh.GoToDifferentPositions(150000,150000,0,15000);
    
    
    return true;
}



bool Hand::grip_srv(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    int a=0;
    int moved=0;
    int iposition[4];
    int curposition[4];
    
    printf("Grip Harder on those strings!!!\n");
    
    for (int motor=0;motor<4;motor++){
        iposition[motor]=(int) bh.RTGetPosition( motor+'1');
    }
    
    bh.RTUpdate(true,true);
    
    for (int motor=0;motor<4;motor++){
        curposition[motor]=iposition[motor];
    }
    
    //   bh.RTSetTorque('1',1000);
    //   bh.RTSetTorque('2',1000);
    
    while(a<100 && moved==0){
        printf("a:%d\n",a);
        //    bh.RTSetTorque('1',a*250);
        //        bh.RTSetTorque('2',a*250);
        bh.RTSetVelocity('1',a*2);
        bh.RTSetVelocity('2',a*2);
        
        
        for (int motor=0;motor<4;motor++){
            //  printf("%d %d\n",iposition[motor],curposition[motor]);
            
            if(fabs(curposition[motor]-iposition[motor]) > 10){
                moved=1;
                //bh.RTSetTorque('1',0);
                //bh.RTSetTorque('2',0);
                bh.RTSetVelocity('1',0);
                bh.RTSetVelocity('2',0);
            }
            curposition[motor]=(int) bh.RTGetPosition( motor+'1');
        }
        bh.RTUpdate(true,true);
        
        
        
        hjoints_pub.publish(hjoints);
        //bh.RTUpdate(true,true);
        
        
        
        a++;
    }
    return true;
}



bool Hand::rels_srv(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    int a=0;
    int moved=0;
    int iposition[4];
    int curposition[4];
    
    printf("Unleash the finger!!!\n");
    
    for (int motor=0;motor<4;motor++){
        iposition[motor]=(int) bh.RTGetPosition( motor+'1');
    }
    
    bh.RTUpdate(true,true);
    
    for (int motor=0;motor<4;motor++){
        curposition[motor]=iposition[motor];
    }
    
    
    while(a<100 && moved==0){
        printf("a:%d\n",a);
        bh.RTSetVelocity('1',-a*2);
        bh.RTSetVelocity('2',-a*2);
        for (int motor=0;motor<4;motor++){
            printf("%d %d\n",iposition[motor],curposition[motor]);
            
            if(fabs(curposition[motor]-iposition[motor]) > 10){
                moved=1;
                //bh.RTSetTorque('1',0);
                //bh.RTSetTorque('2',0);
                bh.RTSetVelocity('1',0);
                bh.RTSetVelocity('2',0);
                
            }
            curposition[motor]=(int) bh.RTGetPosition( motor+'1');
        }
        bh.RTUpdate(true,true);
        
        
        hjoints_pub.publish(hjoints);
        //bh.RTUpdate(true,true);
        
        
        a++;
    }
    return true;
}


void Hand::vUpdate(const std_msgs::Float32MultiArray::ConstPtr& msg){
    
    for(int i=0;i<4;i++){
        v[i]=(int) floor(msg->data[i]);
    }
    
    
    v[2]=0;
    v[3]=0;
}




int main(int argc,char *argv[]){
    
    ros::init(argc, argv, "hand");
    ros::NodeHandle n;
    //	ros::Rate loop_rate(20);
    Hand hand;
    
    
    hand.sub2 = hand.n.subscribe("hcontrol", 1000, &Hand::vUpdate,&hand);
    hand.srv = hand.n.advertiseService("grip",&Hand::grip_srv,&hand);
    hand.srv2 = hand.n.advertiseService("release",&Hand::rels_srv,&hand);
    hand.srv3 = hand.n.advertiseService("resetpos",&Hand::reset_srv,&hand);
    
    printf( "Initialising...\n" );
    hand.Initialize();
    printf( " Done\n" );
    
    hand.PrepareRealTime();
    printf( "RealTime Loop - " );
    if( hand.RunRealTime() )
    {
        printf("Interrupted\n");
        return 1;
    }
    printf( " Done without interruption\n" );
    
    exit(0);
    return 0;
    
}



/*
BHand bh;
int result;
int a=0;
int RunRealTime(ros::NodeHandle n);


void fallback(int){

        if(a==0){
                bh.GoToDifferentPositions(6807,9092,9564,9); //12297 //12305,12240,0,1550
                a=1;
        }
        else{
                ros::NodeHandle n("handle");
                
                RunRealTime(n);
                a=0;
        }
        
}

void exiter(int){
        bh.Command("T");
        bh.RTAbort();
        exit(1);
}








int  main(int argc,char *argv[]){


        ros::init(argc, argv, "hand");
        ros::NodeHandle n("handle");
        ros::ServiceServer srv = n.advertiseService
                        <std_srvs::Empty::Request, std_srvs::Empty::Response>("handoff",handoff_srv);
                        
                        
        printf( "Initialisation..." );
        Initialize();
        printf( " Done\n" );
        
        PrepareRealTime();
        printf( "RealTime Loop - " );
        if( RunRealTime(n) )
        {
                printf("Interrupted\n");
                return 1;
        }
        printf( " Done without interruption\n" );
        return 0;
}

 */
