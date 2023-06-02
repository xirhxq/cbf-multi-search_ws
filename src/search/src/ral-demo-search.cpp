#include "Utils.h"
#include "MyDataFun.h"
#include "MyMathFun.h"
#include "FlightControl.hpp"
#include "DataLogger.hpp"
#define CAMERA_ANGLE 30
// #define ON_GROUND_TEST

using namespace dji_osdk_ros;
using namespace std;

typedef enum { TAKEOFF, ASCEND, SEARCH, HOLD, BACK, LAND } ControlState;
ControlState task_state;
vector<geometry_msgs::Vector3> search_tra;
size_t search_tra_cnt;
double hold_begin_time, ascend_begin_time;
DataLogger dl("search.csv");


void toStepTakeoff(){
    task_state = TAKEOFF;
}

void toStepAscend(){
    ascend_begin_time = get_time_now();
    task_state = ASCEND;
}

void toStepSearch(){
    search_tra_cnt = 0;
    task_state = SEARCH;
}

void toStepHold(){
    task_state = HOLD;
    hold_begin_time = get_time_now();
}

void toStepLand(){
    task_state = LAND;
}

void StepTakeoff() {
    ROS_INFO("###----StepTakeoff----###");
    double expected_height = 1.4;
    ROS_INFO("Expected height @ %.2lf", expected_height);
    M210_position_yaw_rate_ctrl(0, 0, expected_height, 0);
    if (MyMathFun::nearly_is(current_pos_raw.z, expected_height, 0.2)){
        // ROS_INFO("Arrive expected height @ %.2lf", expected_height);
        toStepSearch();
        // toStepAscend();
        // toStepHold();
    }
}

void StepAscend(){
    ROS_INFO("###----StepAscend----###");
    double ascend_time = 3.0;
    ROS_INFO("Ascending by 0.1m/s: %.2lf", get_time_now() - ascend_begin_time);
    M210_hold_ctrl(0.1);
    if (enough_time_after(ascend_begin_time, ascend_time)){
        toStepHold();
    }
}

void StepSearch() {
    ROS_INFO("###----StepSearch(Ground)----###");
    double tol = 0.2;
    geometry_msgs::Vector3 desired_point;
    MyDataFun::set_value(desired_point, search_tra[search_tra_cnt]);
    ROS_INFO("Go to %s(%ld th)", MyDataFun::output_str(desired_point).c_str(), search_tra_cnt);
    UAV_Control_to_Point_with_yaw(desired_point, yaw_offset);
    if (is_near(desired_point, tol)){
        search_tra_cnt++;
        if (search_tra_cnt == search_tra.size()){
            toStepHold();
        }
    }
}

void StepHold() {
    ROS_INFO("###----StepHold----###");
    double hold_time = 6.0;
    ROS_INFO("Hold %.2lf", get_time_now() - hold_begin_time);
    // M210_hold_ctrl(0.0);
    M210_adjust_yaw(yaw_offset);
    if (enough_time_after(hold_begin_time, hold_time)){
        toStepLand();
    }
}

void StepBack() {
    ROS_INFO("###----StepBack----###");
}

void StepLand() {
    ROS_INFO("###----StepLand----###");
    ROS_INFO("Landing...");
    takeoff_land(dji_osdk_ros::DroneTaskControl::Request::TASK_LAND);
    // task_state = BACK;
}

void ControlStateMachine() {
    switch (task_state) {
        case TAKEOFF: {
            StepTakeoff();
            break;
        }
        case SEARCH: {
            StepSearch();
            break;
        }
        case HOLD: {
            StepHold();
            break;
        }
        case BACK: {
            StepBack();
            break;
        }
        case LAND: {
            StepLand();
            break;
        }
        default: {
            StepHold();
            break;
        }
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "mbzirc_demo_search", ros::init_options::AnonymousName);
    ros::NodeHandle nh;

    std::vector<std::pair<std::string, std::string> > vn = {
        {"ros_time", "double"},
        {"state", "enum"},
        {"pos", "Point"},
    };
    dl.initialize(vn);
    
	string uav_name = "none";
	if (argc > 1) {
		uav_name = std::string(argv[1]);
	}
    while (uav_name == "none"){
        ROS_ERROR("Invalid vehicle name: %s", uav_name.c_str());
    }
	ROS_INFO("Vehicle name: %s", uav_name.c_str());

    ros::Subscriber attitudeSub =
        nh.subscribe(uav_name + "/dji_osdk_ros/attitude", 10, &attitude_callback);
    ros::Subscriber gimbal_sub =
        nh.subscribe(uav_name + "/dji_osdk_ros/gimbal_angle", 10, &gimbal_callback);
    ros::Subscriber height_sub =
        nh.subscribe(uav_name + "/dji_osdk_ros/height_above_takeoff", 10, &height_callback);
    ros::Subscriber vo_pos_sub =
        nh.subscribe(uav_name + "/dji_osdk_ros/vo_position", 10, &vo_pos_callback);
    ros::Subscriber range_pos_sub = 
        nh.subscribe(uav_name + "/filter/odom", 10, &range_pos_callback);
	ros::Subscriber flightStatusSub =
			nh.subscribe(uav_name + "/dji_osdk_ros/flight_status", 10, &flight_status_callback);
	ros::Subscriber displayModeSub =
			nh.subscribe(uav_name + "/dji_osdk_ros/display_mode", 10, &display_mode_callback);
    ros::Subscriber cmd_sub = 
            nh.subscribe(uav_name + "/commander_cmd", 10, &cmd_callback);
    gimbal_angle_cmd_pub =
        nh.advertise<geometry_msgs::Vector3>(uav_name + "/gimbal_angle_cmd", 10);
    gimbal_speed_cmd_pub =
        nh.advertise<geometry_msgs::Vector3>(uav_name + "/gimbal_speed_cmd", 10);
    ros::Subscriber local_pos_sub = nh.subscribe(uav_name + "/dji_osdk_ros/local_position", 10, &local_pos_callback);

    ctrl_cmd_pub = nh.advertise<sensor_msgs::Joy>(
        uav_name + "/dji_osdk_ros/flight_control_setpoint_generic", 10);
    sdk_ctrl_authority_service =
        nh.serviceClient<dji_osdk_ros::SDKControlAuthority>(
            uav_name + "/dji_osdk_ros/sdk_control_authority");
    drone_task_service = nh.serviceClient<dji_osdk_ros::DroneTaskControl>(
        uav_name + "/dji_osdk_ros/drone_task_control");
    set_local_pos_reference = nh.serviceClient<dji_osdk_ros::SetLocalPosRef> (uav_name + "/dji_osdk_ros/set_local_pos_ref");

    ros::Rate rate(50);
    for (int i = 0; i < 100; i++) {
        ros::spinOnce();
        rate.sleep();
    }


    yaw_offset = current_euler_angle.z;
    ROS_INFO("Yaw offset: %.2lf", yaw_offset * RAD2DEG_COE);
    for (int i = 0; i < 100; i++) {
        ros::spinOnce();
        rate.sleep();
    }
    MyDataFun::set_value(position_offset, current_pos_raw);
    ROS_INFO("Position offset: %s", MyDataFun::output_str(position_offset).c_str());
   
    double y_dir;
    if (uav_name == "suav_1") y_dir = -1.0;
    else if (uav_name == "suav_2") y_dir = 1.0; 
    search_tra.push_back(compensate_offset(MyDataFun::new_point(1, 0.0, 2.0)));
    search_tra.push_back(compensate_offset(MyDataFun::new_point(1, y_dir, 2.0)));
    search_tra.push_back(compensate_offset(MyDataFun::new_point(0, y_dir, 2.0)));
    search_tra.push_back(compensate_offset(MyDataFun::new_point(0.0, 0.0, 2.0)));

    ROS_INFO("Search Trajectory:");
    for (auto a: search_tra){
        ROS_INFO("%s", MyDataFun::output_str(a).c_str());
    }
#ifdef GPS_HEIGHT
    ROS_INFO("Use GPS for height");
#else
    ROS_INFO("Use supersonic wave for height");
#endif
    string confirm_input;
    while (confirm_input != "yes"){
        ROS_INFO("Confirm: yes/no");
        cin >> confirm_input;
        if (confirm_input == "no"){
            return 0;
        }
    }

    
    // if (!set_local_position()){
    //     ROS_ERROR("GPS health insufficient - No local frame reference for height. Exiting.");
    //     return 1;
    // }
    

    ROS_INFO("Waiting for command to take off...");
    sleep(3);
    // while(cmd != "ok"){
    //     ros::spinOnce();
    //     rate.sleep();
    // }
    #ifndef ON_GROUND_TEST
    obtain_control();
    monitoredTakeoff();
    #endif


    ROS_INFO("Start Control State Machine...");
    toStepTakeoff();

    while (ros::ok()) {
        // std::cout << "\033c" << std::flush;
        ROS_INFO("-----------");
        ROS_INFO("M210(State: %d) @ %s", task_state, MyDataFun::output_str(current_pos_raw).c_str());
        // ROS_INFO("Gimbal %s", MyDataFun::output_str(current_gimbal_angle).c_str());
        ROS_INFO("Attitude (R%.2lf, P%.2lf, Y%.2lf) / deg", current_euler_angle.x * RAD2DEG_COE,
                                                    current_euler_angle.y * RAD2DEG_COE,
                                                    current_euler_angle.z * RAD2DEG_COE);
    #ifndef ON_GROUND_TEST
        if (EMERGENCY){
            M210_hold_ctrl();
            printf("!!!!!!!!!!!!EMERGENCY!!!!!!!!!!!!\n");
        }
        else {
            ControlStateMachine();
        }
    #endif

        dl.log("ros_time", get_time_now());
        dl.log("state", task_state);
        dl.log("pos", current_pos_raw);
        dl.newline();
        
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
