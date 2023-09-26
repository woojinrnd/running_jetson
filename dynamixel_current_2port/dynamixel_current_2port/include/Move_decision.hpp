#ifndef MOVE_DECISION_H
#define MOVE_DECISION_H

#include <eigen3/Eigen/Dense>
#include <vector>
#include <iostream>
#include <ros/ros.h>
#include <boost/thread.hpp>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Imu.h>
#include <opencv2/opencv.hpp>
#include <string.h>
#include <tf/tf.h>

#include "dynamixel_current_2port/SendMotion.h"

#include "img_proc.hpp"

#define UD_MAX 90
#define UD_MIN 0
#define UD_CENTER 50

#define RL_MAX 90
#define RL_MIN -90
#define RL_CENTER 0

#define TURN_MAX 30
#define TURN_MIN -30

#define LINE_TURN 10

using namespace std;

class Move_Decision
{
public:
    enum Motion_Index
    {
        InitPose = 0,
        Forward_4step = 1,
        Left_2step = 2,
        Step_in_place = 3,
        Right_2step = 4,
        Forward_Nstep = 5,
        Huddle_Jump = 6,
        ForWard_fast4step = 7,
        FWD_UP = 8,
        BWD_UP = 9,
        Forward_Halfstep = 10,
        Left_Halfstep = 11,
        Right_Halfstep = 12,
        Back_Halfstep = 13,
        NONE = 99,
    };

    enum Running_Mode
    {
        LINE_MODE = 0,
        NO_LINE_MODE = 1,
        STOP_MODE = 2,
        WAKEUP_MODE = 3,
        GOAL_MODE = 4,
        HUDDLE_MODE = 5,
        WALL_MODE = 6,
        CORNER_MODE = 7,
    };

    enum Stand_Status
    {
        Stand = 0,
        Fallen_Forward = 1,
        Fallen_Back = 2,
    };

    string Str_InitPose = "InitPose";
    string Str_Forward_4step = "Forward_4step";
    string Str_Left_2step = "Left_2step";
    string Str_Step_in_place = "Step_in_place";
    string Str_Right_2step = "Right_2step";
    string Str_ForWard_fast4step = "ForWard_fast4step";
    string Str_Forward_Nstep = "Forward_Nstep";
    string Str_Huddle_Jump = "Huddle_Jump";
    string Str_Forward_Halfstep = "Forward_Halfstep";
    string Str_Left_Halfstep = "Left_Halfstep";
    string Str_Right_Halfstep = "Right_Halfstep";
    string Str_Back_Halfstep = "Back_Halfstep";
    string Str_FWD_UP = "FWD_UP";
    string Str_BWD_UP = "BWD_UP";
    string Str_NONE = "NONE";

    string Str_LINE_MODE = "LINE_MODE";
    string Str_NO_LINE_MODE = "NO_LINE_MODE";
    string Str_STOP_MODE = "STOP_MODE";
    string Str_WAKEUP_MODE = "WAKEUP_MODE";
    string Str_GOAL_MODE = "GOAL_MODE";
    string Str_HUDDLE_MODE = "HUDDLE_MODE";
    string Str_WALL_MODE = "WALL_MODE";
    string Str_CORNER_MODE = "CORNER_MODE";

    // Constructor
    Move_Decision(Img_proc *img_procPtr);
    Img_proc *img_procPtr;

    // Destructor
    ~Move_Decision();

    // ********************************************** PROCESS THREAD************************************************** //

    void process();
    void processThread();
    void LINE_mode();
    void NOLINE_mode();
    void STOP_mode();
    void WAKEUP_mode();
    void GOAL_LINE_mode();
    void HUDDLE_mode();
    void WALL_mode();
    void CORNER_mode();
    void CORNER_mode_debug();
    void Test_service();

    bool tmp_img_proc_line_det_flg_ = false;
    bool tmp_img_proc_no_line_det_flg_ = false;
    bool tmp_img_proc_huddle_det_flg_ = false;
    bool tmp_img_proc_wall_det_flg_ = false;
    bool tmp_img_proc_goal_det_flg_ = false;
    bool tmp_img_proc_corner_det_flg_ = false;

    // ********************************************** CALLBACK THREAD ************************************************** //

    void Running_Mode_Decision();
    void callbackThread();
    void startMode();

    bool SendMotion(dynamixel_current_2port::SendMotion::Request &req, dynamixel_current_2port::SendMotion::Response &res);

    std::tuple<int8_t, double> playMotion();
    double turn_angle();
    double Move_UD_NeckAngle();
    double Move_RL_NeckAngle();
    bool Emergency();

    // Publish & Subscribe
    // ros::Publisher Emergency_pub_;
    // IMU
    void IMUsensorCallback(const std_msgs::Float32::ConstPtr &IMU);
    ros::Subscriber IMU_sensor_x_subscriber_; ///< Gets IMU Sensor data from Sensor_node
    ros::Subscriber IMU_sensor_y_subscriber_; ///< Gets IMU Sensor data from Sensor_node
    ros::Subscriber IMU_sensor_z_subscriber_; ///< Gets IMU Sensor data from Sensor_node

    bool stop_fallen_check_;
    double present_pitch_;
    double present_roll_;
    Eigen::VectorXd RPY = Eigen::VectorXd::Zero(3); // Roll Pitch Yaw

    // Server && Client
    ros::ServiceServer SendMotion_server_;

    // ********************************************** FUNCTION ************************************************** //

    void Motion_Info();
    void Running_Info();

    // ********************************************** GETTERS ************************************************** //

    bool Get_Emergency_() const;
    int8_t Get_motion_index_() const;
    int8_t Get_stand_status_() const;
    int8_t Get_running_mode_() const;
    double Get_turn_angle_() const;
    double Get_distance_() const;

    bool Get_ProcessON() const;
    bool Get_MoveDecisionON() const;
    bool Get_CallbackON() const;

    // RUNNING MODE
    bool Get_goal_line_det_flg() const;
    bool Get_line_det_flg() const;
    bool Get_no_line_det_flg() const;
    bool Get_huddle_det_flg() const;
    bool Get_wall_det_flg() const;
    bool Get_stop_det_flg() const;
    bool Get_corner_det_flg() const;
    bool Get_huddle_det_stop_flg() const;
    bool Get_corner_det_stop_flg() const;

    bool Get_select_motion_on_flg() const;
    bool Get_turn_angle_on_flg() const;
    bool Get_emergency_on_flg() const;
    bool Get_distance_on_flg() const;

    bool Get_response_sent_() const;

    double Get_RL_NeckAngle() const;
    double Get_UD_NeckAngle() const;
    bool Get_RL_Neck_on_flg() const;
    bool Get_UD_Neck_on_flg() const;

    bool Get_SM_req_finish() const;
    bool Get_TA_req_finish() const;
    bool Get_UD_req_finish() const;
    bool Get_RL_req_finish() const;
    bool Get_EM_req_finish() const;

    // ********************************************** SETTERS ************************************************** //

    void Set_Emergency_(bool Emergency);
    void Set_motion_index_(int8_t motion_index);
    void Set_stand_status_(int8_t stand_status);
    void Set_running_mode_(int8_t running_mode);
    void Set_turn_angle_(double turn_angle);
    void Set_distance_(double distance);

    void Set_ProcessON(bool ProcessON);
    void Set_MoveDecisionON(bool MoveDecisionON);
    void Set_CallbackON(bool CallbackON);

    void Set_response_sent_(bool response_sent);

    void Set_line_det_flg(bool line_det_flg);
    void Set_no_line_det_flg(bool no_line_det_flg);
    void Set_goal_line_det_flg(bool goal_line_det_flg);
    void Set_huddle_det_flg(bool huddle_det_flg);
    void Set_wall_det_flg(bool wall_det_flg);
    void Set_stop_det_flg(bool stop_det_flg);
    void Set_corner_det_flg(bool corner_det_flg);
    void Set_huddle_det_stop_flg(bool huddle_det_stop_flg);
    void Set_corner_det_stop_flg(bool corner_det_stop_flg);

    void Set_select_motion_on_flg(bool select_motion_on_flg);
    void Set_turn_angle_on_flg(bool turn_angle_on_flg);
    void Set_emergency_on_flg(bool emergency_on_flg);
    void Set_distance_on_flg(bool distance_on_flg);

    void Set_RL_NeckAngle(double RL_NeckAngle);
    void Set_UD_NeckAngle(double UD_NeckAngle);
    void Set_RL_Neck_on_flg(bool RL_Neck_on_flg);
    void Set_UD_Neck_on_flg(bool UD_Neck_on_flg);

    void Set_SM_req_finish(bool SM_req_finish);
    void Set_TA_req_finish(bool TA_req_finish);
    void Set_UD_req_finish(bool UD_req_finish);
    void Set_RL_req_finish(bool RL_req_finish);
    void Set_EM_req_finish(bool EM_req_finish);

    // ********************************************** IMG_PROC ************************************************** //

    /////////////////////// Line Mode ///////////////////////
    // StraightLine
    bool straightLine;
    double margin_gradient = 20; // margin of straight line
    void StraightLineDecision(double gra, double mg_gra);
    double Angle_toBeStraight = 40; // max or min
    int8_t line_gradient = 0;
    double line_actual_angle = 0;
    int8_t line_motion = 0;
    double line_ud_neckangle = 0;

    /////////////////////// No Line Mode ///////////////////////
    // If no find line (NO_LINE_MODE)
    // delta_x : Center of window.x - Center of last captured line.x
    // delta_x > 0 : LEFT
    // delta_x < 0 : RIGHT
    // Out of Range -> A straight trun walking
    int8_t tmp_delta_x = 0;
    int8_t noline_motion = 0;
    double noline_actual_angle = 0;
    double Angle_ToFindLine = 30; // max or min
    double noline_neckangle = 0;

    // Actural send turn angle
    double Actual_angle = 0;
    double increment = 0;

    /////////////////////// WAKEUP_MODE ///////////////////////
    // WakeUp_seq = 0 : Initial
    // WakeUp_seq = 1 : FWD_UP or BWD_UP
    // WakeUp_seq = 2 : Motion_Index : Initial_pose
    // WakeUp_seq = 3 : Line_mode()
    int8_t WakeUp_seq = 0;
    int8_t tmp_stand_status = 0;
    int8_t wakeup_motion = 0;
    int8_t wakeup_running = 0;

    /////////////////////// Huddle Mode ///////////////////////

    // Huddle Sequence
    // 0 : Motion : InitPose (for Getting distance) (Depth)
    // 1 : Motion : Forward_Nstep (Far)
    // 2 : Motion : InitPose (for Getting distance) (Depth)
    // 3 : Motion : Forward_Nstep (Approach)
    // 4 : Motion : Step in place (Pose Control)
    // 5 : Motion : InitPose
    // 6 : MOtion : Forward_halfstep (Aprroach Huddle)
    // 7 : Motion : Huddle Jump
    // 8 : Initializing
    int8_t tmp_huddle_seq = 0;
    double huddle_distance = 0;
    double huddle_actual_angle = 0;
    int8_t huddle_motion = 0;
    double huddle_ud_neck_angle = 0;
    std::vector<double> huddle_distance_save;
    bool contain_huddle_to_foot = false;
    int8_t to_be_line_mode = 0;

    string Str_HUDDLE_SEQUENCE_0 = "HUDDLE_SEQUENCE_0 : InitPose (for Getting distance) (Depth)";
    string Str_HUDDLE_SEQUENCE_1 = "HUDDLE_SEQUENCE_1 : Forward_Nstep (Far)";
    string Str_HUDDLE_SEQUENCE_2 = "HUDDLE_SEQUENCE_2 : InitPose (for Getting distance) (Depth)";
    string Str_HUDDLE_SEQUENCE_3 = "HUDDLE_SEQUENCE_3 : Forward_Nstep (Approach)";
    string Str_HUDDLE_SEQUENCE_4 = "HUDDLE_SEQUENCE_4 : Step in place (Pose Control)";
    string Str_HUDDLE_SEQUENCE_5 = "HUDDLE_SEQUENCE_5 : InitPose";
    string Str_HUDDLE_SEQUENCE_6 = "HUDDLE_SEQUENCE_6 : Forward_halfstep (Aprroach Huddle)";
    string Str_HUDDLE_SEQUENCE_7 = "HUDDLE_SEQUENCE_7 : Huddle Jump";
    string Str_HUDDLE_SEQUENCE_8 = "HUDDLE_SEQUENCE_8 : Initializing";

    /////////////////////// Corner Mode ///////////////////////

    // Corner Sequence
    // 0 : corner_shape dicision (From img_proc_corner_number) (Depth)
    // 1 : Motion : InitPose (For getting distance) (Depth)
    // 2 : Motion : Forward_Nstep (Far)
    // 3 : Motion : InitPose (For getting distance) (Depth)
    // 4 : Motion : Forward_Nstep (Approach)
    // 5 : Motion : Step in place
    // 6 : Motion : Turn Angle 90(ㅓ) or -90(ㅜ)
    //                       |
    //                       |
    //                       |
    //                      \!/
    // 0 : Approach to the Corner --> Motion : Motion_Index::Forward_Halfstep (Until corner center)
    // 1 : Pose Control (Posture(Gradient))
    // 2 : Motion : Step in place + Turn Angle 90(ㅓ) or -90(ㅜ)
    // 3 : Initializing

    double Angle_ToStartWall = 90;
    bool Turn90 = false;
    int8_t turn_30 = 0;

    // corner shape ㅓ(1) / ㅜ(2)
    int8_t tmp_corner_shape = 0;
    int8_t tmp_corner_seq = 0;
    bool corner_seq_finish = false;

    double corner_actual_angle = 0;
    int8_t tmp_turn90 = 0;

    double corner_distance = 0;
    std::vector<double> corner_distance_save;

    int8_t corner_motion = 0;
    double corner_ud_neck_angle = 0;

    bool img_proc_contain_corner_to_foot = false; // corner Y Point
    int8_t img_proc_corner_delta_x = 0;
    double img_proc_corner_angle = 0;

    bool contain_corner_X = false; // corner X Point
    bool contain_corner_Y = false; // corner Y Point
    bool corner_posture = false;   // corner gradient

    string Str_CORNER_SEQUENCE_0 = "CORNER_SEQUENCE_0 : POSITION CONTROL";
    string Str_CORNER_SEQUENCE_1 = "CORNER_SEQUENCE_1 : POSTURE CONTROL";
    string Str_CORNER_SEQUENCE_2 = "CORNER_SEQUENCE_2 : TURN 90";
    string Str_CORNER_SEQUENCE_3 = "CORNER_SEQUENCE_3 : INITIALIZING";

    /////////////////////// Wall Mode ///////////////////////
    // case 1 : After corner, Starting Wall Mode
    // 0: Motion : InitPose (For getting Distance) -> 1 : Motion : ForwardNstep

    // case 2 : Until Right plane determine
    // 0 : Motion : InitPose (For getting Distance) -> 1 : Motion : ForwardNstep

    int8_t wall_motion = 0;
    int8_t img_wall_number_case = 0;
    int8_t wall_number_seq = 0;
    double wall_neck_angle = 0;
    double wall_distance = 0;
    std::vector<double> wall_distance_save;

    /////////////////////// Sequence++ ///////////////////////
    bool finish_past = false;
    int8_t req_finish_count = 0;

    // check the variable sharing with multi thread
    int aaaa = 0;
    int ccc = 0;
    int abc = 0;
    int b = aaaa % 2;

    int warning_counter = 0;
    bool warning_printed = false;

private:
    ros::NodeHandle nh;
    ros::Publisher pub;

    bool response_sent_ = false;
    std::set<int> processed_requests_;

    void recordProcessedRequest(int request_id)
    {
        processed_requests_.insert(request_id);
    }

    // Check if the request ID has already been processed
    bool isRequestProcessed(int request_id)
    {
        return processed_requests_.find(request_id) != processed_requests_.end();
    }

    const double FALL_FORWARD_LIMIT;
    const double FALL_BACK_LIMIT;
    const int SPIN_RATE;

    int8_t motion_index_ = 99;
    int8_t stand_status_;
    int8_t running_mode_;

    // Body Angle
    // Counter Clock Wise(+)
    // LEFT(+) / RIGHT(-)
    double turn_angle_ = 0;

    // Distance
    double distance_ = 0;

    // Neck
    // Counter Clock Wise(+)
    // LEFT(+) / RIGHT(-)
    double RL_NeckAngle_ = 0;
    bool RL_Neck_on_flg_ = false;
    // Counter Clock Wise(+)
    // UP(+) / DOWN(-)
    double UD_NeckAngle_ = 0;
    bool UD_Neck_on_flg_ = false;

    // Running mode
    bool goal_line_det_flg_ = false;
    bool line_det_flg_ = false;
    bool no_line_det_flg_ = false;
    bool huddle_det_flg_ = false;
    bool stop_det_flg_ = false;
    bool wall_det_flg_ = false;
    bool corner_det_flg_ = false;

    // True : unEnable to write the value
    // False : Enable to write the value
    bool select_motion_on_flg_ = false;
    bool turn_angle_on_flg_ = false;
    bool emergency_on_flg_ = false;
    bool distance_on_flg_ = false;

    bool huddle_det_stop_flg_ = false;
    bool corner_det_stop_flg_ = false;
    int8_t Wall_mode = 0;

    bool Emergency_;

    /// Thread switch ///
    bool ProcessON_;
    bool MoveDecisionON_;
    bool CallbackON_;

    // true -> req.finish is true
    // false -> req.finish is false
    bool SM_req_finish_ = false;
    bool TA_req_finish_ = false;
    bool UD_req_finish_ = false;
    bool RL_req_finish_ = false;
    bool EM_req_finish_ = false;

    // ********************************************** MUTEX ************************************************** //
    mutable std::mutex mtx_goal_line_det_flg;
    mutable std::mutex mtx_line_det_flg;
    mutable std::mutex mtx_no_line_det_flg;
    mutable std::mutex mtx_huddle_det_flg;
    mutable std::mutex mtx_wall_det_flg;
    mutable std::mutex mtx_stop_det_flg;
    mutable std::mutex mtx_corner_det_flg;
    mutable std::mutex mtx_huddle_det_stop_flg;
    mutable std::mutex mtx_corner_det_stop_flg;

    mutable std::mutex mtx_RL_NeckAngle_;
    mutable std::mutex mtx_UD_NeckAngle_;

    mutable std::mutex mtx_RL_Neck_on_flg;
    mutable std::mutex mtx_UD_Neck_on_flg;

    mutable std::mutex mtx_turn_angle_;
    mutable std::mutex mtx_distance_;

    mutable std::mutex mtx_motion_index_;
    mutable std::mutex mtx_stand_status_;
    mutable std::mutex mtx_running_mode_;
    mutable std::mutex mtx_response_sent_;

    mutable std::mutex mtx_select_motion_on_flg_;
    mutable std::mutex mtx_turn_angle_on_flg_;
    mutable std::mutex mtx_emergency_on_flg_;
    mutable std::mutex mtx_distance_on_flg_;

    mutable std::mutex mtx_Emergency_;
    mutable std::mutex mtx_ProcessON_;
    mutable std::mutex mtx_MoveDecisionON_;
    mutable std::mutex mtx_CallbackON_;

    mutable std::mutex mtx_SM_req_finish_;
    mutable std::mutex mtx_TA_req_finish_;
    mutable std::mutex mtx_UD_req_finish_;
    mutable std::mutex mtx_RL_req_finish_;
    mutable std::mutex mtx_EM_req_finish_;
};
#endif // MOVE_DECISION_H