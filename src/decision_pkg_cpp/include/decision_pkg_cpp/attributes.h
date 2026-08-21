#ifndef ATTRIBUTES_H
#define ATTRIBUTES_H

#include "custom_interfaces/msg/neck_position.hpp"
#include "custom_interfaces/msg/vision.hpp"
#include "custom_interfaces/msg/humanoid_league_msgs.hpp"
#include "std_msgs/msg/bool.hpp" 
#include "std_msgs/msg/int32.hpp" 
#include "std_msgs/msg/string.hpp" 
#include "custom_interfaces/msg/joint_state.hpp"
#include "custom_interfaces/msg/set_position.hpp"
#include "vision_msgs/msg/point2_d.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp" 

enum State
{
    searching_ball = 1,
    aligning_with_the_ball = 2,
    ball_approach = 3,
    ball_close = 4,
    kick_ball = 5
};

enum FallStatus
{
    NotFallen   = 0,
    FallenFront = 1,
    FallenBack  = 2,
    FallenRight = 3,
    FallenLeft  = 4
};

enum Move
{
    stand_still         = 1,
    greeting            = 2,
    right_kick          = 3,
    left_kick           = 4,
    right_kick_to_right = 31,
    //right_kick_to_left  = 32,
    left_kick_to_left   = 34,
    left_kick_penalty  = 35,
    turn_right          = 5,
    turn_left           = 6,
    goodbye             = 7,
    squat               = 13,
    walk                = 14,
    gait                = 15,
    stand_up_back       = 16,
    stand_up_front      = 17,
    stand_up_side_left  = 18, // "Fallen Side Left"  (control.cpp case 18) — era stand_up_side
    walk_left           = 20,
    walk_right          = 21,
    turn_ball_left      = 9,
    turn_ball_right      = 10,

    // Movimentos do goleiro (control.cpp Control::choose_movement / motion*.json),
    // portados de GoleiroMove em goleiro_decision/src/goleiro_behavior.hpp.
    dive_left            = 11, // "Goalkeeper Fall Left"  (control.cpp case 11)
    dive_right           = 12, // "Goalkeeper Fall Right" (control.cpp case 12)
    stand_up_side_right  = 19  // "Fallen Side Right"     (control.cpp case 19)
};

enum ReadyEtapa
{
    ETAPA_WALK,
    ETAPA_TURN,
    ETAPA_PARAR
};

ReadyEtapa ready_etapa = ETAPA_WALK;

enum RobotBallPosition
{
    left,   // 0
    center, // 1
    right   // 2
};

// Estados da máquina de estados do goleiro, portados de GoleiroState em
// goleiro_decision/src/goleiro_behavior.hpp. Ficam num campo dedicado
// (Robot::gk_state) em vez de reaproveitar o State/robot.state usado por
// bala/kicker — os valores colidiriam (ex: kick_ball=5 e o antigo
// goleiro_queda_esperando=5 seriam o mesmo inteiro).
enum GoalkeeperState
{
    goalkeeper_searching_ball   = 1,
    goalkeeper_tracking_ball    = 2, // seguindo a bola só com o motor 20
    goalkeeper_positioning      = 3, // case goalkeeper_positioning em goalkeeper_normal_game() (Posicionamento())
    goalkeeper_ver_queda        = 4, // case goalkeeper_ver_queda: decide e dispara a queda/agachamento (VerQueda())
    goalkeeper_queda_esperando  = 5, // já caiu/agachou, esperando os 7s no chão
    goalkeeper_queda_levantando = 6, // comando de levantar enviado, esperando finished_move
    goalkeeper_finding_ball_2   = 7  // de pé de novo, case goalkeeper_finding_ball_2 (FindBall2())
};

struct NeckPosition
{
    int position19;
    int position20;
};


struct Robot
{
    FallStatus fall = NotFallen;
    Move movement = stand_still;

    bool finished_move = true;
    State state = searching_ball;
    NeckPosition neck_pos;
    custom_interfaces::msg::Vision camera_ball_position;
    RobotBallPosition ball_position = center;
    std_msgs::msg::Bool localization_msg;
    std_msgs::msg::Int32 goalpost_count;
    custom_interfaces::msg::Vision goalpost_division_lines;
    vision_msgs::msg::Point2D goalpost_px_position;
    geometry_msgs::msg::Vector3Stamped imu_gyro;
    float imu_yaw_rad = 0.0;
    float imu_mag = 0.0;

    // --- goleiro (goalkeeper_normal_game, portado de goleiro_behavior.cpp) ---
    GoalkeeperState gk_state = goalkeeper_searching_ball;
    // Resumo de landmarks do tópico "robot_behavior" (localization_node,
    // pacote goleiro_decision, custom_interfaces::msg::LandmarkCount) —
    // usados nos cases goalkeeper_positioning/goalkeeper_ver_queda dentro
    // de goalkeeper_normal_game().
    int l_count = 0;
    int t_count = 0;
    int x_count = 0;
};

#endif // ATTRIBUTES_H