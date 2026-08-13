#ifndef GOLEIRO_BEHAVIOR_HPP
#define GOLEIRO_BEHAVIOR_HPP

// ============================================================================
// goleiro_behavior.hpp
//
// Estrutura de decisão / localização do robô goleiro (RoboFEI-HT).
// Segue o mesmo padrão de nó/estados usado em decision_pkg_cpp
// (robot_behavior.cpp + decision_node.cpp), mas como um nó independente,
// dedicado ao raciocínio do goleiro.
//
// Publishers do "pacote de posição" (agora dentro deste mesmo pacote
// goleiro_decision, ver src/localization_node.cpp, antigo pacote "posicao"):
// o localization_node assina /l_count, /t_count, /x_count e /goalpost_count
// (publicados frame a frame pelo vision_pkg / detect.py), filtra e consolida
// esses valores e publica, a cada 60 frames, um resumo no tópico
// "robot_behavior" usando custom_interfaces::msg::LandmarkCount (campos
// l_count, t_count, x_count, goalpost_count). É esse tópico consolidado que
// a goleiro_behavior assina abaixo — e não os /l_count e /x_count crus —
// para já se beneficiar do filtro de mediana feito lá.
//
// Reasoning implementado (fluxograma "Goleiro" no Miro):
//   1) searching_ball até a bola ser fixada (ball_is_locked)
//   2) enquanto neck_pos.position20 >= DangerArea -> apenas seguir a bola
//      com o motor 20
//   3) quando neck_pos.position20 <= DangerArea -> chama Posicionamento()
//        3.1) se position20 >= AnguloQueda:
//             a) vendo 1 X e (2 L ou 0 L): alinhar com a bola, aproximar até
//                position19 <= AnguloQueda -> VerQueda()
//             b) vendo 0 X e 1 L e position19 <= XXXX: andar p/ esquerda até
//                position19 <= AnguloQueda -> VerQueda()
//             c) vendo 0 X e 1 L e position19 >= XXXX: andar p/ direita até
//                position19 <= AnguloQueda -> VerQueda()
//
// IMPORTANTE sobre os placeholders:
//   - DangerArea, AnguloQueda, XXXX_1, XXXX_2, XXXX_3 e XXXX_4 são limiares
//     (motor 19/20) que ainda não foram calibrados em campo. Foram deixados
//     como parâmetros ROS2 com valores de exemplo (NÃO são valores reais/
//     calibrados) para que o time ajuste sem precisar recompilar.
//   - Cada XXXX_N representa um uso distinto de "XXXX" no racional original
//     — são mantidos como constantes separadas (podem ou não ter o mesmo
//     valor final; isso cabe à calibração do time).
//   - g_dive_left / g_dive_right e g_stand_up_side_left / g_stand_up_side_right
//     foram confirmados em RoboFEI/src/control/src/control.cpp
//     (Control::choose_movement) e em RoboFEI/src/control/Data/motion*.json:
//     action_number 11 = "Goalkeeper Fall Left", 12 = "Goalkeeper Fall Right",
//     18 = "Fallen Side Left", 19 = "Fallen Side Right" (attributes.h do
//     decision_pkg_cpp só cobria os moves de jogador de campo, por isso não
//     tinha 11/12/19). Atenção: nos 5 motion*.json do repositório, as seções
//     "Goalkeeper Fall Left"/"Goalkeeper Fall Right" só declaram
//     "number of movements" (6), sem os address/position de cada passo —
//     ou seja, o action_number já existe e está roteado no control.cpp, mas
//     a animação de queda em si ainda não foi calibrada/gravada em nenhuma
//     config; falta o time preencher esses passos antes de usar em campo.
// ============================================================================

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "custom_interfaces/msg/humanoid_league_msgs.hpp"
#include "custom_interfaces/msg/joint_state.hpp"
#include "custom_interfaces/msg/vision.hpp"
#include "custom_interfaces/msg/landmark_count.hpp" // publicado pelo localization_node (mesmo pacote) em "robot_behavior"
#include "custom_interfaces/action/control.hpp"

using namespace std::chrono_literals;

// ---------------------------------------------------------------------------
// Enums (mesmo estilo de attributes.h do decision_pkg_cpp)
// ---------------------------------------------------------------------------
enum GoleiroState
{
    goleiro_searching_ball  = 1,
    goleiro_tracking_ball   = 2, // seguindo a bola só com o motor 20
    goleiro_positioning     = 3, // dentro da função Posicionamento()
    goleiro_ver_queda       = 4, // decide e dispara a queda/agachamento (VerQueda)
    goleiro_queda_esperando = 5, // já caiu/agachou, esperando os 7s no chão
    goleiro_queda_levantando = 6, // comando de levantar enviado, esperando finished_move
    goleiro_finding_ball_2  = 7  // de pé de novo, dentro da função FindBall2()
};

// Subconjunto do enum Move usado em attributes.h (decision_pkg_cpp), com os
// action_number conferidos contra o switch real de
// RoboFEI/src/control/src/control.cpp (Control::choose_movement).
enum GoleiroMove
{
    g_stand_still    = 1,
    g_turn_right     = 5,
    g_turn_left      = 6,
    g_squat          = 13, // "Goalkeeper Middle" (control.cpp case 13) — bloqueio central
    g_walk           = 14,
    g_gait           = 15,
    g_stand_up_back  = 16,
    g_stand_up_front = 17,
    g_walk_left      = 20,
    g_walk_right     = 21,

    // Queda lateral do goleiro e respectivas voltas ao normal — action_number
    // conferidos em control.cpp (case 11/12/18/19) e nos motion*.json
    // (RoboFEI/src/control/Data): 11/12 ainda estão sem os passos de
    // movimento gravados em nenhuma config (ver nota no cabeçalho do .hpp).
    g_dive_left           = 11, // "Goalkeeper Fall Left"  (control.cpp case 11)
    g_dive_right          = 12, // "Goalkeeper Fall Right" (control.cpp case 12)
    g_stand_up_side_left  = 18, // "Fallen Side Left"      (control.cpp case 18)
    g_stand_up_side_right = 19  // "Fallen Side Right"     (control.cpp case 19)
};

struct GoleiroNeckPosition
{
    int position19 = 2048; // motor horizontal (pan)
    int position20 = 2048; // motor vertical (tilt)
};

struct GoleiroRobot
{
    GoleiroState state = goleiro_searching_ball;
    GoleiroNeckPosition neck_pos;

    custom_interfaces::msg::Vision ball_info; // /ball_position
    // Vindos do resumo consolidado do localization_node (tópico "robot_behavior",
    // custom_interfaces::msg::LandmarkCount) — não são lidos direto de /x_count, /l_count.
    int x_count = 0;
    int l_count = 0;
    int t_count = 0;
    int goalpost_count = 0;

    GoleiroMove movement = g_stand_still;
    bool finished_move = true;
};

class GoleiroBehavior : public rclcpp::Node
{
public:
    using GameControllerMsg  = custom_interfaces::msg::HumanoidLeagueMsgs;
    using JointStateMsg      = custom_interfaces::msg::JointState;
    using VisionMsg          = custom_interfaces::msg::Vision;
    using LandmarkCountMsg   = custom_interfaces::msg::LandmarkCount; // publicado pelo localization_node
    using ControlActionMsg   = custom_interfaces::action::Control;
    using GoalHandleControl  = rclcpp_action::ClientGoalHandle<ControlActionMsg>;

    GoleiroBehavior();
    virtual ~GoleiroBehavior();

private:
    GoleiroRobot robot;
    GameControllerMsg gc_info;

    // --- ciclo principal -----------------------------------------------------
    void players_behavior();
    void normal_game();
    void goleiro_normal_game();

    // --- raciocínio de decisão (fluxograma) -----------------------------------
    void Posicionamento();
    void VerQueda();
    void FindBall2(); // busca (zigue-zague M19) + realinha pós-queda, ver goleiro_behavior.cpp

    // --- auxiliares ------------------------------------------------------------
    bool ball_is_locked();
    bool ball_centered();
    // Procura a bola girando o pescoço no M19 (pan) e, quando detectada, gira
    // até alinhar ao centro (ball_centered()). Só aí considera a bola travada.
    // Usado tanto em goleiro_searching_ball quanto em FindBall2().
    bool search_and_align_ball_M19();
    void send_goal(const GoleiroMove &move);

    // --- callbacks (publishers do "pacote de posição" / visão) -----------------
    void listener_callback_GC(const GameControllerMsg::SharedPtr msg);
    void listener_callback_neck_pos(const JointStateMsg::SharedPtr msg);
    void listener_callback_ball(const VisionMsg::SharedPtr msg);
    void listener_callback_landmark_summary(const LandmarkCountMsg::SharedPtr msg); // tópico "robot_behavior" (localization_node)

    void goal_response_callback(const GoalHandleControl::SharedPtr &goal_handle);
    void feedback_callback(GoalHandleControl::SharedPtr, const std::shared_ptr<const ControlActionMsg::Feedback> feedback);
    void result_callback(const GoalHandleControl::WrappedResult &result);

    // --- subscribers / publishers -----------------------------------------------
    rclcpp::Subscription<GameControllerMsg>::SharedPtr gc_subscriber_;                 // "gamestate"
    rclcpp::Subscription<JointStateMsg>::SharedPtr neck_position_subscriber_;          // "all_joints_position"
    rclcpp::Subscription<VisionMsg>::SharedPtr ball_subscriber_;                       // "/ball_position"
    rclcpp::Subscription<LandmarkCountMsg>::SharedPtr landmark_summary_subscriber_;    // "robot_behavior" (localization_node)

    rclcpp_action::Client<ControlActionMsg>::SharedPtr action_client_;                 // "control_action"
    rclcpp_action::Client<ControlActionMsg>::SendGoalOptions send_goal_options_;
    GoalHandleControl::SharedPtr goal_handle_;

    rclcpp::TimerBase::SharedPtr behavior_timer_;

    // --- limiares (parâmetros ROS2, calibrar em campo) ---------------------------
    int DangerArea;   // motor 20: abaixo disso, para de só acompanhar e chama Posicionamento()
    int AnguloQueda;  // motor 19/20: abaixo disso, aciona VerQueda()
    int XXXX_1;       // TODO: valor indefinido (motor 19) - Posicionamento, caso "andar p/ esquerda"
    int XXXX_2;       // TODO: valor indefinido (motor 19) - Posicionamento, caso "andar p/ direita"
    int XXXX_3;       // TODO: valor indefinido (motor 19) - VerQueda, caso 2: >= cai p/ direita, < cai p/ esquerda
    int XXXX_4;       // TODO: valor indefinido (motor 20) - FindBall2, margem somada a DangerArea

    // --- controle da sequência de queda/agachamento (VerQueda) -------------------
    rclcpp::Time queda_wait_start_;     // marca o instante em que caiu/agachou, p/ contar os 7s
    GoleiroMove  queda_get_up_move_ = g_stand_up_front; // qual move de levantar usar depois dos 7s

    // --- varredura em zigue-zague (search_and_align_ball_M19) --------------------
    // Limites do M19 (pan) onde a varredura inverte de sentido. Valores default
    // iguais ao NECK_LEFT_LIMIT/NECK_RIGHT_LIMIT do decision_pkg_cpp original
    // (mesmos limites físicos do pescoço já calibrados lá) — confirmar se valem
    // para a montagem do goleiro.
    int NeckSearchLeftLimit;
    int NeckSearchRightLimit;
    bool search_going_left_ = true; // sentido atual da varredura (true = indo p/ esquerda)
};

#endif // GOLEIRO_BEHAVIOR_HPP
