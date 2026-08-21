#ifndef ROBOT_BEHAVIOR_HPP
#define ROBOT_BEHAVIOR_HPP

#include "decision_pkg_cpp/decision_node.hpp"
#include "decision_pkg_cpp/attributes.h"
#include "decision_pkg_cpp/utils.h"
#include "decision_pkg_cpp/AssyncTimer.hpp"

// Fixo em tempo de compilação (não é o parâmetro ROS2 "robot_number" runtime
// de decision_node.cpp, que hoje não é usado aqui). is_goalkeeper()==true só
// pra 1 — ver robot_behavior.cpp is_goalkeeper/is_kicker/is_bala. Setado como
// 1 (goleiro) neste checkout; se este mesmo repositório/binário for buildado
// também pro kicker/bala, ESSE define precisa voltar pra 2/3 lá, senão os
// outros robôs vão se identificar como goleiro.
#define ROBOT_NUMBER 1

using namespace std::chrono_literals;

class RobotBehavior : public DecisionNode
{
    public:
        void players_behavior();
        void normal_game();
        void normal_game_prepair();
        void player_normal_game();
        void goalkeeper_normal_game(); // FSM portada de GoleiroBehavior::goleiro_normal_game() (goleiro_decision/src/goleiro_behavior.cpp)
        void bala_normal_game();
        void kicker_normal_game();
        bool is_goalkeeper(int robot_num); // feito
        bool is_bala(int robot_num);
        bool is_kicker(int robot_num);
        void kicker_localization_game();
        void bala_localization_game();
        bool ball_is_locked();
        bool goalpost_is_locked(); 
        bool vision_stable();
        bool ball_in_camera_center();
        bool ball_in_robot_limits();
        bool ball_in_left_limit();
        bool ball_in_right_limit();
        bool ball_in_close_limit();
        bool ball_in_right_foot();
        bool ball_in_left_foot();
        bool robot_align_with_the_ball();
        bool goalkeeper_align_with_the_ball(); // feito, precisa testar
        bool robot_align_for_kick_right();
        bool robot_align_for_kick_left();
        bool centered_neck();
        bool full_centered_neck();
        void detect_ball_position();
        bool neck_to_left();
        bool neck_to_right();
        void turn_to_ball();

        void penalty();
        void player_penalty();
        void goalkeeper_penalty(); // fazer

        // --- estados internos de goalkeeper_normal_game(), portados de
        // goleiro_decision/src/goleiro_behavior.cpp (GoleiroBehavior). Usam
        // robot.gk_state (GoalkeeperState) em vez de robot.state, que
        // continua exclusivo de bala/kicker. Nomes mantidos próximos aos
        // originais (Posicionamento/VerQueda/FindBall2/search_and_align_ball_M19)
        // para facilitar comparação com o arquivo de origem.
        void goalkeeper_positioning_logic();     // Posicionamento()
        void goalkeeper_decide_fall();           // VerQueda()
        void goalkeeper_find_ball_after_fall();  // FindBall2()
        bool goalkeeper_search_and_align_ball_M19();
        bool goalkeeper_ball_centered();
        bool goalkeeper_ball_is_locked();

        RobotBehavior();
        virtual ~RobotBehavior();

    private:

        AssyncTimer lost_ball_timer;
        AssyncTimer look_right_timer;
        AssyncTimer check_goalpost_timer;
        bool is_penalized();
        void get_up();

        rclcpp::TimerBase::SharedPtr robot_behavior_;

        // --- limiares do goleiro (parâmetros ROS2, mesmos nomes/defaults de
        // goleiro_behavior.cpp — AINDA NÃO calibrados em campo, ver TODOs lá) ---
        int DangerArea;   // motor 20: abaixo disso, para de só acompanhar e chama goalkeeper_positioning_logic()
        int AnguloQueda;  // motor 19/20: abaixo disso, aciona goalkeeper_decide_fall()
        int XXXX_1;       // TODO: valor indefinido (motor 19) - Posicionamento, caso "andar p/ esquerda"
        int XXXX_2;       // TODO: valor indefinido (motor 19) - Posicionamento, caso "andar p/ direita"
        int XXXX_3;       // TODO: valor indefinido (motor 19) - goalkeeper_decide_fall, caso 2: <= cai p/ direita, > XXXX_3+XXXX_4 cai p/ esquerda
        int XXXX_4;       // TODO: valor indefinido - usado em dois lugares: (1) goalkeeper_find_ball_after_fall,
                          // margem somada a DangerArea (motor 20); (2) goalkeeper_decide_fall, largura da zona
                          // intermediária somada a XXXX_3 (motor 19) — entre XXXX_3 e XXXX_3+XXXX_4 agacha no meio

        // --- controle da sequência de queda/agachamento (goalkeeper_decide_fall) ---
        rclcpp::Time gk_queda_wait_start_;                     // marca o instante em que caiu/agachou, p/ contar os 7s
        Move gk_queda_get_up_move_ = stand_up_front;           // qual move de levantar usar depois dos 7s

        // --- varredura em zigue-zague (goalkeeper_search_and_align_ball_M19) ---
        // Reaproveita NECK_LEFT_LIMIT/NECK_RIGHT_LIMIT (DecisionNode) como
        // limites de inversão, mesmos valores usados por goleiro_behavior.cpp.
        bool gk_search_going_left_ = true; // sentido atual da varredura (true = indo p/ esquerda)
};

#endif // ROBOT_BEHAVIOR_H