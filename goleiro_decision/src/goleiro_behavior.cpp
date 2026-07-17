// ============================================================================
// goleiro_behavior.cpp
//
// Implementação do nó de decisão/localização do goleiro. Ver goleiro_behavior.hpp
// para o racional completo (fluxograma "Goleiro" do Miro) e para a explicação
// dos placeholders DangerArea / AnguloQueda / XXXX_1 / XXXX_2.
//
// Baseado no padrão de decision_pkg_cpp (robot_behavior.cpp + decision_node.cpp)
// do repositório RoboFEI-HT_2023_SOFTWARE, no publisher "robot_behavior" do
// localization_node (mesmo pacote, custom_interfaces::msg::LandmarkCount)
// e no all_joints_position para a posição dos motores 19 (pan) e 20 (tilt).
// ============================================================================

#include "goleiro_behavior.hpp"

GoleiroBehavior::GoleiroBehavior() : Node("goleiro_behavior_node")
{
    RCLCPP_INFO(this->get_logger(), "Iniciando Goleiro Behavior Node");

    // --- Subscribers: publishers do "pacote de posição" / visão --------------
    gc_subscriber_ = this->create_subscription<GameControllerMsg>(
        "gamestate", rclcpp::QoS(10),
        std::bind(&GoleiroBehavior::listener_callback_GC, this, std::placeholders::_1));

    neck_position_subscriber_ = this->create_subscription<JointStateMsg>(
        "all_joints_position", rclcpp::QoS(10),
        std::bind(&GoleiroBehavior::listener_callback_neck_pos, this, std::placeholders::_1));

    ball_subscriber_ = this->create_subscription<VisionMsg>(
        "/ball_position", rclcpp::QoS(10),
        std::bind(&GoleiroBehavior::listener_callback_ball, this, std::placeholders::_1));

    // Resumo consolidado de landmarks publicado pelo localization_node
    // (mesmo pacote), já filtrado (mediana) a partir de /l_count,
    // /t_count, /x_count e /goalpost_count do vision_pkg.
    landmark_summary_subscriber_ = this->create_subscription<LandmarkCountMsg>(
        "robot_behavior", rclcpp::QoS(10),
        std::bind(&GoleiroBehavior::listener_callback_landmark_summary, this, std::placeholders::_1));

    // --- Action client de movimentação (mesmo do decision_pkg_cpp) -----------
    action_client_ = rclcpp_action::create_client<ControlActionMsg>(this, "control_action");

    send_goal_options_.goal_response_callback =
        std::bind(&GoleiroBehavior::goal_response_callback, this, std::placeholders::_1);
    send_goal_options_.feedback_callback =
        std::bind(&GoleiroBehavior::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options_.result_callback =
        std::bind(&GoleiroBehavior::result_callback, this, std::placeholders::_1);
    goal_handle_ = nullptr;

    // --- Limiares (parâmetros ROS2 - NÃO calibrados, apenas placeholders) -----
    DangerArea  = this->declare_parameter("danger_area", 1500);   // TODO: calibrar em campo
    AnguloQueda = this->declare_parameter("angulo_queda", 1300);  // TODO: calibrar em campo
    XXXX_1      = this->declare_parameter("xxxx_1", 2048);        // TODO: valor indefinido (Posicionamento, caso "esquerda")
    XXXX_2      = this->declare_parameter("xxxx_2", 2048);        // TODO: valor indefinido (Posicionamento, caso "direita")
    XXXX_3      = this->declare_parameter("xxxx_3", 2048);        // TODO: valor indefinido (VerQueda, escolha do lado da queda)
    XXXX_4      = this->declare_parameter("xxxx_4", 0);           // TODO: valor indefinido (FindBall2, margem sobre DangerArea)

    // Mesmos defaults de NECK_LEFT_LIMIT/NECK_RIGHT_LIMIT do decision_pkg_cpp
    // original (decision_node.cpp) — reaproveitados como ponto de partida,
    // confirmar se valem para a montagem do goleiro.
    NeckSearchLeftLimit  = this->declare_parameter("neck_search_left_limit", 2650);
    NeckSearchRightLimit = this->declare_parameter("neck_search_right_limit", 1350);

    behavior_timer_ = this->create_wall_timer(8ms, std::bind(&GoleiroBehavior::players_behavior, this));
}

GoleiroBehavior::~GoleiroBehavior() {}

// ---------------------------------------------------------------------------
void GoleiroBehavior::players_behavior()
{
    normal_game();
}

void GoleiroBehavior::normal_game()
{
    if (gc_info.game_state == GameControllerMsg::GAMESTATE_PLAYING)
    {
        goleiro_normal_game();
    }
    else
    {
        send_goal(g_stand_still);
    }
}

// ---------------------------------------------------------------------------
// Raciocínio principal do goleiro (item 1, 2 e 3 do racional)
// ---------------------------------------------------------------------------
void GoleiroBehavior::goleiro_normal_game()
{
    switch (robot.state)
    {
    case goleiro_searching_ball:
        // 1) Entrar no estado searching_ball até fixar a visão na bola: gira
        // o M19 procurando, alinha ao centro e só então considera travada.
        RCLCPP_DEBUG(this->get_logger(), "Goleiro: searching_ball");
        if (search_and_align_ball_M19())
        {
            robot.state = goleiro_tracking_ball;
        }
        break;

    case goleiro_tracking_ball:
        // 2) Enquanto neck_pos.position20 >= DangerArea, apenas segue a bola
        //    com o motor 20.
        if (!ball_is_locked())
        {
            robot.state = goleiro_searching_ball;
        }
        else if (robot.neck_pos.position20 >= DangerArea)
        {
            RCLCPP_DEBUG(this->get_logger(), "Goleiro: seguindo bola (motor 20) pos20=%d", robot.neck_pos.position20);
            send_goal(g_stand_still); // corpo parado, pescoço acompanha a bola verticalmente
        }
        else
        {
            // 3) neck_pos.position20 <= DangerArea -> chama Posicionamento()
            robot.state = goleiro_positioning;
        }
        break;

    case goleiro_positioning:
        Posicionamento();
        break;

    case goleiro_ver_queda:
        // Decide (uma vez) squat ou queda p/ um dos lados, e já entra em
        // goleiro_queda_esperando dentro de VerQueda().
        VerQueda();
        break;

    case goleiro_queda_esperando:
        // Fica parado (squat/caído) até completar os 7 segundos no chão.
        // OBS: se o move "squat"/queda não for uma postura sustentada (isto é,
        // se o robô volta a ficar de pé sozinho após a animação terminar),
        // vai ser necessário reenviar send_goal(...) aqui a cada ciclo para
        // manter a postura até os 7s — hoje o goal é enviado só uma vez,
        // dentro de VerQueda().
        if ((this->now() - queda_wait_start_).seconds() >= 7.0)
        {
            send_goal(queda_get_up_move_);
            robot.state = goleiro_queda_levantando;
        }
        break;

    case goleiro_queda_levantando:
        // Espera o movimento de levantar terminar (finished_move vira true
        // via result_callback, mesmo mecanismo do send_goal/action_client).
        if (robot.finished_move)
        {
            RCLCPP_DEBUG(this->get_logger(), "Goleiro: de pé de novo, chamando FindBall2()");
            robot.state = goleiro_finding_ball_2;
        }
        break;

    case goleiro_finding_ball_2:
        FindBall2();
        break;
    }
}

// ---------------------------------------------------------------------------
// Posicionamento(): casos descritos no racional (item 3)
// ---------------------------------------------------------------------------
void GoleiroBehavior::Posicionamento()
{
    if (robot.neck_pos.position20 >= AnguloQueda)
    {
        // Caso a) 1 X e (2 L ou 0 L): a bola já chega travada/centralizada em
        // Posicionamento() (foi travada em searching_ball/FindBall2 e vem
        // sendo acompanhada nos M19+M20 desde então) — não é preciso checar
        // ball_centered() nem realinhar aqui de novo. O robô só continua se
        // aproximando enquanto os dois motores do pescoço seguem a bola, e
        // decide VerQueda() olhando direto pro M19.
        if (robot.x_count == 1 && (robot.l_count == 2 || robot.l_count == 0))
        {
            if (robot.neck_pos.position19 <= AnguloQueda)
            {
                robot.state = goleiro_ver_queda;
            }
            else
            {
                send_goal(g_walk); // aproxima-se da bola (M19/M20 seguem sozinhos)
            }
        }
        // Caso b) 0 X e 1 L, position19 <= XXXX_1: andar para a esquerda
        else if (robot.x_count == 0 && robot.l_count == 1 && robot.neck_pos.position19 <= XXXX_1)
        {
            if (robot.neck_pos.position19 <= AnguloQueda)
            {
                robot.state = goleiro_ver_queda;
            }
            else
            {
                send_goal(g_walk_left);
            }
        }
        // Caso c) 0 X e 1 L, position19 >= XXXX_2: andar para a direita
        else if (robot.x_count == 0 && robot.l_count == 1 && robot.neck_pos.position19 >= XXXX_2)
        {
            if (robot.neck_pos.position19 <= AnguloQueda)
            {
                robot.state = goleiro_ver_queda;
            }
            else
            {
                send_goal(g_walk_right);
            }
        }
        else
        {
            // Combinação de X/L não coberta pelo racional original recebido.
            send_goal(g_stand_still);
        }
    }
    else
    {
        // position20 já abaixo do ângulo de queda -> vai direto para VerQueda()
        robot.state = goleiro_ver_queda;
    }
}

void GoleiroBehavior::VerQueda()
{
    // Chamada uma única vez ao entrar no estado goleiro_ver_queda: decide o
    // que fazer (agachar ou cair p/ um lado) e já agenda a espera de 7s.

    // Caso 1: mesma combinação de landmarks do caso (a) de Posicionamento()
    // -> 1 X e (2 L ou 0 L): agacha (squat) em vez de mergulhar.
    if (robot.x_count == 1 && (robot.l_count == 2 || robot.l_count == 0))
    {
        RCLCPP_INFO(this->get_logger(), "VerQueda: caso 1 (1X, 2L/0L) -> squat");
        send_goal(g_squat);
        queda_get_up_move_ = g_stand_up_side; // TODO: confirmar move de levantar do squat
    }
    // Caso 2: qualquer outra combinação de landmarks -> cai p/ a direita ou
    // p/ a esquerda de acordo com robot.neck_pos.position19 vs XXXX_3.
    else
    {
        if (robot.neck_pos.position19 >= XXXX_3)
        {
            RCLCPP_INFO(this->get_logger(), "VerQueda: caso 2 -> queda p/ direita (pos19=%d)", robot.neck_pos.position19);
            send_goal(g_dive_right); // TODO: confirmar action_number real da queda (não existe ainda em attributes.h)
            queda_get_up_move_ = g_stand_up_side; // TODO: confirmar move de levantar específico da queda p/ direita
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "VerQueda: caso 2 -> queda p/ esquerda (pos19=%d)", robot.neck_pos.position19);
            send_goal(g_dive_left); // TODO: confirmar action_number real da queda (não existe ainda em attributes.h)
            queda_get_up_move_ = g_stand_up_side; // TODO: confirmar move de levantar específico da queda p/ esquerda
        }
    }

    queda_wait_start_ = this->now();
    robot.state = goleiro_queda_esperando;
}

void GoleiroBehavior::FindBall2()
{
    // Chamada em todo ciclo enquanto robot.state == goleiro_finding_ball_2.
    // Primeiro precisa travar a visão na bola de novo — mesmo método usado em
    // goleiro_searching_ball: procura com o M19 e alinha ao centro.
    if (!search_and_align_ball_M19())
    {
        return; // permanece em goleiro_finding_ball_2 até a bola travar
    }

    if (robot.neck_pos.position20 <= (DangerArea + XXXX_4))
    {
        // Já está dentro de DangerArea (+ margem XXXX_4) -> aciona
        // Posicionamento() imediatamente, sem passar pelo tracking_ball.
        RCLCPP_INFO(this->get_logger(),
            "FindBall2: pos20=%d <= DangerArea(%d)+XXXX_4(%d) -> Posicionamento()",
            robot.neck_pos.position20, DangerArea, XXXX_4);
        robot.state = goleiro_positioning;
        Posicionamento();
    }
    else
    {
        // Ainda longe (considerando a margem): volta a se alinhar/acompanhar
        // a bola com o motor 20, reaproveitando o mesmo estado/lógica de
        // goleiro_tracking_ball — que por si só já transiciona para
        // goleiro_positioning assim que M20 <= DangerArea (sem a margem extra).
        RCLCPP_DEBUG(this->get_logger(),
            "FindBall2: pos20=%d ainda acima do limite -> volta a seguir a bola (tracking_ball)",
            robot.neck_pos.position20);
        robot.state = goleiro_tracking_ball;
    }
}

// ---------------------------------------------------------------------------
bool GoleiroBehavior::ball_is_locked()
{
    return robot.ball_info.detected;
}

bool GoleiroBehavior::ball_centered()
{
    return robot.ball_info.detected && robot.ball_info.center;
}

bool GoleiroBehavior::search_and_align_ball_M19()
{
    if (!robot.ball_info.detected)
    {
        // Ainda não vê a bola: varredura em zigue-zague com o M19 -> gira pra
        // um lado até bater no limite daquele lado, inverte, gira pro outro
        // lado até bater no limite dele, inverte de novo, e assim por diante.
        if (search_going_left_)
        {
            send_goal(g_turn_left);
            if (robot.neck_pos.position19 >= NeckSearchLeftLimit)
            {
                search_going_left_ = false; // bateu no limite esquerdo -> inverte pra direita
            }
        }
        else
        {
            send_goal(g_turn_right);
            if (robot.neck_pos.position19 <= NeckSearchRightLimit)
            {
                search_going_left_ = true; // bateu no limite direito -> inverte pra esquerda
            }
        }
        return false;
    }

    if (!ball_centered())
    {
        // Vê a bola, mas ainda não está alinhada ao centro (M19) -> gira
        // para o lado indicado pelo Vision msg até centralizar.
        if (robot.ball_info.left)
        {
            send_goal(g_turn_left);
        }
        else if (robot.ball_info.right)
        {
            send_goal(g_turn_right);
        }
        return false;
    }

    // Detectada e centralizada -> bola travada.
    return true;
}

// ---------------------------------------------------------------------------
void GoleiroBehavior::send_goal(const GoleiroMove &order)
{
    if (!this->action_client_->wait_for_action_server(1s))
    {
        RCLCPP_ERROR(this->get_logger(), "Action server (control_action) indisponível");
        return;
    }

    auto goal_msg = ControlActionMsg::Goal();
    goal_msg.action_number = order;

    if (order != robot.movement || robot.finished_move)
    {
        action_client_->async_send_goal(goal_msg, send_goal_options_);
        robot.movement = order;
        robot.finished_move = false;
    }
}

// ---------------------------------------------------------------------------
// Callbacks - publishers do "pacote de posição" (pescoço) e da visão
// (vision_pkg / detect.py)
// ---------------------------------------------------------------------------
void GoleiroBehavior::listener_callback_GC(const GameControllerMsg::SharedPtr msg)
{
    gc_info = *msg;
}

void GoleiroBehavior::listener_callback_neck_pos(const JointStateMsg::SharedPtr msg)
{
    // JointState publica um vetor "info" indexado pelo id do motor
    // (mesmo padrão usado em decision_node.cpp: info[19] / info[20]).
    robot.neck_pos.position19 = msg->info[19];
    robot.neck_pos.position20 = msg->info[20];
}

void GoleiroBehavior::listener_callback_ball(const VisionMsg::SharedPtr msg)
{
    robot.ball_info = *msg;
}

void GoleiroBehavior::listener_callback_landmark_summary(const LandmarkCountMsg::SharedPtr msg)
{
    // Tópico "robot_behavior" (localization_node): resumo já filtrado,
    // atualizado a cada 60 frames de visão.
    robot.l_count = msg->l_count;
    robot.t_count = msg->t_count;
    robot.x_count = msg->x_count;
    robot.goalpost_count = msg->goalpost_count;
}

void GoleiroBehavior::goal_response_callback(const GoalHandleControl::SharedPtr &goal_handle)
{
    goal_handle_ = goal_handle;
    if (!goal_handle)
    {
        RCLCPP_ERROR(this->get_logger(), "Goal recusado pelo servidor de controle");
    }
}

void GoleiroBehavior::feedback_callback(
    GoalHandleControl::SharedPtr,
    const std::shared_ptr<const ControlActionMsg::Feedback> feedback)
{
    (void)feedback;
}

void GoleiroBehavior::result_callback(const GoalHandleControl::WrappedResult &result)
{
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
    {
        robot.finished_move = true;
    }
}

// ---------------------------------------------------------------------------
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GoleiroBehavior>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
