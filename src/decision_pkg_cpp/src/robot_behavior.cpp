// ros2 run decision_pkg_cpp decision --ros-args -p body_activate_:=false

#include "decision_pkg_cpp/robot_behavior.hpp"
#include "decision_node.cpp"
#include<unistd.h> // apenas para delay, apagar

#define MAX_LOST_BALL_TIME 7000 //10 seconds

// defining the opposite side --> 1 - right --> 0 - left
int opposite_side;
bool aligned_goalpost = false; // alinhado com a trave

RobotBehavior::RobotBehavior()
{
    robot_behavior_ = this->create_wall_timer(
        8ms,
        std::bind(&RobotBehavior::players_behavior, this));

    // Limiares da FSM do goleiro (goalkeeper_normal_game), mesmos nomes de
    // parâmetro e defaults de goleiro_behavior.cpp — ainda placeholders, a
    // calibrar em campo (ver robot_config1.yaml).
    DangerArea  = this->declare_parameter("danger_area", 2003);
    AnguloQueda = this->declare_parameter("angulo_queda", 1830);
    XXXX_1      = this->declare_parameter("xxxx_1", 2350);
    XXXX_2      = this->declare_parameter("xxxx_2", 1750);
    XXXX_3      = this->declare_parameter("xxxx_3", 1900);
    XXXX_4      = this->declare_parameter("xxxx_4", 300);
}

RobotBehavior::~RobotBehavior()     //checkpoint
{
}

void RobotBehavior::players_behavior()
{
    
    if (is_penalized())  RCLCPP_INFO(this->get_logger(), "Penalizado");  //robo penalizado
    else
    {
        if(robot_fallen(robot)) get_up();
	    else
        {
            switch (gc_info.secondary_state)
            {
            case GameControllerMsg::STATE_NORMAL:
                normal_game();
                break;
            
            case GameControllerMsg::STATE_PENALTYSHOOT:
                //RCLCPP_INFO(this->get_logger(), "ENTROU NO PENALTYSHOOT");
                penalty();
                break;

            default:
                break;
            }
        }
    }

}

void RobotBehavior::penalty()           //penalizado
{
    RCLCPP_INFO(this->get_logger(), "entrou no penalty");
    switch (gc_info.game_state)
    {
    case GameControllerMsg::GAMESTATE_INITAL: 
        send_goal(stand_still);
        break;
        
    case GameControllerMsg::GAMESTATE_SET: 
        send_goal(stand_still);
        break;
    
    case GameControllerMsg::GAMESTATE_PLAYING: 
        if(gc_info.has_kick_off)
        {
            //player_penalty();
            RCLCPP_INFO(this->get_logger(), "CHUTANDO");
            send_goal(left_kick);
        }
        // else goalkeeper_penalty();
        break;
   
    case GameControllerMsg::GAMESTATE_FINISHED: 
        send_goal(stand_still);
        break;
    }
}


void RobotBehavior::normal_game()           //jogo normal
{

    // RCLCPP_DEBUG(this->get_logger(), "Normal Game: %d", gc_info.game_state);
    switch (gc_info.game_state)
    {
    case GameControllerMsg::GAMESTATE_INITAL: // conferido

        if(robot.neck_pos.position19 < 2048){ // definir lado do campo 1 - right 0 - left
            opposite_side = 1;
        }else{
            opposite_side = 0;
        }
        send_goal(stand_still);
        break;
    
    case GameControllerMsg::GAMESTATE_READY:
    {
        //RCLCPP_INFO(this->get_logger(), "🎯 READY | Etapa:ck:");

        send_goal(stand_still);
        // static int ready_counter = 0;
        // static int ready_etapa = ETAPA_WALK;

        // POR CONTANDOR 

        // switch (ready_etapa)
        // {
        // case ETAPA_WALK:
        //     send_goal(walk);
        //     ready_counter++;
        //     if (ready_counter > 3700) 
        //     {
        //         send_goal(stand_still);
        //         ready_etapa = ETAPA_TURN;
        //         ready_counter = 0;
        //     }
        //     break;

        // case ETAPA_TURN:
        //     send_goal(turn_right);
        //     ready_counter++;
        //     if (ready_counter >= 80) // ~1.6 segundos de giro
        //     {
        //         ready_etapa = ETAPA_PARAR;
        //         ready_counter = 0;
        //     }
        //     break;

        // case ETAPA_PARAR:
        //     send_goal(stand_still);
        //     break;
        // }

        

        // COM A POSIÇÃO DA BOLA

        // int neck = robot.neck_pos.position19;
        
        // static bool yaw_fixed = false;  // variável estática local para garantir que IMU atualiza 1 vez so

        // if (!yaw_fixed) // atualizar ponto zero da IMU
        // {
        //     yaw_reference_ = robot.imu_yaw_rad;
        //     yaw_reference_set_ = true;
        //     yaw_fixed = true;

        // }
        //RCLCPP_INFO(this->get_logger(), "🎯 READY | Etapa: %d | Neck: %d", ready_etapa, neck);    
        
        
        // RCLCPP_INFO(this->get_logger(), "%d", opposite_side);   
        // send_goal(walk);
        // switch (ready_etapa)
        // {
        // case ETAPA_WALK:
        //     if (opposite_side == 0)
        //     {
        //         if (neck >= 1700)
        //         {
        //             send_goal(walk);
        //             //RCLCPP_INFO(this->get_logger(), "🚶 Andando lateralmente");
        //         }
        //         else
        //         {
        //             send_goal(stand_still);
        //             ready_etapa = ETAPA_TURN;
        //             //RCLCPP_INFO(this->get_logger(), "➡️ Mudando para ETAPA_TURN");
        //         }
        //     }
        //     else  if(opposite_side == 1)
        //     {
        //         if (neck >= 1500)
        //         {
        //             send_goal(walk);
        //             //RCLCPP_INFO(this->get_logger(), "🚶 Andando lateralmente");
        //         }
        //         else
        //         {
        //             send_goal(stand_still);
        //             ready_etapa = ETAPA_TURN;
        //             //RCLCPP_INFO(this->get_logger(), "➡️ Mudando para ETAPA_TURN");
        //         }   
        //     }
                
            
        //     break;

        // case ETAPA_TURN:
        //     if (neck > 2068 || neck < 2028)
        //     {
        //         if (neck > 2068) send_goal(turn_left);
        //         else send_goal(turn_right);
        //         //RCLCPP_INFO(this->get_logger(), "↩️ Virando até alinhar com 2048");
        //     }
        //     else
        //     {
        //         ready_etapa = ETAPA_PARAR;
        //         //RCLCPP_INFO(this->get_logger(), "✅ Alinhou com a bola, indo para PARAR");
        //     }
        //     break;

        // case ETAPA_PARAR:
        //     send_goal(stand_still);
        //     //RCLCPP_INFO(this->get_logger(), "🛑 Parado (etapa final)");
        //     break;
        // }

        // break;
    }
        
    case GameControllerMsg::GAMESTATE_SET: // feito
        send_goal(stand_still);
        yaw_reference_set_ = false;
        break;
    
    case GameControllerMsg::GAMESTATE_PLAYING:  // começo do jogo
        if(gc_info.has_kick_off || (!gc_info.has_kick_off && gc_info.secondary_seconds_remaining == 0))
        {
            std_msgs::msg::Bool localization_active = robot.localization_msg; // topico que manda true/false para localização

            if(!localization_active.data){ 
                // RCLCPP_INFO(this->get_logger(), "MACACAQUITO DANÇANDO")
                if(is_goalkeeper(ROBOT_NUMBER)) goalkeeper_normal_game();
                else if (is_kicker(ROBOT_NUMBER)) kicker_normal_game();
                else if(is_bala(ROBOT_NUMBER)) bala_normal_game();
            }
            else{
                // RCLCPP_INFO(this->get_logger(), "COM LOCALIZAÇÃO")
                if (is_kicker(ROBOT_NUMBER)) kicker_localization_game();
                else if (is_bala(ROBOT_NUMBER)) bala_localization_game();
            }
        }
        break;
   
    case GameControllerMsg::GAMESTATE_FINISHED: // feito
        send_goal(stand_still);
        break;
    }

}


void RobotBehavior::bala_normal_game()                //estado de jogo normal; jogo rolando 
{

    //RCLCPP_INFO(this->get_logger(), "Recebido yaw_est: %f", yaw_est_value_);
    //RCLCPP_INFO(this->get_logger(), "robot state %d", robot.state);
    //RCLCPP_FATAL(this->get_logger(), "posição do 20: %d", robot.neck_pos.position20);
    //RCLCPP_WARN(this->get_logger(), "posição do 19: %d", robot.neck_pos.position19);
    //RCLCPP_INFO(this->get_logger(), "Bala Normal Game");

    switch (robot.state)
    {
    case searching_ball:
        //RCLCPP_ERROR(this->get_logger(), "Searching ball");
        //RCLCPP_ERROR(this->get_logger(), "lost ball timer  %d", lost_ball_timer.delayNR(MAX_LOST_BALL_TIME));

        if(ball_is_locked())
            {   //RCLCPP_ERROR(this->get_logger(), "ball locked");
                if(robot.ball_position == center) robot.state = ball_approach;      //anda ate a bola
                else robot.state = aligning_with_the_ball;
            }
        else if(lost_ball_timer.delayNR(MAX_LOST_BALL_TIME)) send_goal(turn_left);        //alinha o corpo com a bola
        //else send_goal(gait); // gait
        break;
    
    case aligning_with_the_ball:
        //RCLCPP_WARN(this->get_logger(), "Aligning with the_ball");
        if(robot_align_with_the_ball()) robot.state = ball_approach;
        //else if(ball_is_locked()) robot.state = ball_approach;
        else if(!robot.camera_ball_position.detected) robot.state = searching_ball;
        else if(robot.neck_pos.position19 > 2150) send_goal(turn_left);
        else if(robot.neck_pos.position19 < 1900) send_goal(turn_right);
        break;

    case ball_approach:
        //RCLCPP_ERROR(this->get_logger(), "neck limit %d, ball locked %d, ball close %d", ball_in_close_limit(), ball_is_locked(), robot.camera_ball_position.close);
        //RCLCPP_ERROR(this->get_logger(), "ball_approach");
        if((robot.neck_pos.position20 < 1300) && (ball_is_locked())) 
        {
            robot.state = kick_ball;
        }         //perdeu a bola
        else if(!robot.camera_ball_position.detected) robot.state = searching_ball; 
        else if(!robot_align_with_the_ball()) robot.state = aligning_with_the_ball;
        else send_goal(walk);
        break;


    case ball_close:
        //RCLCPP_WARN(this->get_logger(), "ball right %d, ball left %d", robot_align_for_kick_right(), robot_align_for_kick_left());
        //RCLCPP_WARN(this->get_logger(), "ball close");
        if (robot.neck_pos.position19 > 2600)
        {
            send_goal(turn_left);
        }
        else if (robot.neck_pos.position19 < 1500)
        {
            send_goal(turn_right);
        }
        else if ((robot.neck_pos.position19 < 2150) && (robot.neck_pos.position19 > 1980))
        {
            robot.state = kick_ball;
        }
        else if(!robot.camera_ball_position.detected || !robot.camera_ball_position.close) robot.state = searching_ball;
        else if(!robot_align_with_the_ball()) robot.state = aligning_with_the_ball;
        //else if(robot_align_for_kick_left()) robot.state = kick_ball;
        break;

    case kick_ball:
        //RCLCPP_ERROR(this->get_logger(), "kick");
        if ((!robot.camera_ball_position.detected) || (robot.ball_position != center))
	    {
            send_goal(gait);
            robot.state = searching_ball;
            lost_ball_timer.reset();
	    } 
        if (robot.neck_pos.position20 >= 1555)
        {
            robot.state = aligning_with_the_ball;
        }
        else if(robot.neck_pos.position19 >= 2300){
            send_goal(walk_left);
        }
        else if(robot.neck_pos.position19 <= 1700){
            send_goal(walk_right);
        }
        else/* if((robot.neck_pos.position19 > 1996) && (robot.neck_pos.position19 < 2200))*/{
            send_goal(walk);
        }
        //else if(lost_ball_timer.delayNR(2000)) robot.state = searching_ball; //para testar com o corpo desatiavdo
	    break;
    }
}

void RobotBehavior::kicker_normal_game()                //estado de jogo normal; jogo rolando 
{
    //RCLCPP_INFO(this->get_logger(), "robot state %d", robot.state);
    //RCLCPP_FATAL(this->get_logger(), "posição do 20: %d", robot.neck_pos.position20);
    //RCLCPP_WARN(this->get_logger(), "posição do 19: %d", robot.neck_pos.position19);
    //RCLCPP_INFO(this->get_logger(), "Kicker Normal Game");
    switch (robot.state)
    {
    case searching_ball:
        RCLCPP_INFO(this->get_logger(), "Searching ball");
        //RCLCPP_ERROR(this->get_logger(), "lost ball timer  %d", lost_ball_timer.delayNR(MAX_LOST_BALL_TIME));

        if(ball_is_locked())
            {   //RCLCPP_INFO(this->get_logger(), "ball locked");
                if(robot.ball_position == center) robot.state = ball_approach;      //anda ate a bola
                else robot.state = aligning_with_the_ball;
            }
        else if(lost_ball_timer.delayNR(MAX_LOST_BALL_TIME)) send_goal(turn_left);        //alinha o corpo com a bola
        //else send_goal(gait); // gait
        break;
    
    case aligning_with_the_ball:
        //RCLCPP_INFO(this->get_logger(), "Aligning with the_ball");
        if(robot_align_with_the_ball()) robot.state = ball_approach;
        //else if(ball_is_locked()) robot.state = ball_approach;
        else if(!robot.camera_ball_position.detected) robot.state = searching_ball;
        else if(robot.neck_pos.position19 > 2300) send_goal(turn_left);
        else if(robot.neck_pos.position19 < 1600) send_goal(turn_right);
        break;

    case ball_approach:
        //RCLCPP_ERROR(this->get_logger(), "neck limit %d, ball locked %d, ball close %d", ball_in_close_limit(), ball_is_locked(), robot.camera_ball_position.close);
        //RCLCPP_INFO(this->get_logger(), "ball_approach");
        if((robot.neck_pos.position20 < 1300)& (ball_is_locked())) 
            {
            robot.state = kick_ball;
            }         //perdeu a bola
        else if(!robot.camera_ball_position.detected) 
            {
            robot.state = searching_ball;
            } // pode estar bugando
        else if(!robot_align_with_the_ball()) 
            {
            robot.state = aligning_with_the_ball;
            }
        else 
            {
            send_goal(walk);
            }
        break;


    // case ball_close:
    //     //RCLCPP_INFO(this->get_logger(), "ball right %d, ball left %d", robot_align_for_kick_right(), robot_align_for_kick_left());
    //     RCLCPP_INFO(this->get_logger(), "ball close");

    //     if(!robot.camera_ball_position.detected || !robot.camera_ball_position.close) robot.state = searching_ball;
    //     else if (robot.neck_pos.position20 < 1200) robot.state = kick_ball;
        
    //     if (robot.neck_pos.position19 < 1600)
    //     {
    //         send_goal(walk_right);
    //         RCLCPP_INFO(this->get_logger(), "walking right");
    //     }
    //     else if (robot.neck_pos.position19 > 2360)
    //     {
    //         send_goal(walk_left);
    //         RCLCPP_INFO(this->get_logger(), "walking left");
    //     }
    //     else
    //     {
    //         robot.state = kick_ball;
    //     }
    //     //else if(robot_align_for_kick_left()) robot.state = kick_ball;
    //     break;

    case kick_ball:
        //RCLCPP_INFO(this->get_logger(), "kick");
        if(robot.movement != 3 && robot.movement != 4) 
        {
            if (robot.neck_pos.position19 >= 1950) 
                {   
                    //RCLCPP_INFO(this->get_logger(), "kick_left");
                    send_goal(left_kick);
                    robot.state = searching_ball;
                    
                // RCLCPP_INFO(this->get_logger(), "posição do 20: %d", robot.neck_pos.position20);
                }
                else
                {   

                    //RCLCPP_INFO(this->get_logger(), "kick_right");
                    send_goal(right_kick);
                    robot.state = searching_ball;
                // RCLCPP_INFO(this->get_logger(), "posição do 20: %d", robot.neck_pos.position20);
            } 
        }
        else if(robot.finished_move)
	    {
            robot.state = searching_ball;
            lost_ball_timer.reset();
        }
        else if(!robot.camera_ball_position.detected || !robot.camera_ball_position.close) robot.state = searching_ball;
        else if (robot.neck_pos.position20 > 1400) robot.state = aligning_with_the_ball;
        //else if(lost_ball_timer.delayNR(2000)) robot.state = searching_ball; //para testar com o corpo desatiavdo
        break;
    }
}

void RobotBehavior::kicker_localization_game()                //estado de jogo normal; jogo rolando 
{
    switch (robot.state)
    {
    case searching_ball:
        this->free_neck();
        //RCLCPP_DEBUG(this->get_logger(), "Seaching ball");
        if(ball_is_locked())
            {   //RCLCPP_ERROR(this->get_logger(), "ball locked");
                if(robot.ball_position == center) robot.state = ball_approach;      //anda ate a bola
                else robot.state = aligning_with_the_ball;
            }
        else if(lost_ball_timer.delayNR(MAX_LOST_BALL_TIME)) send_goal(turn_left);        //alinha o corpo com a bola
        break;
    
    case aligning_with_the_ball:
        //RCLCPP_DEBUG(this->get_logger(), "Aligning with the_ball");
        if(robot_align_with_the_ball()) robot.state = ball_approach;
        //else if(ball_is_locked()) robot.state = ball_approach;
        else if(!robot.camera_ball_position.detected) robot.state = searching_ball;
        else if(robot.neck_pos.position19 > 2100) send_goal(turn_left);
        else if(robot.neck_pos.position19 < 1900) send_goal(turn_right);
        break;

    case ball_approach:
        // RCLCPP_DEBUG(this->get_logger(), "ball_approach");
        if((robot.neck_pos.position20 < 1350)& (ball_is_locked())) 
            {
            robot.state = ball_close;
            }         
            
        else if(!robot.camera_ball_position.detected) 
            {
            robot.state = searching_ball;
            }
        else if(!robot_align_with_the_ball()) 
            {
            robot.state = aligning_with_the_ball;
            }
        else 
            {
            send_goal(walk);
            }
        break;

    case ball_close:
        RCLCPP_DEBUG(this->get_logger(), "ball close");
        
            
        delta_yaw = fabs(robot.imu_yaw_rad - yaw_reference_); // estabelece a referencia do "zero" da IMU
        //RCLCPP_INFO(this->get_logger(), "Delta yaw: %f", delta_yaw);
        RCLCPP_INFO(this->get_logger(), "Delta yaw ABS: %f", fabs(delta_yaw));

        if (delta_yaw > M_PI) delta_yaw -= 2*M_PI; 
        if (delta_yaw < -M_PI) delta_yaw += 2*M_PI;
        
        // entrei pelo lado direito
        if (opposite_side == 0)
        {
            if (delta_yaw > 0.1 && delta_yaw <= 0.6 && robot.neck_pos.position20 < 1230) // gol alinado
            {
                robot.state = kick_ball;
            }
            else if (delta_yaw > 0.6 && delta_yaw <= 2)
            {
                send_goal(turn_ball_left);
            }
            else
            {
                send_goal(turn_ball_right);
            }
        }
        if(!robot.camera_ball_position.detected) robot.state = searching_ball;

        break;
    

    case kick_ball:
        // Envia o chute apenas se ele ainda não está em execução
        if(robot.movement != left_kick && robot.movement != right_kick)
        {
            if (robot.neck_pos.position19 >= 2048) 
            {
                send_goal(left_kick);  
            }
            else 
            {
                send_goal(right_kick); 
            }
        }
    
        // Espera o chute terminar
        if(robot.finished_move)
        {
            robot.state = searching_ball;
            lost_ball_timer.reset();
        }
        break;
    }
     
}

void RobotBehavior::bala_localization_game()                //estado de jogo normal; jogo rolando 
{
    //RCLCPP_INFO(this->get_logger(), "BALA LOCALIZATION GAME");
    switch (robot.state)
    {
    case searching_ball:
        this->free_neck();
        //RCLCPP_DEBUG(this->get_logger(), "Seaching ball");
        if(ball_is_locked())
            {   //RCLCPP_ERROR(this->get_logger(), "ball locked");
                if(robot.ball_position == center) robot.state = ball_approach;      //anda ate a bola
                else robot.state = aligning_with_the_ball;
            }
        else if(lost_ball_timer.delayNR(MAX_LOST_BALL_TIME)) send_goal(turn_left);        //alinha o corpo com a bola
        break;
    
    case aligning_with_the_ball:
        //RCLCPP_DEBUG(this->get_logger(), "Aligning with the_ball");
        if(robot_align_with_the_ball()) robot.state = ball_approach;
        //else if(ball_is_locked()) robot.state = ball_approach;
        else if(!robot.camera_ball_position.detected) robot.state = searching_ball;
        else if(robot.neck_pos.position19 > 2100) send_goal(turn_left);
        else if(robot.neck_pos.position19 < 1900) send_goal(turn_right);
        break;

    case ball_approach:
        //RCLCPP_DEBUG(this->get_logger(), "ball_approach");
        if((robot.neck_pos.position20 < 1400)& (ball_is_locked())) 
            {
            robot.state = ball_close;
            }         
            
        else if(!robot.camera_ball_position.detected) 
            {
            //RCLCPP_ERROR(this->get_logger(), "primeiro else if");
            robot.state = searching_ball;
            } // pode estar bugando
        else if(!robot_align_with_the_ball()) 
            {
            //RCLCPP_ERROR(this->get_logger(), "segundo else if");
            robot.state = aligning_with_the_ball;
            }
        else 
            {
            send_goal(walk);
            }
        break;

    case ball_close:
        RCLCPP_DEBUG(this->get_logger(), "ball close");
        
        //RCLCPP_INFO(this->get_logger(), "Z: %f", robot.imu_gyro.vector.z);
            
        delta_yaw = fabs(robot.imu_yaw_rad - yaw_reference_);
        RCLCPP_INFO(this->get_logger(), "Delta yaw ABS: %f", delta_yaw);

        if (delta_yaw > M_PI) delta_yaw -= 2*M_PI;
        if (delta_yaw < -M_PI) delta_yaw += 2*M_PI;
        
        // entrei pelo lado direito
        if (opposite_side == 0)
        {
            if (delta_yaw > 0.1 && delta_yaw <= 0.6)
            {
                robot.state = kick_ball;
            }
            else if (delta_yaw > 0.6 && delta_yaw <= 2)
            {
                send_goal(turn_left);
            }
            else
            {
                send_goal(turn_right);
            }
        }

        if(!robot.camera_ball_position.detected) 
        {
            robot.state = searching_ball;
        }

        break;

    

    case kick_ball:
        //RCLCPP_ERROR(this->get_logger(), "kick");
        if ((!robot.camera_ball_position.detected) || (robot.ball_position != center))
	    {
            send_goal(gait);
            robot.state = searching_ball;
            lost_ball_timer.reset();
	    }
        else send_goal(walk);

        //else if(lost_ball_timer.delayNR(2000)) robot.state = searching_ball; //para testar com o corpo desatiavdo
	    break;
    }

}

void RobotBehavior::goalkeeper_normal_game() // caso o jogador seja o goleiro
{
    // Máquina de estados portada de GoleiroBehavior::goleiro_normal_game()
    // (goleiro_decision/src/goleiro_behavior.cpp). Usa robot.gk_state
    // (GoalkeeperState), um campo dedicado — robot.state (State) continua
    // sendo usado só por bala/kicker, sem risco de colisão de valores.
    RCLCPP_DEBUG(this->get_logger(), "goalkeeper state %d", robot.gk_state);

    switch (robot.gk_state)
    {
    case goalkeeper_searching_ball:
        // 1) Entrar em goalkeeper_searching_ball até fixar a visão na bola:
        // gira o M19 procurando, alinha ao centro e só então trava.
        RCLCPP_DEBUG(this->get_logger(), "Goalkeeper: searching_ball");
        if (goalkeeper_search_and_align_ball_M19())
        {
            robot.gk_state = goalkeeper_tracking_ball;
        }
        break;

    case goalkeeper_tracking_ball:
        // 2) Enquanto neck_pos.position20 >= DangerArea, só segue a bola com
        //    o motor 20 (corpo parado).
        if (!goalkeeper_ball_is_locked())
        {
            robot.gk_state = goalkeeper_searching_ball;
        }
        else if (robot.neck_pos.position20 >= DangerArea)
        {
            RCLCPP_DEBUG(this->get_logger(), "Goalkeeper: seguindo bola (motor 20) pos20=%d", robot.neck_pos.position20);
            send_goal(stand_still);
        }
        else
        {
            // 3) neck_pos.position20 <= DangerArea -> chama goalkeeper_positioning_logic()
            robot.gk_state = goalkeeper_positioning;
        }
        break;

    case goalkeeper_positioning:
        goalkeeper_positioning_logic();
        break;

    case goalkeeper_ver_queda:
        // Decide (uma vez) squat ou queda p/ um dos lados, e já entra em
        // goalkeeper_queda_esperando dentro de goalkeeper_decide_fall().
        goalkeeper_decide_fall();
        break;

    case goalkeeper_queda_esperando:
        // Fica parado (squat/caído) até completar os 7 segundos no chão.
        if ((this->now() - gk_queda_wait_start_).seconds() >= 7.0)
        {
            send_goal(gk_queda_get_up_move_);
            robot.gk_state = goalkeeper_queda_levantando;
        }
        break;

    case goalkeeper_queda_levantando:
        // Espera o movimento de levantar terminar (finished_move vira true
        // via result_callback).
        if (robot.finished_move)
        {
            RCLCPP_DEBUG(this->get_logger(), "Goalkeeper: de pé de novo, chamando goalkeeper_find_ball_after_fall()");
            robot.gk_state = goalkeeper_finding_ball_2;
        }
        break;

    case goalkeeper_finding_ball_2:
        goalkeeper_find_ball_after_fall();
        break;
    }
}

// ---------------------------------------------------------------------------
// goalkeeper_positioning_logic(): porta Posicionamento() de goleiro_behavior.cpp
// ---------------------------------------------------------------------------
void RobotBehavior::goalkeeper_positioning_logic()
{
    if (robot.neck_pos.position20 >= AnguloQueda)
    {
        // Caso a) 1 X e (2 L ou 0 L): a bola já chega travada/centralizada
        // aqui — o robô só continua se aproximando enquanto os dois motores
        // do pescoço seguem a bola, e decide a queda olhando direto pro M19.
        if (robot.x_count == 1 && (robot.l_count == 2 || robot.l_count == 0))
        {
            if (robot.neck_pos.position19 <= AnguloQueda) robot.gk_state = goalkeeper_ver_queda;
            else send_goal(walk); // aproxima-se da bola (M19/M20 seguem sozinhos)
        }
        // Caso b) 0 X e 1 L, position19 <= XXXX_1: andar para a esquerda
        else if (robot.x_count == 0 && robot.l_count == 1 && robot.neck_pos.position19 <= XXXX_1)
        {
            if (robot.neck_pos.position19 <= AnguloQueda) robot.gk_state = goalkeeper_ver_queda;
            else send_goal(walk_left);
        }
        // Caso c) 0 X e 1 L, position19 >= XXXX_2: andar para a direita
        else if (robot.x_count == 0 && robot.l_count == 1 && robot.neck_pos.position19 >= XXXX_2)
        {
            if (robot.neck_pos.position19 <= AnguloQueda) robot.gk_state = goalkeeper_ver_queda;
            else send_goal(walk_right);
        }
        else
        {
            // Combinação de X/L não coberta pelo racional original.
            send_goal(stand_still);
        }
    }
    else
    {
        // position20 já abaixo do ângulo de queda -> vai direto pra decisão de queda
        robot.gk_state = goalkeeper_ver_queda;
    }
}

// ---------------------------------------------------------------------------
// goalkeeper_decide_fall(): porta VerQueda() de goleiro_behavior.cpp
// ---------------------------------------------------------------------------
void RobotBehavior::goalkeeper_decide_fall()
{
    // Chamada uma única vez ao entrar no estado goalkeeper_ver_queda: decide
    // o que fazer (agachar ou cair p/ um lado) e já agenda a espera de 7s.

    // Caso 1: mesma combinação de landmarks do caso (a) de
    // goalkeeper_positioning_logic() -> 1 X e (2 L ou 0 L): agacha (squat)
    // em vez de mergulhar.
    if (robot.x_count == 1 && (robot.l_count == 2 || robot.l_count == 0))
    {
        RCLCPP_INFO(this->get_logger(), "Goalkeeper decide_fall: caso 1 (1X, 2L/0L) -> squat");
        send_goal(squat); // "Goalkeeper Middle" (control.cpp case 13) — bloqueio central
        gk_queda_get_up_move_ = stand_up_front; // bloqueio central levanta como uma queda de frente
    }
    // Caso 2: qualquer outra combinação de landmarks -> cai p/ a direita, p/
    // a esquerda, ou agacha no meio (zona intermediária) de acordo com
    // robot.neck_pos.position19 vs [XXXX_3, XXXX_3 + XXXX_4].
    else
    {
        if (robot.neck_pos.position19 <= XXXX_3)
        {
            RCLCPP_INFO(this->get_logger(), "Goalkeeper decide_fall: caso 2 -> queda p/ direita (pos19=%d)", robot.neck_pos.position19);
            send_goal(dive_right); // "Goalkeeper Fall Right" (control.cpp case 12)
            gk_queda_get_up_move_ = stand_up_side_right; // "Fallen Side Right" (control.cpp case 19)
        }
        else if (robot.neck_pos.position19 > XXXX_3 + XXXX_4)
        {
            RCLCPP_INFO(this->get_logger(), "Goalkeeper decide_fall: caso 2 -> queda p/ esquerda (pos19=%d)", robot.neck_pos.position19);
            send_goal(dive_left); // "Goalkeeper Fall Left" (control.cpp case 11)
            gk_queda_get_up_move_ = stand_up_side_left; // "Fallen Side Left" (control.cpp case 18)
        }
        else
        {
            // Zona intermediária (XXXX_3 < pos19 <= XXXX_3 + XXXX_4): bola
            // longe demais dos dois lados p/ mergulhar com confiança -> agacha
            // no meio, igual ao caso 1.
            RCLCPP_INFO(this->get_logger(), "Goalkeeper decide_fall: caso 2 -> zona intermediária (pos19=%d) -> squat", robot.neck_pos.position19);
            send_goal(squat); // "Goalkeeper Middle" (control.cpp case 13) — bloqueio central
            gk_queda_get_up_move_ = stand_up_front; // bloqueio central levanta como uma queda de frente
        }
    }

    gk_queda_wait_start_ = this->now();
    robot.gk_state = goalkeeper_queda_esperando;
}

// ---------------------------------------------------------------------------
// goalkeeper_find_ball_after_fall(): porta FindBall2() de goleiro_behavior.cpp
// ---------------------------------------------------------------------------
void RobotBehavior::goalkeeper_find_ball_after_fall()
{
    // Chamada em todo ciclo enquanto robot.gk_state == goalkeeper_finding_ball_2.
    // Primeiro precisa travar a visão na bola de novo.
    if (!goalkeeper_search_and_align_ball_M19())
    {
        return; // permanece em goalkeeper_finding_ball_2 até a bola travar
    }

    if (robot.neck_pos.position20 <= (DangerArea + XXXX_4))
    {
        // Já está dentro de DangerArea (+ margem XXXX_4) -> aciona
        // goalkeeper_positioning_logic() imediatamente.
        RCLCPP_INFO(this->get_logger(),
            "Goalkeeper find_ball_after_fall: pos20=%d <= DangerArea(%d)+XXXX_4(%d) -> positioning",
            robot.neck_pos.position20, DangerArea, XXXX_4);
        robot.gk_state = goalkeeper_positioning;
        goalkeeper_positioning_logic();
    }
    else
    {
        // Ainda longe (considerando a margem): volta a acompanhar a bola com
        // o motor 20 (goalkeeper_tracking_ball), que por si só já transiciona
        // pra goalkeeper_positioning assim que M20 <= DangerArea.
        RCLCPP_DEBUG(this->get_logger(),
            "Goalkeeper find_ball_after_fall: pos20=%d ainda acima do limite -> tracking_ball",
            robot.neck_pos.position20);
        robot.gk_state = goalkeeper_tracking_ball;
    }
}

// ---------------------------------------------------------------------------
bool RobotBehavior::goalkeeper_ball_is_locked()
{
    // Porta GoleiroBehavior::ball_is_locked() — checagem simples de detecção
    // (sem exigir vision_stable()/lost_ball_timer como o ball_is_locked()
    // de bala/kicker; a FSM do goleiro não usa MAX_LOST_BALL_TIME).
    return robot.camera_ball_position.detected;
}

bool RobotBehavior::goalkeeper_ball_centered()
{
    return robot.camera_ball_position.detected && robot.camera_ball_position.center;
}

bool RobotBehavior::goalkeeper_search_and_align_ball_M19()
{
    // Porta GoleiroBehavior::search_and_align_ball_M19().
    if (!robot.camera_ball_position.detected)
    {
        // Ainda não vê a bola: varredura em zigue-zague com o M19 -> gira pra
        // um lado até bater no limite daquele lado, inverte, gira pro outro
        // lado até bater no limite dele, inverte de novo, e assim por diante.
        if (gk_search_going_left_)
        {
            send_goal(turn_left);
            if (robot.neck_pos.position19 >= NECK_LEFT_LIMIT)
            {
                gk_search_going_left_ = false; // bateu no limite esquerdo -> inverte pra direita
            }
        }
        else
        {
            send_goal(turn_right);
            if (robot.neck_pos.position19 <= NECK_RIGHT_LIMIT)
            {
                gk_search_going_left_ = true; // bateu no limite direito -> inverte pra esquerda
            }
        }
        return false;
    }

    if (!goalkeeper_ball_centered())
    {
        // Vê a bola, mas ainda não está alinhada ao centro (M19) -> gira
        // para o lado indicado pelo Vision msg até centralizar.
        if (robot.camera_ball_position.left) send_goal(turn_left);
        else if (robot.camera_ball_position.right) send_goal(turn_right);
        return false;
    }

    // Detectada e centralizada -> bola travada.
    return true;
}

void RobotBehavior::player_penalty()
{
   RCLCPP_INFO(this->get_logger(), "ENTROU AQUI");
   switch (robot.state)
    {
    case searching_ball:
        //RCLCPP_INFO(this->get_logger(), "Searching ball");
        //RCLCPP_ERROR(this->get_logger(), "lost ball timer  %d", lost_ball_timer.delayNR(MAX_LOST_BALL_TIME));

        if(ball_is_locked())
            {   RCLCPP_INFO(this->get_logger(), "ball locked");
                if(robot.ball_position == center) robot.state = ball_approach;      //anda ate a bola
                else robot.state = aligning_with_the_ball;
            }
        else if(lost_ball_timer.delayNR(MAX_LOST_BALL_TIME)) send_goal(turn_left);        //alinha o corpo com a bola
        //else send_goal(gait); // gait
        break;
    
    case aligning_with_the_ball:
        //RCLCPP_INFO(this->get_logger(), "Aligning with the_ball");
        if(robot_align_with_the_ball()) robot.state = ball_approach;
        //else if(ball_is_locked()) robot.state = ball_approach;
        else if(!robot.camera_ball_position.detected) robot.state = searching_ball;
        else if(robot.neck_pos.position19 > 2100) send_goal(turn_left);
        else if(robot.neck_pos.position19 < 1900) send_goal(turn_right);
        break;

    case ball_approach:
        //RCLCPP_ERROR(this->get_logger(), "neck limit %d, ball locked %d, ball close %d", ball_in_close_limit(), ball_is_locked(), robot.camera_ball_position.close);
        //RCLCPP_INFO(this->get_logger(), "ball_approach");
        if((robot.neck_pos.position20 < 1350)& (ball_is_locked())) 
            {
            robot.state = ball_close;
            }         //perdeu a bola
        else if(!robot.camera_ball_position.detected) 
            {
            robot.state = searching_ball;
            } // pode estar bugando
        else if(!robot_align_with_the_ball()) 
            {
            robot.state = aligning_with_the_ball;
            }
        else 
            {
            send_goal(walk);
            }
        break;


    case ball_close:
        //RCLCPP_INFO(this->get_logger(), "ball right %d, ball left %d", robot_align_for_kick_right(), robot_align_for_kick_left());
        //RCLCPP_INFO(this->get_logger(), "ball close");

        if(!robot.camera_ball_position.detected || !robot.camera_ball_position.close) robot.state = searching_ball;
        else if (robot.neck_pos.position20 > 1400) robot.state = aligning_with_the_ball;
        
        if (robot.neck_pos.position19 < 1500)
        {
            send_goal(walk_right);
            RCLCPP_INFO(this->get_logger(), "walking right");
        }
        else if (robot.neck_pos.position19 > 2460)
        {
            send_goal(walk_left);
            RCLCPP_INFO(this->get_logger(), "walking left");
        }
        else
        {
            robot.state = kick_ball;
        }
        //else if(robot_align_for_kick_left()) robot.state = kick_ball;
        break;

    case kick_ball:
        //RCLCPP_INFO(this->get_logger(), "kick");
        if(/*robot.movement != 3 && */robot.movement != 4) 
        {
            // if (robot.neck_pos.position19 >= 2048) 
            //     {   
                    RCLCPP_INFO(this->get_logger(), "kick_left");
                    send_goal(left_kick);
                    robot.state = searching_ball;
                    
                // RCLCPP_INFO(this->get_logger(), "posição do 20: %d", robot.neck_pos.position20);
            //     }
            //     else
            //     {   

            //         RCLCPP_INFO(this->get_logger(), "kick_right");
            //         send_goal(right_kick);
            //         robot.state = searching_ball;
            //     // RCLCPP_INFO(this->get_logger(), "posição do 20: %d", robot.neck_pos.position20);
            // } 
        }
        else if(robot.finished_move)
	    {
            robot.state = searching_ball;
            lost_ball_timer.reset();
        }
        else if(!robot.camera_ball_position.detected || !robot.camera_ball_position.close) robot.state = searching_ball;
        else if (robot.neck_pos.position20 > 1400) robot.state = aligning_with_the_ball;
        //else if(lost_ball_timer.delayNR(2000)) robot.state = searching_ball; //para testar com o corpo desatiavdo
        break;
    }
}

bool RobotBehavior::robot_align_for_kick_left() //fazer
{
    if(ball_in_left_foot()) return true;
    else send_goal(walk_right); 
    return false;
}

bool RobotBehavior::robot_align_for_kick_right()
{
    if(ball_in_right_foot()) return true;
    else send_goal(walk_left); 
    return false;
}

bool RobotBehavior::ball_in_right_foot()
{
    if(ball_is_locked() && robot.neck_pos.position19 < 1850) return true;
    return false;
}

bool RobotBehavior::ball_in_left_foot()
{
    if(ball_is_locked() && robot.neck_pos.position19 > 2250) return true;
    return false;
}


bool RobotBehavior::is_goalkeeper(int robot_num)
{
    return robot_num == 1;
}

bool RobotBehavior::is_kicker(int robot_num)
{
    return robot_num == 2;
    RCLCPP_FATAL(this->get_logger(), "kicker");
}

bool RobotBehavior::is_bala(int robot_num)
{
    return robot_num == 3;
    RCLCPP_FATAL(this->get_logger(), "bala" );
}

bool RobotBehavior::goalkeeper_align_with_the_ball()
{
    if(vision_stable())
    {
        if(neck_to_left()) send_goal(walk_left);
        else if(neck_to_right()) send_goal(walk_right);
        else return true;
    }
    return false;
}

bool RobotBehavior::robot_align_with_the_ball()
{
    if(vision_stable())
    {
        if(robot.state == ball_approach) return centered_neck();
        else return full_centered_neck();
    }
    else return false;
}

void RobotBehavior::turn_to_ball()
{
    if(robot.ball_position == left) send_goal(turn_left);
    else if(robot.ball_position == right) send_goal(turn_right);     
    // Melhorar essa logica

    // if(robot.ball_position == right) send_goal(turn_right);
}

bool RobotBehavior::full_centered_neck()
{
    return abs(robot.neck_pos.position19 - NECK_TILT_CENTER) < 50;
}


bool RobotBehavior::centered_neck() // feito
{    
    return !neck_to_left() && !neck_to_right();
}

bool RobotBehavior::ball_is_locked() // feito 
{
    if(robot.camera_ball_position.detected)
    {
        lost_ball_timer.reset();
        if(vision_stable()) return true;
    }
    return false;
}

bool RobotBehavior::goalpost_is_locked() //fazer
{
    if(robot.camera_ball_position.detected)
    {
        lost_ball_timer.reset();
        if(vision_stable()) return true;
    }
    return false;
}

bool RobotBehavior::vision_stable() // feito   
{
    if(ball_in_camera_center() || ball_in_robot_limits())
    {
        detect_ball_position();
        return true;
    }
    return false;
}

void RobotBehavior::detect_ball_position() // Funciona
{
    // RCLCPP_DEBUG(this->get_logger(), "debug 1: centered_neck %d", centered_neck());
    // RCLCPP_DEBUG(this->get_logger(), "debug 1: neck_to_left %d", neck_to_left());
    // RCLCPP_DEBUG(this->get_logger(), "debug 1: neck_to_right %d", neck_to_right());
    if(robot.state == aligning_with_the_ball)
    {
        if(robot.neck_pos.position19 > NECK_TILT_CENTER) robot.ball_position = left;
        else robot.ball_position = right;
    }
    else
    {
        if(centered_neck()) robot.ball_position = center;
        if(neck_to_left()) robot.ball_position = left;
        if(neck_to_right()) robot.ball_position = right;
        RCLCPP_DEBUG(this->get_logger(), "ball side %d", robot.ball_position);
    }

}

bool RobotBehavior::neck_to_right() // testar
{
    return NECK_RIGHT_TH > robot.neck_pos.position19;
}

bool RobotBehavior::neck_to_left() // testar
{
    RCLCPP_DEBUG(this->get_logger(), "Neck left th: %d\nrobot neck pos: %d", NECK_LEFT_TH, robot.neck_pos.position19); 
    return NECK_LEFT_TH < robot.neck_pos.position19;
}

bool RobotBehavior::ball_in_robot_limits() // feito
{
    if(robot.camera_ball_position.left  && ball_in_left_limit())     return true;
    if(robot.camera_ball_position.right && ball_in_right_limit())    return true;
    if(robot.camera_ball_position.close && ball_in_close_limit())    return true;
    return false;
}

bool RobotBehavior::ball_in_close_limit() // feito
{
    return abs(robot.neck_pos.position20 - NECK_CLOSE_LIMIT) < LIMIT_TH;
}

bool RobotBehavior::ball_in_right_limit() // feito
{
    return abs(robot.neck_pos.position19 - NECK_RIGHT_LIMIT) < LIMIT_TH;
}

bool RobotBehavior::ball_in_left_limit() // feito 
{
    return abs(robot.neck_pos.position19 - NECK_LEFT_LIMIT) < LIMIT_TH;
}

bool RobotBehavior::ball_in_camera_center() // feito
{
    return (robot.camera_ball_position.center) && robot.camera_ball_position.med;
}

bool RobotBehavior::is_penalized() // feito
{
    if(gc_info.penalized)
    {
        RCLCPP_DEBUG(this->get_logger(), "Robot Penalized, remain %d seconds", gc_info.seconds_till_unpenalized);

        if(gc_info.seconds_till_unpenalized < 5)
        {
            RCLCPP_DEBUG(this->get_logger(), "Preparing to return, gait started");
            send_goal(gait);
        } 
        else
        {
            send_goal(stand_still);
        }
        return true;
    }
    return false;
}

void RobotBehavior::get_up() // feito
{
    RCLCPP_DEBUG(this->get_logger(), "Robot Fallen");

    switch (robot.fall)
    {
    case FallenFront:
        RCLCPP_DEBUG(this->get_logger(), "Stand up front");
        send_goal(stand_up_front);
        break;
    
    case FallenBack:
        RCLCPP_DEBUG(this->get_logger(), "Stand up back");
        send_goal(stand_up_back);
        break;

    default:
        break;
    }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto decision_node = std::make_shared<RobotBehavior>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(decision_node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
