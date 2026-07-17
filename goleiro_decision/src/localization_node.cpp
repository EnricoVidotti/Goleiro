#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "custom_interfaces/msg/landmark_count.hpp" // novo msg: contagem de landmarks (l/t/x/goalpost)

#include <deque>
#include <algorithm>

// -----------------------------------------------------------------------------------
// LOCALIZAÇÃO - Robô Goleiro
//
// Esse node interpreta o que o vision_pkg está enxergando em campo: ele assina a
// contagem de landmarks publicada por frame ("L", "T", "X" e traves, nos tópicos
// /l_count, /t_count, /x_count e /goalpost_count), aplica um filtro pra reduzir
// ruído de detecção, mantém um contador por tipo de landmark e, a cada 60 frames,
// publica um resumo consolidado no tópico "robot_behavior" pro robot_behavior
// usar na tomada de decisão.
//
// (Antigo pacote "posicao" / src/main.cpp, fundido em goleiro_decision.)
// -----------------------------------------------------------------------------------

// intervalo de frames entre cada publish do resumo de landmarks
#define FRAME_WINDOW 60

// tamanho da janela do filtro de mediana usado pra suavizar ruído de detecção
#define FILTER_WINDOW 3

using namespace std::placeholders;

// -----------------------------------------------------------------------------------
// Filtro simples de mediana. Cada leitura de contagem (L, T, X ou trave) passa por
// esse filtro antes de virar o valor "atual" daquele landmark. Isso evita que um pico
// ou uma falha isolada de detecção (ex.: o YOLO perdeu o landmark por 1 frame) faça
// o valor publicado oscilar à toa.
//
// IMPORTANTE: o valor filtrado é sobrescrito a cada frame, nunca somado. O node não
// soma as leituras dos 60 frames da janela — se fizesse isso, um único L parado na
// frente do robô por 60 frames seguidos apareceria como "60 L's" no publish, quando
// na real é só 1 L sendo visto o tempo todo. O que é publicado a cada 60 frames é uma
// "foto" do que está sendo visto naquele instante (já suavizada pelo filtro), não um
// acúmulo histórico.
// -----------------------------------------------------------------------------------
class MedianFilter
{
public:
    int filter(int new_value)
    {
        buffer_.push_back(new_value);
        if (buffer_.size() > FILTER_WINDOW)
        {
            buffer_.pop_front();
        }

        std::deque<int> sorted_buffer(buffer_);
        std::sort(sorted_buffer.begin(), sorted_buffer.end());

        return sorted_buffer[sorted_buffer.size() / 2]; // mediana
    }

private:
    std::deque<int> buffer_;
};

class LocalizationNode : public rclcpp::Node
{
public:
    LocalizationNode() : Node("localization_node")
    {
        // vision_pkg publica, a cada frame processado, a quantidade de cada landmark
        // detectada naquele frame nos tópicos abaixo (std_msgs/Int32).
        l_count_subscriber_ = this->create_subscription<std_msgs::msg::Int32>("/l_count", 10,
            std::bind(&LocalizationNode::listener_callback_l_count, this, _1));

        t_count_subscriber_ = this->create_subscription<std_msgs::msg::Int32>("/t_count", 10,
            std::bind(&LocalizationNode::listener_callback_t_count, this, _1));

        x_count_subscriber_ = this->create_subscription<std_msgs::msg::Int32>("/x_count", 10,
            std::bind(&LocalizationNode::listener_callback_x_count, this, _1));

        goalpost_count_subscriber_ = this->create_subscription<std_msgs::msg::Int32>("/goalpost_count", 10,
            std::bind(&LocalizationNode::listener_callback_goalpost_count, this, _1));

        // publisher que manda o resumo de landmarks pro robot_behavior a cada 60 frames
        landmark_summary_publisher_ = this->create_publisher<custom_interfaces::msg::LandmarkCount>(
            "robot_behavior", 10);

        RCLCPP_INFO(this->get_logger(), "Localization node has started");
    }

private:

    // Cada callback de landmark só atualiza o valor filtrado "atual" daquele tipo
    // (sobrescreve, nunca soma). Usa /l_count como "relógio" de frame (tick_frame),
    // já que o vision_pkg publica esse tópico uma vez por frame processado, mesmo
    // quando nenhum L é detectado no frame.

    void listener_callback_l_count(const std_msgs::msg::Int32::SharedPtr msg)
    {
        l_current_ = l_filter_.filter(msg->data);
        tick_frame();
    }

    void listener_callback_t_count(const std_msgs::msg::Int32::SharedPtr msg)
    {
        t_current_ = t_filter_.filter(msg->data);
    }

    void listener_callback_x_count(const std_msgs::msg::Int32::SharedPtr msg)
    {
        x_current_ = x_filter_.filter(msg->data);
    }

    void listener_callback_goalpost_count(const std_msgs::msg::Int32::SharedPtr msg)
    {
        goalpost_current_ = goalpost_filter_.filter(msg->data);
    }

    // Chamado uma vez por frame de visão. A cada FRAME_WINDOW (60) frames,
    // publica uma "foto" do que está sendo visto no momento (valores já filtrados),
    // sem acumular contagem de frames passados.
    void tick_frame()
    {
        frame_counter_++;

        if (frame_counter_ >= FRAME_WINDOW)
        {
            publish_landmark_summary();
            frame_counter_ = 0;
        }
    }

    void publish_landmark_summary()
    {
        auto summary = custom_interfaces::msg::LandmarkCount();
        summary.l_count = l_current_;
        summary.t_count = t_current_;
        summary.x_count = x_current_;
        summary.goalpost_count = goalpost_current_;

        landmark_summary_publisher_->publish(summary);

        RCLCPP_INFO(this->get_logger(),
            "[LANDMARKS] L=%d | T=%d | X=%d | Traves=%d (a cada %d frames)",
            summary.l_count, summary.t_count, summary.x_count, summary.goalpost_count, FRAME_WINDOW);
    }

    // ---------------- Subscribers / Publisher ----------------

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr l_count_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr t_count_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr x_count_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr goalpost_count_subscriber_;
    rclcpp::Publisher<custom_interfaces::msg::LandmarkCount>::SharedPtr landmark_summary_publisher_;

    // ---------------- Estado / contadores ----------------

    int frame_counter_ = 0;

    int l_current_ = 0;
    int t_current_ = 0;
    int x_current_ = 0;
    int goalpost_current_ = 0;

    MedianFilter l_filter_;
    MedianFilter t_filter_;
    MedianFilter x_filter_;
    MedianFilter goalpost_filter_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LocalizationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
