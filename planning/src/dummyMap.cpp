#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sstream>

class DummyMapNode : public rclcpp::Node {
public:
    DummyMapNode() : Node("dummy_map_node") {
        publisher_ = this->create_publisher<std_msgs::msg::String>("matrix_topic", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&DummyMapNode::publish_matrix, this)
        );
    }

private:
    void publish_matrix() {
        // Crea una rappresentazione della matrice trasposta 4x3 come stringa
        std::ostringstream matrix_stream;

        matrix_stream << "[\n";  // Iniziamo con l'apertura della matrice

        // Aggiungi 4 righe di 3 elementi ciascuna (trasposta della matrice originale 3x4)
        add_vector(matrix_stream, 0, 0, 0);
        add_vector(matrix_stream, 0, 0, 0);
        add_vector(matrix_stream, 0, 0, 0);
        add_vector(matrix_stream, 0, 0, 0);

        matrix_stream << "\n]";  // Chiusura della matrice

        // Crea il messaggio e pubblichiamo la matrice come stringa
        auto message = std_msgs::msg::String();
        message.data = matrix_stream.str();  // La matrice come stringa formattata
        RCLCPP_INFO(this->get_logger(), "Publishing: %s", message.data.c_str());
        publisher_->publish(message);
    }

    void add_vector(std::ostringstream &matrix_stream, double a, double b, double c) {
        matrix_stream << "    [" << a << ", " << b << ", " << c << "],\n";  // Aggiungi ogni vettore
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DummyMapNode>());
    rclcpp::shutdown();
    return 0;
}

