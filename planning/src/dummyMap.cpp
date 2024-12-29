#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sstream>

class DummyMapNode : public rclcpp::Node {
public:
    DummyMapNode() : Node("dummy_map_node") {
        publisher_ = this->create_publisher<std_msgs::msg::String>("map_topic", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&DummyMapNode::publish_map, this)
        );
    }

private:
    void publish_map() {
        // Crea una rappresentazione di dati simile a JSON
        std::ostringstream json_stream;
        json_stream << "{";
        json_stream << "\"waypoints\": [";

        // Aggiungi alcuni waypoint
        add_waypoint(json_stream, "1", 1.0, 2.0, 3.0);
        add_waypoint(json_stream, "2", 4.0, 5.0, 6.0);
	 add_waypoint(json_stream, "3", 1.0, 2.0, 3.0);
        add_waypoint(json_stream, "4", 4.0, 5.0, 6.0);

        json_stream << "]";
        json_stream << "}";

        auto message = std_msgs::msg::String();
        message.data = json_stream.str();
        RCLCPP_INFO(this->get_logger(), "Publishing: %s", message.data.c_str());
        publisher_->publish(message);
    }

    void add_waypoint(std::ostringstream &json_stream, const std::string &name, double x, double y, double z) {
        static bool first = true;
        if (!first) {
            json_stream << ",";
        }
        first = false;

        json_stream << "{";
        json_stream << "\"name\": \"" << name << "\", ";
        json_stream << "\"x\": " << x << ", ";
        json_stream << "\"y\": " << y << ", ";
        json_stream << "\"z\": " << z;
        json_stream << "}";
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

