#include <memory>
#include <algorithm>

#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "ros2_aruco_interfaces/srv/get_map_data.hpp"  // Inclusione del servizio

using namespace std::chrono_literals;

class Detect : public plansys2::ActionExecutorClient
{
public:
  Detect()
  : plansys2::ActionExecutorClient("detect", 1s), progress_(0.0)
  {
    // Inizializza il client di servizio
    service_client_ = this->create_client<ros2_aruco_interfaces::srv::GetMapData>("/get_map_data");
  }

private:
  void do_work()
  {
    if (progress_ == 0.0) {
      // Chiamata al servizio alla prima iterazione
      if (!service_client_->wait_for_service(1s)) {
        RCLCPP_ERROR(this->get_logger(), "Service /get_map_data not available");
        finish(false, 0.0, "Service not available");
        return;
      }

      auto request = std::make_shared<ros2_aruco_interfaces::srv::GetMapData::Request>();
      
      auto garbage = service_client_->async_send_request(request,
        [this](rclcpp::Client<ros2_aruco_interfaces::srv::GetMapData>::SharedFuture response) {
          try {
            auto result = response.get();
            //RCLCPP_INFO(this->get_logger(), "junk del servizio: x=%.2f, y=%.2f, marker_id=%.2f", 
            //      result->x, result->y, result->marker_id);
            //RCLCPP_INFO(this->get_logger(), "Received map data: %s", result->map_data.c_str());
          } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service: %s", e.what());
            finish(false, 0.0, "Service call failed");
          }
        });

        


    }

    // Simulazione di un processo di rilevamento
    if (progress_ < 1.0) {
      progress_ += 0.05;  // Aumenta il progresso del 5% ad ogni chiamata
      send_feedback(progress_, "Detect running");  // Invia feedback con il progresso
    } else {
            
      auto request = std::make_shared<ros2_aruco_interfaces::srv::GetMapData::Request>();

      auto future = service_client_->async_send_request(request,
        [this](rclcpp::Client<ros2_aruco_interfaces::srv::GetMapData>::SharedFuture response) {
          try {
            auto result = response.get();
            RCLCPP_INFO(this->get_logger(), "Risultato del servizio: x=%.2f, y=%.2f, marker_id=%ld", 
                  result->x, result->y, result->marker_id);
            //RCLCPP_INFO(this->get_logger(), "Received map data: %s", result->map_data.c_str());
          } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to call service: %.2f", e.what());
            finish(false, 0.0, "Service call failed");
          }
        });
      finish(true, 1.0, "Detect completed");  // Completa l'azione quando il progresso è 1.0

      progress_ = 0.0;  // Reset del progresso
      std::cout << std::endl;
    }

    // Stampa il progresso a schermo
    std::cout << "\r\e[K" << std::flush;
    std::cout << "Detecting ... [" << std::min(100.0, progress_ * 100.0) << "%]  " <<
      std::flush;
  }

  float progress_;  // Variabile per tenere traccia del progresso dell'azione
  rclcpp::Client<ros2_aruco_interfaces::srv::GetMapData>::SharedPtr service_client_;  // Client per il servizio
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);  // Inizializza il sistema ROS 2
  auto node = std::make_shared<Detect>();  // Crea un oggetto di tipo Detect

  node->set_parameter(rclcpp::Parameter("action_name", "detect"));  // Imposta il nome dell'azione
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);  // Esegue la transizione di configurazione

  rclcpp::spin(node->get_node_base_interface());  // Inizia l'esecuzione del nodo

  rclcpp::shutdown();  // Spegne il sistema ROS 2

  return 0;
}

