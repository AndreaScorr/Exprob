import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from std_msgs.msg import String
from ros2_aruco_interfaces.srv import GetMapData

class MapDataService(Node):

    def __init__(self):
        super().__init__('map_data_service')

        # Iniziamo a sottoscrivere il topic /map_topic
        self.map_data = ""  # Memorizziamo l'ultimo messaggio ricevuto

        # Creiamo un publisher per il topic /map_topic (opzionale, se vuoi testare)
        self.publisher = self.create_publisher(String, 'map_topic', 10)
        
        # Sottoscriviamo al topic /map_topic
        self.create_subscription(
            String,
            'map_topic',
            self.map_callback,
            10
        )

        # Creiamo il servizio
        self.srv = self.create_service(
            GetMapData,
            'get_map_data',
            self.get_map_data_callback
        )

    def map_callback(self, msg: String):
        """Callback per memorizzare l'ultimo messaggio ricevuto."""
        self.map_data = msg.data
        self.get_logger().info(f"Received map data: {self.map_data}")

    def get_map_data_callback(self, request, response):
        """Callback per il servizio. Restituisce i dati mappa."""
        if self.map_data:
            response.map_data = String()  # Creiamo un oggetto String
            response.map_data.data = self.map_data  # Assegniamo la stringa al campo 'data'
            self.get_logger().info(f"Sending map data: {self.map_data}")
        else:
            response.map_data = String()  # Creiamo un oggetto String
            response.map_data.data = "No data received yet."  # Assegniamo un messaggio predefinito
            self.get_logger().info("No map data received yet.")
        return response

def main(args=None):
    rclpy.init(args=args)

    map_data_service = MapDataService()

    # Eseguiamo il nodo
    rclpy.spin(map_data_service)

    # Quando il nodo è fermato
    map_data_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

