#include <gazebo/gazebo_client.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

int main(int argc, char **argv) {
    // Inizializza Gazebo
    gazebo::client::setup(argc, argv);
    printf("Client setup");

    // Crea un nodo di trasporto
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();
    printf("Initialized node");

    // Crea un publisher
    gazebo::transport::PublisherPtr pub = node->Advertise<gazebo::msgs::GzString>("~/polygon/join");

    // Attendi un po' per stabilire la connessione
    pub->WaitForConnection();
    printf("Connected");

    // Invia un messaggio di comando
    gazebo::msgs::GzString msg;
    msg.set_data("join");
    pub->Publish(msg);
    printf("Message published");

    // Chiudi Gazebo
    gazebo::client::shutdown();
    return 0;
}