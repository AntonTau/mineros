#include "my_first_behaviourtree/move_to.hpp"
#include "my_first_behaviourtree/current_position.hpp"
#include "my_first_behaviourtree/determine_next_position.hpp"

#include <rclcpp/rclcpp.hpp>

class MyFirstBehaviourTree : public rclcpp::Node {
    public:
        MyFirstBehaviourTree() : Node("my_first_behaviour_tree_node") {
            auto nhMove = std::make_shared<rclcpp::Node>("move_to_node");
            auto nhCurrentPos = std::make_shared<rclcpp::Node>("current_position_node");
            
            BT::BehaviorTreeFactory factory;
            factory.registerNodeType<DetermineNextPosition>("DetermineNextPosition");
            factory.registerNodeType<MoveTo>("MoveTo", BT::RosNodeParams(nhMove, "/mineros/set_position"));
            factory.registerNodeType<CurrentPosition>("CurrentPosition", BT::RosNodeParams(nhCurrentPos, "/mineros/local_position/pose"));

            BT::Tree tree = factory.createTreeFromFile("src/my_first_behaviourtree/trees/test_tree.xml");
            RCLCPP_INFO(this->get_logger(), "Behavior Tree successfully created");

            tree.tickWhileRunning();
            
        }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MyFirstBehaviourTree>());
    rclcpp::shutdown();
    return 0;
}