#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <rr_estimator/rr_estimator_component.hpp>

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<abu2023::RREstimator>());
    rclcpp::shutdown();
    return 0;
}