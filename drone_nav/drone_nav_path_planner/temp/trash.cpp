#include <rclcpp/rclcpp.hpp>
#include <>


class localOccupancyPublisher : public rclcpp::Node     
{
public:
  localOccupancyPublisher() : Node("localoccupancypublishernode")
  {

  }
private:

};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);                       

  rclcpp::spin(std::make_shared<localOccupancyPublisher>());  

  rclcpp::shutdown();                               

  return 0;
}