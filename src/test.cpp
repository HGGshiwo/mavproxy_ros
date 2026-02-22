#include <iostream>
#include <core.hpp>
#include <components/ros.hpp>

class MyCallbackManager : public CallbackManager<MyCallbackManager>
{
public:
    MyCallbackManager(const std::vector<std::shared_ptr<ComponentConfig>> &config_list)
        : CallbackManager(config_list)
    {

        // 调用处：此时ros.topic(...)返回的是可调用的Lambda，嵌套()调用合法
        _ros.topic<std_msgs::String>("chatter", 10, 1)(
            [this](std_msgs::String data) -> void
            {
                std::cout << "\n[MyCallbackManager] ROS topic callback triggered" << std::endl;
                std::cout << "  - Data: " << data.data << std::endl;
            });
    }
};

int main(int argc, char **argv)
{

    ROSConfig ros_cfg;
    std::vector<std::shared_ptr<ComponentConfig>> config_list = {std::make_shared<ROSConfig>()};

    ros::init(argc, argv, "ros_event_callback");
    auto manager = MyCallbackManager::create(config_list);  // 保持生命周期
    ros::spin();
    return 0;
}