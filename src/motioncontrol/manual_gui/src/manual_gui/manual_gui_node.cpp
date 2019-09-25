#include "../../include/manual_gui_core.hpp"

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "manual_gui_node");

    manual_gui::manual_gui_demo demo(ros::NodeHandle(), ros::NodeHandle("~"));
    demo.run();

    return 0;
}
