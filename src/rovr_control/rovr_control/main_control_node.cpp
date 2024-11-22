#include "behaviortree_cpp/bt_factory.h"

using namespace BT;

int main() {
    BehaviorTreeFactory factory;

    auto tree = factory.createTreeFromText("../../../../config/behavior_trees/main_tree.xml");
    tree.tickWhileRunning();
    return 0;
}