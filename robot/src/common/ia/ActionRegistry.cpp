#include "ActionRegistry.hpp"

void ActionRegistry::registerAction(const std::string& name, ActionFn fn)
{
    if (actions_.count(name) > 0) {
        logger().info() << "registerAction: overriding action '" << name << "'" << logs::end;
    }
    actions_[name] = std::move(fn);
}

bool ActionRegistry::call(const std::string& name) const
{
    auto it = actions_.find(name);
    if (it == actions_.end()) {
        logger().error() << "call: action '" << name << "' not registered" << logs::end;
        return false;
    }
    logger().info() << "call: '" << name << "'" << logs::end;
    try {
        return it->second();
    } catch (const std::exception& e) {
        logger().error() << "call: exception in '" << name << "': " << e.what() << logs::end;
        return false;
    }
}

bool ActionRegistry::has(const std::string& name) const
{
    return actions_.find(name) != actions_.end();
}
