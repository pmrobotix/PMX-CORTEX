#include "FlagManager.hpp"

void FlagManager::set(const std::string& flag)
{
    if (flag.empty()) return;
    auto r = flags_.insert(flag);
    if (r.second) {
        logger().info() << "set '" << flag << "'" << logs::end;
    }
}

void FlagManager::clear(const std::string& flag)
{
    if (flag.empty()) return;
    if (flags_.erase(flag) > 0) {
        logger().info() << "clear '" << flag << "'" << logs::end;
    }
}

bool FlagManager::has(const std::string& flag) const
{
    if (flag.empty()) return false;
    return flags_.find(flag) != flags_.end();
}

void FlagManager::clearAll()
{
    if (!flags_.empty()) {
        logger().info() << "clearAll (" << flags_.size() << " flags)" << logs::end;
    }
    flags_.clear();
}
