#ifndef COMMON_IA_FLAG_MANAGER_HPP_
#define COMMON_IA_FLAG_MANAGER_HPP_

#include <set>
#include <string>

#include "log/LoggerFactory.hpp"

/*!
 * \brief Ensemble nomme de flags booleens utilises par le runner de strategie JSON.
 *
 * Le runner utilise ces flags pour :
 * - skipper une instruction ou une task si son `needed_flag` n'est pas actif
 * - lever un flag apres succes d'une instruction (`action_flag`)
 * - effacer des flags apres succes d'une instruction (`clear_flags`)
 *
 * Voir robot/md/STRATEGY_JSON_FORMAT.md sec. 3.
 */
class FlagManager
{
public:
    void set(const std::string& flag);
    void clear(const std::string& flag);
    bool has(const std::string& flag) const;
    void clearAll();

    size_t size() const { return flags_.size(); }
    const std::set<std::string>& all() const { return flags_; }

private:
    static inline const logs::Logger& logger()
    {
        static const logs::Logger& instance = logs::LoggerFactory::logger("FlagManager");
        return instance;
    }

    std::set<std::string> flags_;
};

#endif
