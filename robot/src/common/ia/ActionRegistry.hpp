#ifndef COMMON_IA_ACTION_REGISTRY_HPP_
#define COMMON_IA_ACTION_REGISTRY_HPP_

#include <functional>
#include <string>
#include <unordered_map>

#include "log/LoggerFactory.hpp"

/*!
 * \brief Registre nom -> callback pour les tasks MANIPULATION du runner JSON.
 *
 * Le runner appelle call(action_id) quand il rencontre une task
 * { "type": "MANIPULATION", "action_id": "banderole" }.
 *
 * Politique sur action inconnue : call() retourne false. Le runner decide
 * ensuite si c'est une erreur fatale ou juste un warning.
 */
class ActionRegistry
{
public:
    using ActionFn = std::function<bool()>;

    /*!
     * \brief Enregistre une action par nom. Remplace si deja present.
     */
    void registerAction(const std::string& name, ActionFn fn);

    /*!
     * \brief Appelle l'action. Retourne false si inconnue ou si fn retourne false.
     */
    bool call(const std::string& name) const;

    /*!
     * \brief True si une action est enregistree sous ce nom.
     */
    bool has(const std::string& name) const;

    size_t size() const { return actions_.size(); }

private:
    static inline const logs::Logger& logger()
    {
        static const logs::Logger& instance = logs::LoggerFactory::logger("ActionRegistry");
        return instance;
    }

    std::unordered_map<std::string, ActionFn> actions_;
};

#endif
