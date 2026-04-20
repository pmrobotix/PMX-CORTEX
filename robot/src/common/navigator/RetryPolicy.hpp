#ifndef COMMON_NAVIGATOR_RETRYPOLICY_HPP_
#define COMMON_NAVIGATOR_RETRYPOLICY_HPP_

/*!
 * \brief Parametres de la boucle de retry pour les deplacements.
 */
struct RetryPolicy
{
    //! Temps d'attente entre chaque tentative (microsecondes)
    int waitTempoUs = 2000000;

    //! Nombre max de tentatives sur detection obstacle
    int maxObstacleRetries = 2;

    //! Nombre max de tentatives sur collision mecanique
    int maxCollisionRetries = 2;

    //! Distance de recul en mm apres detection obstacle (0 = pas de recul)
    int reculObstacleMm = 0;

    //! Distance de recul en mm apres collision (0 = pas de recul)
    int reculCollisionMm = 0;

    //! Ignorer les collisions (ne pas interrompre)
    bool ignoreCollision = false;

    // --- Presets ---

    //! Pas de retry : 1 seul essai
    static RetryPolicy noRetry()
    {
        return { 0, 1, 1, 0, 0, false };
    }

    //! Retry par defaut (2 essais obstacle, 2 essais collision)
    static RetryPolicy standard()
    {
        return { 2000000, 2, 2, 0, 0, false };
    }

    //! Retry agressif pour les actions critiques (5 essais, recul 50mm)
    static RetryPolicy aggressive()
    {
        return { 2000000, 5, 5, 50, 20, false };
    }

    //! Retry patient pour fin de match (20 essais obstacle, 10 collision)
    static RetryPolicy patient()
    {
        return { 2000000, 20, 10, 0, 0, false };
    }

    //! Retry rapide pour tests automatises : 500ms entre essais, 2 tentatives.
    //! Permet de tester la boucle retry sans payer 2s d'attente par essai.
    static RetryPolicy quickTest()
    {
        return { 500000, 2, 2, 0, 0, false };
    }
};

#endif
