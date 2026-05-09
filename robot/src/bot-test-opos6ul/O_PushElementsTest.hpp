#ifndef O_PUSH_ELEMENTS_TEST_HPP
#define O_PUSH_ELEMENTS_TEST_HPP

#include "utils/FunctionalTest.hpp"
#include "log/LoggerFactory.hpp"

/*!
 * \brief Test du module push_elements (mode dual).
 *
 * MODE 1 - Validation logique (sans args)
 *   ./bot-opos6ul t /n <num>
 *   ./bot-opos6ul t /k pe
 *   Itere sur ~30 cas (BLEU + YELLOW x P1/P3/P4/P14 x idx 0..5) et verifie
 *   la mecanique de combinaison (mapping idx, swap couleur SWAP_COLOR_IDX,
 *   flip suffixe horizontal, offsets P4/P14, garde-fou dist<=0).
 *   Les attendus sont DERIVES des constantes (distDirecteAt, distInverseAt,
 *   zoneOffsetFor) -> le test reste vert quel que soit le reglage de
 *   D1..D4 (la calibration physique se fait sur table reelle, cf doc).
 *
 * MODE 2 - Poussage reel (3 args obligatoires)
 *   ./bot-opos6ul t /n <num> P14 D 3
 *   ./bot-opos6ul t /k pe P1 H 0 /y
 *   Force le wrapper push_elements_<zone>_<X> avec idx=<value>, lance la
 *   manipulation runtime (le robot pousse vraiment). Idem CLI /u <Pn_X>
 *   <value> mais via test isole, sans charger une strategie JSON.
 *
 * Args mode 2 :
 *   argv[1] = zone     (P1..P14)
 *   argv[2] = suffixe  (B|H|D|G)
 *   argv[3] = idx      (0..5)
 *
 * Cf robot/md/PUSH_ELEMENTS_2026.md.
 */
class O_PushElementsTest : public FunctionalTest
{
public:
    O_PushElementsTest();
    virtual ~O_PushElementsTest() {}

    std::string usageHelp() const override
    {
        return
            "        args: <zone> <suffixe> <idx>  (mode poussage reel) ou aucun (mode validation)\n"
            "        ex:   /k pe                   # mode validation logique : ~30 cas, PASS/FAIL\n"
            "        ex:   /k pe P14 D 3           # mode poussage reel : push_elements_P14_D idx=3\n"
            "        ex:   /k pe P1 H 0 /y         # idem en YELLOW (passe /y au binaire)";
    }

    void configureConsoleArgs(int argc, char** argv) override;
    virtual void run(int argc, char** argv) override;

private:
    static inline const logs::Logger& logger()
    {
        static const logs::Logger& instance = logs::LoggerFactory::logger("O_PushElementsTest");
        return instance;
    }

    /// Mode 1 : iterate sur les cas de validation logique.
    void runValidation();

    /// Mode 2 : invoque push_elements_zone() avec parsing argv.
    void runPushReel(const char* zone, const char* suffixe, int idx);
};

#endif
