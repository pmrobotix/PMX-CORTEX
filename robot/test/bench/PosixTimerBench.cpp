/*!
 * \file
 * \brief Benchmark de precision des timers POSIX.
 */

#include "PosixTimerBench.hpp"

#include <chrono>
#include <ctime>
#include <vector>
#include <thread>

#include "timer/ActionManagerPosixTimer.hpp"
#include "timer/ITimerPosixListener.hpp"
#include "utils/Chronometer.hpp"
#include "log/Logger.hpp"
#include "log/LoggerFactory.hpp"

using namespace utils;

// Lecture timestamp CLOCK_MONOTONIC en us (insensible aux sauts NTP).
static inline long monotonic_us() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long) ts.tv_sec * 1000000L + (long) (ts.tv_nsec / 1000);
}

// --- Mock ---

class TimingPosixTimer: public ITimerPosixListener {
private:
    std::vector<long> timestamps_us_;
    utils::Mutex mtimestamps_;

public:
    TimingPosixTimer(std::string label, int time_ms) {
        this->init(label, time_ms * 1000);
    }

    virtual ~TimingPosixTimer() {}

    void onTimer(utils::Chronometer chrono) {
        long now = monotonic_us();
        mtimestamps_.lock();
        timestamps_us_.push_back(now);
        mtimestamps_.unlock();
    }

    void onTimerEnd(utils::Chronometer chrono) {}

    std::vector<long> getIntervals() {
        std::vector<long> intervals;
        mtimestamps_.lock();
        for (size_t i = 1; i < timestamps_us_.size(); i++) {
            intervals.push_back(timestamps_us_[i] - timestamps_us_[i - 1]);
        }
        mtimestamps_.unlock();
        return intervals;
    }

    size_t count() {
        mtimestamps_.lock();
        size_t s = timestamps_us_.size();
        mtimestamps_.unlock();
        return s;
    }
};

// --- Utilitaire ---

static void runBench(test::PosixTimerBench *self, const char *label, int intervalMs, int durationMs)
{
    static const logs::Logger &log = logs::LoggerFactory::logger("test::PosixTimerBench");

    ActionManagerPosixTimer manager;
    TimingPosixTimer *pt = new TimingPosixTimer(label, intervalMs);

    manager.addTimer(pt);
    manager.start(label, 2);

    std::this_thread::sleep_for(std::chrono::milliseconds(durationMs));

    manager.stopAllPTimers();
    manager.stop();

    std::vector<long> intervals = pt->getIntervals();
    size_t nbSamples = intervals.size();

    long expectedUs = intervalMs * 1000L;

    log.info() << "=== " << label << " ===" << logs::end;
    log.info() << "Intervalle attendu: " << expectedUs << " us" << logs::end;
    log.info() << "Samples collectes: " << nbSamples << logs::end;

    if (nbSamples >= 2) {
        long sum = 0;
        long minObs = intervals[0];
        long maxObs = intervals[0];
        for (auto dt : intervals) {
            sum += dt;
            if (dt < minObs) minObs = dt;
            if (dt > maxObs) maxObs = dt;
        }
        long avgUs = sum / (long) intervals.size();
        long jitterMax = (maxObs - minObs);
        double errPercent = ((double)(avgUs - expectedUs) / (double) expectedUs) * 100.0;

        log.info() << "Moyenne:    " << avgUs << " us  (erreur: "
                << (errPercent >= 0 ? "+" : "") << (int) errPercent << "%)" << logs::end;
        log.info() << "Min:        " << minObs << " us" << logs::end;
        log.info() << "Max:        " << maxObs << " us" << logs::end;
        log.info() << "Jitter max: " << jitterMax << " us" << logs::end;
    }

    delete pt;
}

// --- Suite ---

void test::PosixTimerBench::suite() {
    benchTimer100ms();
    benchTimer10ms();
    benchTimer1ms();
}

void test::PosixTimerBench::benchTimer100ms() {
    logger().info() << "benchTimer100ms..." << logs::end;
    runBench(this, "timer_100ms", 100, 2000);
    logger().info() << "benchTimer100ms OK" << logs::end;
}

void test::PosixTimerBench::benchTimer10ms() {
    logger().info() << "benchTimer10ms..." << logs::end;
    runBench(this, "timer_10ms", 10, 2000);
    logger().info() << "benchTimer10ms OK" << logs::end;
}

void test::PosixTimerBench::benchTimer1ms() {
    logger().info() << "benchTimer1ms..." << logs::end;
    runBench(this, "timer_1ms", 1, 2000);
    logger().info() << "benchTimer1ms OK" << logs::end;
}
