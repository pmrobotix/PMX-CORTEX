/*!
 * \file
 * \brief Benchmark de precision du scheduler actuel (ActionTimerScheduler).
 */

#include "ActionTimerSchedulerBench.hpp"

#include <chrono>
#include <ctime>
#include <thread>
#include <vector>

#include "timer/ActionTimerScheduler.hpp"
#include "timer/ITimerScheduledListener.hpp"
#include "thread/Mutex.hpp"
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

class TimingScheduledTimer: public ITimerScheduledListener {
private:
    std::vector<long> timestamps_us_;
    utils::Mutex mtimestamps_;

public:
    TimingScheduledTimer(std::string label, int period_us) {
        this->init(label, period_us);
    }

    virtual ~TimingScheduledTimer() {}

    void onTimer(utils::Chronometer chrono) override {
        long now = monotonic_us();
        mtimestamps_.lock();
        timestamps_us_.push_back(now);
        mtimestamps_.unlock();
    }

    void onTimerEnd(utils::Chronometer chrono) override {}

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

// --- Utilitaires ---

static void printStats(const logs::Logger &log, const char *label,
        const std::vector<long> &intervals, long expectedUs)
{
    size_t nbSamples = intervals.size();

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
}

static void runBench(const char *label, int intervalMs, int durationMs)
{
    static const logs::Logger &log = logs::LoggerFactory::logger("test::ActionTimerSchedulerBench");

    ActionTimerScheduler scheduler;
    TimingScheduledTimer *t = new TimingScheduledTimer(label, intervalMs * 1000);

    scheduler.start(label, 2);
    scheduler.addTimer(t);

    std::this_thread::sleep_for(std::chrono::milliseconds(durationMs));

    scheduler.stop();

    std::vector<long> intervals = t->getIntervals();
    long expectedUs = intervalMs * 1000L;
    printStats(log, label, intervals, expectedUs);

    delete t;
}

// --- Suite ---

void test::ActionTimerSchedulerBench::suite() {
    benchTimer100ms();
    benchTimer10ms();
    benchTimer1ms();
    benchMultipleTimersConcurrent();
}

void test::ActionTimerSchedulerBench::benchTimer100ms() {
    logger().info() << "benchTimer100ms..." << logs::end;
    runBench("sched_100ms", 100, 2000);
    logger().info() << "benchTimer100ms OK" << logs::end;
}

void test::ActionTimerSchedulerBench::benchTimer10ms() {
    logger().info() << "benchTimer10ms..." << logs::end;
    runBench("sched_10ms", 10, 2000);
    logger().info() << "benchTimer10ms OK" << logs::end;
}

void test::ActionTimerSchedulerBench::benchTimer1ms() {
    logger().info() << "benchTimer1ms..." << logs::end;
    runBench("sched_1ms", 1, 2000);
    logger().info() << "benchTimer1ms OK" << logs::end;
}

void test::ActionTimerSchedulerBench::benchMultipleTimersConcurrent() {
    logger().info() << "benchMultipleTimersConcurrent..." << logs::end;

    static const logs::Logger &log = logs::LoggerFactory::logger("test::ActionTimerSchedulerBench");

    ActionTimerScheduler scheduler;
    TimingScheduledTimer *fast = new TimingScheduledTimer("multi_5ms",   5 * 1000);
    TimingScheduledTimer *med  = new TimingScheduledTimer("multi_20ms",  20 * 1000);
    TimingScheduledTimer *slow = new TimingScheduledTimer("multi_100ms", 100 * 1000);

    scheduler.start("sched_multi", 2);
    scheduler.addTimer(fast);
    scheduler.addTimer(med);
    scheduler.addTimer(slow);

    std::this_thread::sleep_for(std::chrono::milliseconds(3000));

    scheduler.stop();

    log.info() << "--- 3 timers concurrents (5ms / 20ms / 100ms) pendant 3s ---" << logs::end;
    printStats(log, "multi_5ms",   fast->getIntervals(),   5 * 1000L);
    printStats(log, "multi_20ms",  med->getIntervals(),   20 * 1000L);
    printStats(log, "multi_100ms", slow->getIntervals(), 100 * 1000L);

    delete fast;
    delete med;
    delete slow;

    logger().info() << "benchMultipleTimersConcurrent OK" << logs::end;
}
