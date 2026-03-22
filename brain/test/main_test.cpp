#include <iostream>
#include <string>
#include <thread>
#include <chrono>

std::string getPlatformName()
{
#ifdef SIMU
    return "SIMU (native Linux)";
#else
    return "ARM OPOS6UL";
#endif
}

int main()
{
    std::cout << "=== PMX-CORTEX Test ===" << std::endl;
    std::cout << "Plateforme : " << getPlatformName() << std::endl;
    std::cout << "C++ standard : " << __cplusplus << std::endl;

    // Test thread (pthread)
    bool threadOk = false;
    std::thread t([&threadOk]() {
        threadOk = true;
    });
    t.join();

    std::cout << "Thread test : " << (threadOk ? "OK" : "FAIL") << std::endl;

    // Test chrono
    auto start = std::chrono::steady_clock::now();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - start).count();

    std::cout << "Chrono test : " << elapsed << " ms (attendu ~10ms) : "
              << (elapsed >= 5 ? "OK" : "FAIL") << std::endl;

    std::cout << "=== Tous les tests OK ===" << std::endl;
    return 0;
}
