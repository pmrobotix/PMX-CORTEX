/*!
 * \file
 * \brief Wrapper I2C avec repeated start pour lectures atomiques.
 *
 * Remplace as_devices/AsI2c pour la communication avec le slave Teensy (0x2D)
 * et le MCP23017 (0x20).
 * Utilise un seul file descriptor et un seul ioctl(I2C_RDWR) avec 2 messages
 * pour les lectures registre (repeated start), evitant la desynchronisation
 * du toggle got_reg_num de I2CRegisterSlave cote Teensy.
 *
 * Bus recovery : apres N erreurs consecutives, ferme et rouvre le fd.
 * Le driver kernel i2c-imx reset le bus (9 pulses SCL + STOP) ce qui
 * resynchronise le slave.
 */

#ifndef AS_I2C_ATOMIC_HPP_
#define AS_I2C_ATOMIC_HPP_

#include <cstdint>
#include <cstddef>
#include <cerrno>
#include <ctime>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#include "log/LoggerFactory.hpp"

class AsI2cAtomic
{
public:
    AsI2cAtomic(int bus, uint8_t slaveAddr)
        : bus_(bus), slaveAddr_(slaveAddr)
    {
    }

    ~AsI2cAtomic()
    {
        if (fd_ >= 0) close(fd_);
    }

    /*!
     * \brief Ouvre /dev/i2c-N. A appeler une fois au demarrage.
     * \return true si OK.
     */
    bool open()
    {
        // Unbind/rebind une seule fois par bus au premier open(),
        // pour garantir un bus propre apres un crash precedent.
        resetBusOnce(bus_);

        char path[20];
        snprintf(path, sizeof(path), "/dev/i2c-%d", bus_);
        fd_ = ::open(path, O_RDWR);
        if (fd_ < 0) {
            logger().error() << "AsI2cAtomic: open " << path << " FAILED" << logs::end;
            return false;
        }
        consecutiveReadErrors_ = 0;
        logger().debug() << "AsI2cAtomic: opened " << path << " fd=" << fd_ << logs::end;
        return true;
    }

    bool isOpen() const { return fd_ >= 0; }

    /*!
     * \brief Force un bus recovery (unbind+rebind imx-i2c).
     * Utilisable quand begin() echoue pour tenter un reset du bus.
     */
    void forceRecover() { recover(); }

    /*!
     * \brief Lecture registre atomique (repeated start, 1 seul ioctl, 2 messages).
     * \return 0 si OK, -1 si erreur.
     */
    int readReg(uint8_t reg, uint8_t* data, size_t len)
    {
        if (fd_ < 0) return -1;

        struct i2c_msg msgs[2] = {
            { slaveAddr_, 0,        1,                        &reg },
            { slaveAddr_, I2C_M_RD, static_cast<__u16>(len),  data }
        };
        struct i2c_rdwr_ioctl_data rdwr = { msgs, 2 };

        if (ioctl(fd_, I2C_RDWR, &rdwr) < 0) {
            // Retry 1× apres 1ms (NACK transitoire du slave)
            usleep(1000);
            if (ioctl(fd_, I2C_RDWR, &rdwr) < 0) {
                // 2e echec = vraie erreur
                logger().warn() << "FAIL reg=0x" << std::hex << (int)reg << std::dec
                        << " len=" << len << " errno=" << errno
                        << " (" << strerror(errno) << ")" << logs::end;
                consecutiveReadErrors_++;
                if (consecutiveReadErrors_ >= MAX_CONSECUTIVE_ERRORS && bus_ == 0) {
                    // Cooldown : pas de recovery si le dernier a eu lieu il y a moins de 10s
                    struct timespec now;
                    clock_gettime(CLOCK_MONOTONIC, &now);
                    long elapsed_ms = (now.tv_sec - lastRecovery_.tv_sec) * 1000
                                    + (now.tv_nsec - lastRecovery_.tv_nsec) / 1000000;
                    if (elapsed_ms > RECOVERY_COOLDOWN_MS) {
                        recover();
                        clock_gettime(CLOCK_MONOTONIC, &lastRecovery_);
                    }
                    consecutiveReadErrors_ = 0;
                }
                return -1;
            }
            logger().warn() << "retry OK reg=0x" << std::hex << (int)reg << std::dec
                    << " len=" << len << logs::end;
        }
        consecutiveReadErrors_ = 0;
        return 0;
    }

    /*!
     * \brief Lecture d'un octet depuis un registre.
     * \return valeur lue, ou 0xFF si erreur.
     */
    uint8_t readRegByte(uint8_t reg)
    {
        uint8_t val = 0;
        if (readReg(reg, &val, 1) < 0) return 0xFF;
        return val;
    }

    /*!
     * \brief Ecriture registre (1 ioctl, 1 message).
     * \return 0 si OK, -1 si erreur.
     */
    int writeReg(uint8_t reg, const uint8_t* data, size_t len)
    {
        if (fd_ < 0) return -1;

        // Buffer : [reg, data0, data1, ...]
        uint8_t buf[len + 1];
        buf[0] = reg;
        for (size_t i = 0; i < len; i++) buf[i + 1] = data[i];

        struct i2c_msg msg = { slaveAddr_, 0, static_cast<__u16>(len + 1), buf };
        struct i2c_rdwr_ioctl_data rdwr = { &msg, 1 };

        if (ioctl(fd_, I2C_RDWR, &rdwr) < 0) {
            return -1;
        }
        return 0;
    }

private:
    static inline const logs::Logger& logger()
    {
        static const logs::Logger& instance = logs::LoggerFactory::logger("AsI2cAtomic");
        return instance;
    }


    /*!
     * \brief Bus recovery : ferme et rouvre /dev/i2c-N.
     * Le driver kernel i2c-imx envoie 9 pulses SCL + STOP pour
     * resynchroniser le bus et le slave.
     */
    /*!
     * \brief Bus recovery : unbind/rebind du driver kernel imx-i2c.
     * Force un reset hardware du peripherique I2C (9 pulses SCL + STOP)
     * pour desynchroniser un slave bloque.
     * Device names i.MX6ULL : bus 0 = "21a0000.i2c", bus 1 = "21a4000.i2c".
     */
    void recover()
    {
        logger().warn() << "BUS RECOVERY bus " << bus_
                << " (" << consecutiveReadErrors_ << " consecutive read errors)" << logs::end;

        if (fd_ >= 0) {
            close(fd_);
            fd_ = -1;
        }

        const char* devName = (bus_ == 0) ? "21a0000.i2c" : "21a4000.i2c";

        FILE* f = fopen("/sys/bus/platform/drivers/imx-i2c/unbind", "w");
        if (f) { fprintf(f, "%s", devName); fclose(f); }
        else { logger().error() << "recover unbind FOPEN FAIL errno=" << errno << logs::end; }
        usleep(200000);

        f = fopen("/sys/bus/platform/drivers/imx-i2c/bind", "w");
        if (f) { fprintf(f, "%s", devName); fclose(f); }
        else { logger().error() << "recover rebind FOPEN FAIL errno=" << errno << logs::end; }
        usleep(200000);

        char path[20];
        snprintf(path, sizeof(path), "/dev/i2c-%d", bus_);
        fd_ = ::open(path, O_RDWR);
        if (fd_ >= 0) {
            logger().info() << "bus " << bus_ << " recovered OK fd=" << fd_ << logs::end;
        } else {
            logger().error() << "recover reopen " << path << " FAILED" << logs::end;
        }

        consecutiveReadErrors_ = 0;
    }

    /*!
     * \brief Reset un bus I2C une seule fois (unbind+rebind).
     * Utilise un flag statique par bus pour eviter de reset
     * plusieurs fois quand plusieurs instances partagent le meme bus.
     */
    static void resetBusOnce(int bus)
    {
        static bool busReset[2] = { false, false };
        if (bus < 0 || bus > 1) return;
        if (busReset[bus]) return;  // deja fait pour ce bus
        busReset[bus] = true;

        const char* devName = (bus == 0) ? "21a0000.i2c" : "21a4000.i2c";
        // Log en stderr car le logger n'est pas encore init au boot
        // Niveau debug : ne pas afficher en production

        FILE* f = fopen("/sys/bus/platform/drivers/imx-i2c/unbind", "w");
        if (f) { fprintf(f, "%s", devName); fclose(f); }
        usleep(200000);
        f = fopen("/sys/bus/platform/drivers/imx-i2c/bind", "w");
        if (f) { fprintf(f, "%s", devName); fclose(f); }
        usleep(200000);
    }

    static constexpr int MAX_CONSECUTIVE_ERRORS = 3;
    static constexpr long RECOVERY_COOLDOWN_MS = 10000;  // 10s entre deux recovery

    int fd_ = -1;
    int bus_;
    uint8_t slaveAddr_;
    int consecutiveReadErrors_ = 0;
    struct timespec lastRecovery_ = { 0, 0 };
};

#endif // AS_I2C_ATOMIC_HPP_
