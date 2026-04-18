//drivers...OPO

#include "BeaconSensors.hpp"

#include <stdlib.h>
#include <unistd.h>
#include <cmath>

#include "log/Logger.hpp"

BeaconSensors::BeaconSensors(int bus, unsigned char i2c_addr) :
        i2c_(bus, i2c_addr), connected_BeaconSensors_(false)
{
    logger().debug() << "BeaconSensors(0x" << reinterpret_cast<void*>(i2c_addr) << ") init" << logs::end;
}

bool BeaconSensors::begin(Settings settings)
{
    if (!i2c_.open()) {
        logger().error() << "Hardware status: BeaconSensors open bus FAILED" << logs::end;
        connected_BeaconSensors_ = false;
        return false;
    }

    // Retry : la Teensy peut mettre plusieurs secondes a demarrer son slave I2C
    // (init de 18 capteurs VL53L1X + ecran ILI9341 + LVGL AVANT registerSlave.listen).
    // L'ordre de boot OPOS6UL / Teensy est non-garanti, donc on retente.
    static constexpr int MAX_RETRIES = 10;
    static constexpr int RETRY_DELAY_US = 500000;  // 500 ms entre chaque tentative

    for (int attempt = 1; attempt <= MAX_RETRIES; attempt++) {
        uint8_t dummy = 0x00;
        int err = i2c_.writeReg(0x02, &dummy, 1);
        if (err >= 0) {
            // Carte presente, on peut lire les registres
            Registers regs = getDataFull();
            if ((int) regs.flags != 0xFF) {
                connected_BeaconSensors_ = true;
                logger().info() << "Hardware status: BeaconSensors OK (attempt "
                        << attempt << "/" << MAX_RETRIES << ")" << logs::end;
                return true;
            }
        }
        if (attempt < MAX_RETRIES) {
            logger().info() << "BeaconSensors: not ready, retry "
                    << attempt << "/" << MAX_RETRIES
                    << " (waiting " << (RETRY_DELAY_US / 1000) << "ms)" << logs::end;
            usleep(RETRY_DELAY_US);
        }
    }

    // Toutes les tentatives ont echoue. Tenter un bus recovery
    // (unbind+rebind imx-i2c) au cas ou le bus serait stuck d'un crash precedent.
    logger().warn() << "BeaconSensors: all " << MAX_RETRIES
            << " retries failed, attempting bus recovery..." << logs::end;
    i2c_.forceRecover();

    // Dernier essai apres recovery
    {
        uint8_t dummy = 0x00;
        int err = i2c_.writeReg(0x02, &dummy, 1);
        if (err >= 0) {
            Registers regs = getDataFull();
            if ((int) regs.flags != 0xFF) {
                connected_BeaconSensors_ = true;
                logger().info() << "Hardware status: BeaconSensors OK after bus recovery" << logs::end;
                return true;
            }
        }
    }

    connected_BeaconSensors_ = false;
    logger().error() << "Hardware status: BeaconSensors is NOT connected (even after recovery) !" << logs::end;
    return false;
}

void BeaconSensors::display(int number)
{
    if (!connected_BeaconSensors_) return;
    uint8_t n = (uint8_t) number;
    if (i2c_.writeReg(0x02, &n, 1) < 0) {
        logger().error() << "display() WRITE FAIL" << logs::end;
    }
}

void BeaconSensors::writeLedLuminosity(uint8_t lum)
{
    if (!connected_BeaconSensors_) return;
    if (i2c_.writeReg(0x01, &lum, 1) < 0) {
        logger().error() << "writeLedLuminosity WRITE FAIL" << logs::end;
    }
}

// Ecritures par champ pour les Settings (bloc 1 + bloc 2).
// Retournent true si OK. Voir ARCHITECTURE_BEACON.md pour le mapping des reg.

bool BeaconSensors::writeNumOfBots(int8_t n)
{
    if (!connected_BeaconSensors_) return false;
    uint8_t v = static_cast<uint8_t>(n);
    return i2c_.writeReg(0x00, &v, 1) >= 0;
}

bool BeaconSensors::writeMatchPoints(uint8_t p)
{
    if (!connected_BeaconSensors_) return false;
    return i2c_.writeReg(0x02, &p, 1) >= 0;
}

bool BeaconSensors::writeMatchState(uint8_t s)
{
    if (!connected_BeaconSensors_) return false;
    return i2c_.writeReg(0x03, &s, 1) >= 0;
}

bool BeaconSensors::writeLcdBacklight(uint8_t b)
{
    if (!connected_BeaconSensors_) return false;
    return i2c_.writeReg(0x04, &b, 1) >= 0;
}

bool BeaconSensors::writeMatchColor(uint8_t c)
{
    if (!connected_BeaconSensors_) return false;
    return i2c_.writeReg(0x05, &c, 1) >= 0;
}

bool BeaconSensors::writeStrategy(uint8_t s)
{
    if (!connected_BeaconSensors_) return false;
    return i2c_.writeReg(0x06, &s, 1) >= 0;
}

bool BeaconSensors::writeAdvDiameter(uint8_t d)
{
    if (!connected_BeaconSensors_) return false;
    return i2c_.writeReg(0x08, &d, 1) >= 0;
}

bool BeaconSensors::writeActionReq(uint8_t v)
{
    if (!connected_BeaconSensors_) return false;
    return i2c_.writeReg(0x09, &v, 1) >= 0;
}

bool BeaconSensors::readSettings(Settings &out)
{
    unsigned char buf[11] = { 0 };
    int err = i2c_.readReg(0x00, buf, SETTINGS_SIZE_BeaconSensors);
    if (err < 0) {
        logger().error() << "readSettings FAIL err=" << err << logs::end;
        return false;
    }
    out.numOfBots     = static_cast<int8_t>(buf[0]);
    out.ledLuminosity = static_cast<int8_t>(buf[1]);
    out.matchPoints   = buf[2];
    out.matchState    = buf[3];
    out.lcdBacklight  = buf[4];
    out.matchColor    = buf[5];
    out.strategy      = buf[6];
    out.testMode      = buf[7];
    out.advDiameter   = buf[8];
    out.actionReq     = buf[9];
    out.seq_touch     = buf[10];
    return true;
}

uint8_t BeaconSensors::readFlag()
{
    return i2c_.readRegByte(DATA_BeaconSensors);
}

Registers BeaconSensors::getData()
{
    unsigned char buf[18] = { 0 };
    Registers regs;

    int err = i2c_.readReg(DATA_BeaconSensors, buf, 18);
    if (err < 0) {
        logger().error() << "getData FAIL step1 offset=" << DATA_BeaconSensors << " len=18 err=" << err << logs::end;
        regs.flags = 0xFF;
        return regs;
    }

    regs.flags = buf[0];
    regs.nbDetectedBots = buf[1];
    regs.c1_mm = buf[2] | (buf[3] << 8);
    regs.c2_mm = buf[4] | (buf[5] << 8);
    regs.c3_mm = buf[6] | (buf[7] << 8);
    regs.c4_mm = buf[8] | (buf[9] << 8);
    regs.c5_mm = buf[10] | (buf[11] << 8);
    regs.c6_mm = buf[12] | (buf[13] << 8);
    regs.c7_mm = buf[14] | (buf[15] << 8);
    regs.c8_mm = buf[16] | (buf[17] << 8);

    logger().debug() << "data = FLAGS:" << (int) regs.flags << " NB:" << (int) regs.nbDetectedBots << " C1:"
            << regs.c1_mm << " " << regs.c2_mm << " " << regs.c3_mm << " " << regs.c4_mm << " " << regs.c5_mm << " "
            << regs.c6_mm << " " << regs.c7_mm << " C8:" << regs.c8_mm << logs::end;

    err = i2c_.readReg(DATA_BeaconSensors + 20, buf, 8); // x1/y1/a1
    if (err < 0) {
        logger().error() << "getData FAIL step2 offset=" << (DATA_BeaconSensors + 20) << " len=8 err=" << err << logs::end;
        regs.flags = 0xFF;
        return regs;
    }

    regs.x1_mm = buf[0] | (buf[1] << 8);
    regs.y1_mm = buf[2] | (buf[3] << 8);
    float2bytes_t a1;
    a1.bytes[0] = buf[4]; a1.bytes[1] = buf[5]; a1.bytes[2] = buf[6]; a1.bytes[3] = buf[7];
    regs.a1_deg = a1.f;

    logger().debug() << " x1:" << regs.x1_mm << " " << regs.y1_mm << " " << regs.a1_deg << logs::end;

    err = i2c_.readReg(DATA_BeaconSensors + 28, buf, 8); // x2/y2/a2
    if (err < 0) {
        logger().error() << "getData FAIL step3 offset=" << (DATA_BeaconSensors + 28) << " len=8 err=" << err << logs::end;
        regs.flags = 0xFF;
        return regs;
    }
    regs.x2_mm = buf[0] | (buf[1] << 8);
    regs.y2_mm = buf[2] | (buf[3] << 8);
    float2bytes_t a2;
    a2.bytes[0] = buf[4]; a2.bytes[1] = buf[5]; a2.bytes[2] = buf[6]; a2.bytes[3] = buf[7];
    regs.a2_deg = a2.f;

    logger().debug() << " x2:" << regs.x2_mm << " " << regs.y2_mm << " " << regs.a2_deg << logs::end;

    err = i2c_.readReg(DATA_BeaconSensors + 36, buf, 8); // x3/y3/a3
    if (err < 0) {
        logger().error() << "getData FAIL step4 offset=" << (DATA_BeaconSensors + 36) << " len=8 err=" << err << logs::end;
        regs.flags = 0xFF;
        return regs;
    }
    regs.x3_mm = buf[0] | (buf[1] << 8);
    regs.y3_mm = buf[2] | (buf[3] << 8);
    float2bytes_t a3;
    a3.bytes[0] = buf[4]; a3.bytes[1] = buf[5]; a3.bytes[2] = buf[6]; a3.bytes[3] = buf[7];
    regs.a3_deg = a3.f;

    logger().debug() << " x3:" << regs.x3_mm << " " << regs.y3_mm << " " << regs.a3_deg << logs::end;

    err = i2c_.readReg(DATA_BeaconSensors + 44, buf, 8); // x4/y4/a4
    if (err < 0) {
        logger().error() << "getData FAIL step5 offset=" << (DATA_BeaconSensors + 44) << " len=8 err=" << err << logs::end;
        regs.flags = 0xFF;
        return regs;
    }
    regs.x4_mm = buf[0] | (buf[1] << 8);
    regs.y4_mm = buf[2] | (buf[3] << 8);
    float2bytes_t a4;
    a4.bytes[0] = buf[4]; a4.bytes[1] = buf[5]; a4.bytes[2] = buf[6]; a4.bytes[3] = buf[7];
    regs.a4_deg = a4.f;

    logger().debug() << " x4:" << regs.x4_mm << " " << regs.y4_mm << " " << regs.a4_deg << logs::end;

    err = i2c_.readReg(DATA_BeaconSensors + 52, buf, 8); // d1-d4
    if (err < 0) {
        logger().error() << "getData FAIL step6 offset=" << (DATA_BeaconSensors + 52) << " len=8 err=" << err << logs::end;
        regs.flags = 0xFF;
        return regs;
    }
    regs.d1_mm = buf[0] | (buf[1] << 8);
    regs.d2_mm = buf[2] | (buf[3] << 8);
    regs.d3_mm = buf[4] | (buf[5] << 8);
    regs.d4_mm = buf[6] | (buf[7] << 8);

    logger().debug() << " d1:" << regs.d1_mm << " " << regs.d2_mm << " " << regs.d3_mm << " " << regs.d4_mm
            << logs::end;

    // Lecture des registres timing (t1-t4_us + seq) à l'offset DATA + 124
    err = i2c_.readReg(DATA_BeaconSensors + 124, buf, 12);
    if (err >= 0) {
        regs.t1_us = buf[0] | (buf[1] << 8);
        regs.t2_us = buf[2] | (buf[3] << 8);
        regs.t3_us = buf[4] | (buf[5] << 8);
        regs.t4_us = buf[6] | (buf[7] << 8);
        regs.seq = buf[8] | (buf[9] << 8) | (buf[10] << 16) | (buf[11] << 24);

        logger().debug() << " t1:" << regs.t1_us << "us t2:" << regs.t2_us
                << "us seq:" << regs.seq << logs::end;
    }

    return regs;
}

Registers BeaconSensors::getDataFull()
{
    // Lecture de tous les registres en une seule transaction I2C (136 bytes).
    // Offsets relatifs au debut du read_only_buffer (= DATA_BeaconSensors = 11).
    // Le buffer contient : flags(1) + nb(1) + c1-c8(16) + reserved(2) +
    //   x1y1a1(8) + x2y2a2(8) + x3y3a3(8) + x4y4a4(8) + d1-d4(8) +
    //   zones z1-z4(64) + t1-t4(8) + seq(4) = 136 bytes.
    static constexpr int FULL_SIZE = 136;
    unsigned char buf[FULL_SIZE] = { 0 };
    Registers regs;

    int err = i2c_.readReg(DATA_BeaconSensors, buf, FULL_SIZE);
    if (err < 0) {
        logger().error() << "getDataFull FAIL len=" << FULL_SIZE << " err=" << err << logs::end;
        regs.flags = 0xFF;
        return regs;
    }

    // Offset 0 : flags + nbDetectedBots + collision c1-c8 (18 bytes)
    regs.flags = buf[0];
    regs.nbDetectedBots = buf[1];
    regs.c1_mm = buf[2]  | (buf[3] << 8);
    regs.c2_mm = buf[4]  | (buf[5] << 8);
    regs.c3_mm = buf[6]  | (buf[7] << 8);
    regs.c4_mm = buf[8]  | (buf[9] << 8);
    regs.c5_mm = buf[10] | (buf[11] << 8);
    regs.c6_mm = buf[12] | (buf[13] << 8);
    regs.c7_mm = buf[14] | (buf[15] << 8);
    regs.c8_mm = buf[16] | (buf[17] << 8);

    // Offset 20 : x1/y1/a1 (8 bytes)
    regs.x1_mm = buf[20] | (buf[21] << 8);
    regs.y1_mm = buf[22] | (buf[23] << 8);
    float2bytes_t a1;
    a1.bytes[0] = buf[24]; a1.bytes[1] = buf[25]; a1.bytes[2] = buf[26]; a1.bytes[3] = buf[27];
    regs.a1_deg = a1.f;

    // Offset 28 : x2/y2/a2 (8 bytes)
    regs.x2_mm = buf[28] | (buf[29] << 8);
    regs.y2_mm = buf[30] | (buf[31] << 8);
    float2bytes_t a2;
    a2.bytes[0] = buf[32]; a2.bytes[1] = buf[33]; a2.bytes[2] = buf[34]; a2.bytes[3] = buf[35];
    regs.a2_deg = a2.f;

    // Offset 36 : x3/y3/a3 (8 bytes)
    regs.x3_mm = buf[36] | (buf[37] << 8);
    regs.y3_mm = buf[38] | (buf[39] << 8);
    float2bytes_t a3;
    a3.bytes[0] = buf[40]; a3.bytes[1] = buf[41]; a3.bytes[2] = buf[42]; a3.bytes[3] = buf[43];
    regs.a3_deg = a3.f;

    // Offset 44 : x4/y4/a4 (8 bytes)
    regs.x4_mm = buf[44] | (buf[45] << 8);
    regs.y4_mm = buf[46] | (buf[47] << 8);
    float2bytes_t a4;
    a4.bytes[0] = buf[48]; a4.bytes[1] = buf[49]; a4.bytes[2] = buf[50]; a4.bytes[3] = buf[51];
    regs.a4_deg = a4.f;

    // Offset 52 : d1-d4 (8 bytes)
    regs.d1_mm = buf[52] | (buf[53] << 8);
    regs.d2_mm = buf[54] | (buf[55] << 8);
    regs.d3_mm = buf[56] | (buf[57] << 8);
    regs.d4_mm = buf[58] | (buf[59] << 8);

    // Offset 60-123 : zones z1-z4 (64 bytes) — pas parsees ici

    // Offset 124 : t1-t4 + seq (12 bytes)
    regs.t1_us = buf[124] | (buf[125] << 8);
    regs.t2_us = buf[126] | (buf[127] << 8);
    regs.t3_us = buf[128] | (buf[129] << 8);
    regs.t4_us = buf[130] | (buf[131] << 8);
    regs.seq   = buf[132] | (buf[133] << 8) | (buf[134] << 16) | (buf[135] << 24);

    logger().debug() << "dataFull FLAGS:" << (int)regs.flags << " NB:" << (int)regs.nbDetectedBots
            << " seq:" << regs.seq << " t1:" << regs.t1_us << "us" << logs::end;

    return regs;
}

//________________________________________________

int BeaconSensors::ScanBus()
{
    logger().debug() << "I2C Scanner starting... " << logs::end;
    // ScanBus non supporte avec AsI2cAtomic (pas de scan d'adresses).
    logger().info() << "ScanBus: not implemented with AsI2cAtomic" << logs::end;
    return 0;
}
