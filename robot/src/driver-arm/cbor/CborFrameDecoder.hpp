#ifndef CBOR_FRAME_DECODER_HPP_
#define CBOR_FRAME_DECODER_HPP_

#include <cstdint>
#include <cstring>

// Décodeur de trames CBOR côté Brain (OPOS6UL).
// Gère le framing : sync word 0xDEADBEEF + CRC32 + taille + payload CBOR.
// Le payload est un array CBOR de position envoyé par la Nucleo (SerialCbor).
class CborFrameDecoder
{
public:
    struct PositionData
    {
        int32_t x;
        int32_t y;
        float theta;
        int cmd_id;
        int status;
        int pending_count;
        int8_t left_speed;
        int8_t right_speed;
        bool valid;
    };

    CborFrameDecoder();

    // Pousser un octet reçu du port série
    void pushByte(uint8_t byte);

    // Récupérer la dernière position décodée (retourne false si rien de nouveau)
    bool getPosition(PositionData &pos);

private:
    static constexpr uint32_t SYNC_WORD = 0xDEADBEEF;
    static constexpr size_t MAX_PAYLOAD = 128;

    enum State { SYNC_LOOKUP, READ_CRC, READ_SIZE, READ_PAYLOAD };

    State state_;
    uint32_t syncAccum_;
    uint8_t headerBuf_[8];
    size_t headerIdx_;
    uint32_t expectedCrc_;
    uint32_t payloadSize_;
    uint8_t payloadBuf_[MAX_PAYLOAD];
    size_t payloadIdx_;

    PositionData lastPos_;
    bool posReady_;

    void decodePayload();
};

#endif
