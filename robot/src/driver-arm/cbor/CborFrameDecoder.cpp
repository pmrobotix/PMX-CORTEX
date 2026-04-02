#include "CborFrameDecoder.hpp"
#include "cppCrc.h"
#include <qcbor/qcbor_decode.h>
#include <qcbor/qcbor_spiffy_decode.h>

CborFrameDecoder::CborFrameDecoder()
    : state_(SYNC_LOOKUP), syncAccum_(0), headerIdx_(0),
      expectedCrc_(0), payloadSize_(0), payloadIdx_(0), posReady_(false)
{
    lastPos_ = {};
}

void CborFrameDecoder::pushByte(uint8_t byte)
{
    switch (state_)
    {
    case SYNC_LOOKUP:
        syncAccum_ = (syncAccum_ >> 8) | ((uint32_t)byte << 24);
        if (syncAccum_ == SYNC_WORD)
        {
            state_ = READ_CRC;
            headerIdx_ = 0;
        }
        break;

    case READ_CRC:
        headerBuf_[headerIdx_++] = byte;
        if (headerIdx_ == 4)
        {
            memcpy(&expectedCrc_, headerBuf_, 4);
            state_ = READ_SIZE;
            headerIdx_ = 0;
        }
        break;

    case READ_SIZE:
        headerBuf_[headerIdx_++] = byte;
        if (headerIdx_ == 4)
        {
            memcpy(&payloadSize_, headerBuf_, 4);
            if (payloadSize_ > MAX_PAYLOAD)
            {
                // Trame trop grande, on revient en sync
                state_ = SYNC_LOOKUP;
                syncAccum_ = 0;
            }
            else
            {
                state_ = READ_PAYLOAD;
                payloadIdx_ = 0;
            }
        }
        break;

    case READ_PAYLOAD:
        payloadBuf_[payloadIdx_++] = byte;
        if (payloadIdx_ == payloadSize_)
        {
            // Vérifier le CRC
            uint32_t computedCrc = CRC32::CRC32::calc(payloadBuf_, payloadSize_);
            if (computedCrc == expectedCrc_)
            {
                decodePayload();
            }
            state_ = SYNC_LOOKUP;
            syncAccum_ = 0;
        }
        break;
    }
}

bool CborFrameDecoder::getPosition(PositionData &pos)
{
    if (posReady_)
    {
        pos = lastPos_;
        posReady_ = false;
        return true;
    }
    return false;
}

void CborFrameDecoder::decodePayload()
{
    QCBORDecodeContext ctx;
    UsefulBufC input = {payloadBuf_, payloadSize_};
    QCBORDecode_Init(&ctx, input, QCBOR_DECODE_MODE_NORMAL);

    QCBORDecode_EnterArray(&ctx, nullptr);

    int64_t val;
    double dval;

    QCBORDecode_GetInt64(&ctx, &val);
    lastPos_.x = (int32_t)val;

    QCBORDecode_GetInt64(&ctx, &val);
    lastPos_.y = (int32_t)val;

    QCBORDecode_GetDouble(&ctx, &dval);
    lastPos_.theta = (float)dval;

    QCBORDecode_GetInt64(&ctx, &val);
    lastPos_.cmd_id = (int)val;

    QCBORDecode_GetInt64(&ctx, &val);
    lastPos_.status = (int)val;

    QCBORDecode_GetInt64(&ctx, &val);
    lastPos_.pending_count = (int)val;

    QCBORDecode_GetInt64(&ctx, &val);
    lastPos_.left_speed = (int8_t)val;

    QCBORDecode_GetInt64(&ctx, &val);
    lastPos_.right_speed = (int8_t)val;

    QCBORDecode_ExitArray(&ctx);

    QCBORError err = QCBORDecode_Finish(&ctx);
    if (err == QCBOR_SUCCESS)
    {
        lastPos_.valid = true;
        posReady_ = true;
    }
}
