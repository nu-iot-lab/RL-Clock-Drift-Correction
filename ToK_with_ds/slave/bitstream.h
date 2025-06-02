#ifndef BITSTREAM_H
#define BITSTREAM_H

#include <cstdint>
#include <cstddef>

class BitWriter {
private:
    uint8_t* buffer;
    size_t bufferSize;
    size_t byteIndex;
    uint8_t currentByte;
    int bitPosition; // [0..7]

public:
    BitWriter(uint8_t* buf, size_t size)
        : buffer(buf), bufferSize(size), byteIndex(0), currentByte(0), bitPosition(0) {}

    bool writeBit(bool bit) {
        currentByte |= bit << (7 - bitPosition);
        bitPosition++;
        if (bitPosition == 8) {
            if (byteIndex >= bufferSize) {
                // buffer overflow
                return false;
            }
            buffer[byteIndex++] = currentByte;
            currentByte = 0;
            bitPosition = 0;
        }
        return true;
    }

    bool writeBits(uint32_t bits, int count) {
        for (int i = count - 1; i >= 0; --i) {
            if (!writeBit((bits >> i) & 1)) {
                return false;
            }
        }
        return true;
    }

    bool flush() {
        if (bitPosition > 0) {
            if (byteIndex >= bufferSize) {
                // buffer overflow
                return false;
            }
            buffer[byteIndex++] = currentByte;
            currentByte = 0;
            bitPosition = 0;
        }
        return true;
    }

    size_t getSize() const {
        return byteIndex;
    }
};

class BitReader {
private:
    const uint8_t* buffer;
    size_t bufferSize;
    size_t byteIndex;
    uint8_t currentByte;
    int bitPosition; // [0..7]

public:
    BitReader(const uint8_t* buf, size_t size)
        : buffer(buf), bufferSize(size), byteIndex(0), bitPosition(8) {}

    bool readBit(bool& bit) {
        if (bitPosition == 8) {
            if (byteIndex >= bufferSize) {
                return false; // no more data
            }
            currentByte = buffer[byteIndex++];
            bitPosition = 0;
        }
        bit = (currentByte >> (7 - bitPosition)) & 1;
        bitPosition++;
        return true;
    }

    bool readBits(uint32_t& bits, int count) {
        bits = 0;
        for (int i = 0; i < count; ++i) {
            bool bit;
            if (!readBit(bit)) {
                return false;
            }
            bits = (bits << 1) | bit;
        }
        return true;
    }
};

#endif // BITSTREAM_H