#ifndef CH_SERIAL_HPP
#define CH_SERIAL_HPP

#include <cstdint>
#include <cstring>

// Debug logs - uncomment to enable
// #define CH_DEBUG

constexpr int MAXRAWLEN = 512;     // Max raw frame length
constexpr int MAX_NODE_SIZE = 16;  // Max support node count

// Data items
constexpr uint8_t kItemID = 0x90;
constexpr uint8_t kItemAccRaw = 0xA0;
constexpr uint8_t kItemGyrRaw = 0xB0;
constexpr uint8_t kItemMagRaw = 0xC0;
constexpr uint8_t kItemRotationEul = 0xD0;
constexpr uint8_t kItemRotationQuat = 0xD1;
constexpr uint8_t kItemPressure = 0xF0;
constexpr uint8_t KItemIMUSOL = 0x91;
constexpr uint8_t KItemGWSOL = 0x62;

struct ch_imu_data_t {
    uint32_t id;     // User defined ID
    float acc[3];    // Acceleration
    float gyr[3];    // Angular velocity
    float mag[3];    // Magnetic field
    float eul[3];    // Attitude: Euler angle
    float quat[4];   // Attitude: quaternion
    float pressure;  // Air pressure
    uint32_t timestamp;
};

struct raw_t {
    int nbyte;               // Number of bytes in message buffer
    int len;                 // Message length (bytes)
    uint8_t buf[MAXRAWLEN];  // Message raw buffer
    uint8_t gwid;            // Network ID (HI222)
    uint8_t nimu;            // Number of IMU (HI222)

    ch_imu_data_t imu[MAX_NODE_SIZE];  // IMU data list (for HI226/HI229/CH100/CH110, use imu[0])
    uint8_t item_code[8];              // Item code received in one frame
    uint8_t nitem_code;                // Number of item codes
};

// Function declarations
extern "C" {

/**
 * @brief Input CHAOHE raw message from stream
 *
 * Fetch next raw data and input a message from stream
 *
 * @param raw    Receiver raw data control struct (input/output)
 * @param data   Stream data (1 byte)
 * @return int   Status (-1: error message, 0: no message, 1: input data successfully)
 */
int ch_serial_input(raw_t* raw, uint8_t data);

/**
 * @brief Debug function to dump decode information
 *
 * Dump decode information, need to enable CH_DEBUG
 *
 * @param raw    Receiver raw data control struct (input/output)
 */
void ch_dump_imu_data(raw_t* raw);
}

#endif  // CH_SERIAL_HPP
