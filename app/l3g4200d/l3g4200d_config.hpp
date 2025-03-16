#ifndef L3G4200D_CONFIG_HPP
#define L3G4200D_CONFIG_HPP

#include "../utility/i2c_device.hpp"
#include "../utility/vector3d.hpp"
#include "l3g4200d_registers.hpp"
#include <cstdint>

namespace L3G4200D {

    template <typename T>
    using Vec3D = Utility::Vector3D<T>;

    using I2CDevice = Utility::I2CDevice;

    enum struct RA : std::uint8_t {
        WHO_AM_I = 0x0F,
        CTRL_REG1 = 0x20,
        CTRL_REG2 = 0x21,
        CTRL_REG3 = 0x22,
        CTRL_REG4 = 0x23,
        CTRL_REG5 = 0x24,
        REFERENCE = 0x25,
        OUT_TEMP = 0x26,
        STATUS_REG = 0x27,
        OUT_X_L = 0x28,
        OUT_X_H = 0x29,
        OUT_Y_L = 0x2A,
        OUT_Y_H = 0x2B,
        OUT_Z_L = 0x2C,
        OUT_Z_H = 0x2D,
        FIFO_CTRL_REG = 0x2E,
        FIFO_SRC_REG = 0x2F,
        INT1_CFG = 0x30,
        INT1_SRC = 0x31,
        INT1_THS_XH = 0x32,
        INT1_THS_XL = 0x33,
        INT1_THS_YH = 0x34,
        INT1_THS_YL = 0x35,
        INT1_THS_ZH = 0x36,
        INT1_THS_ZL = 0x37,
        INT1_DURATION = 0x38,
    };

    enum struct DevAddress : std::uint8_t {
        SD0_LOW = 0b1101000,
        SD0_HIGH = 0b1101001,
    };

    enum struct Mode : std::uint8_t {
        BYPASS,
        FIFO,
        STREAM,
        BYPASS_TO_STREAM,
        STREAM_TO_FIFO,
    };

    enum struct DataRate : std::uint8_t {
        ODR_100HZ = 0b00,
        ODR_200HZ = 0b01,
        ODR_400HZ = 0b10,
        ODR_800HZ = 0b11,
    };

    enum struct BandWidth : std::uint8_t {
        BW_12HZ5 = 0b00,
        BW_25HZ = 0b01,
        BW_50HZ = 0b10,
        BW_70HZ = 0b11,
        BW_20HZ = 0b00,
        BW_110HZ = 0b11,
        BW_30HZ = 0b00,
        BW_35HZ = 0b01,
    };

    enum struct HPFMode : std::uint8_t {
        MODE_NORMAL_RR = 0b00,
        MODE_REFERENCE = 0b01,
        MODE_NORMAL = 0b10,
        MODE_AUTORSEET = 0b11,
    };

    enum struct HPF : std::uint8_t {

    };

    enum struct Range : std::uint8_t {
        FS_250DPS = 0b00,
        FS_500DPS = 0b01,
        FS_2000DPS = 0b11,
    };

    enum struct SelfTest : std::uint8_t {
        NORMAL_MODE = 0b00,
        SELT_TEST_0 = 0b01,
        DISABLED = 0b10,
        SELF_TEST_1 = 0b11,
    };

    struct Config {
        CTRL_REG1 ctrl_reg1{};
        CTRL_REG2 ctrl_reg2{};
        CTRL_REG3 ctrl_reg3{};
        CTRL_REG4 ctrl_reg4{};
        CTRL_REG5 ctrl_reg5{};
        FIFO_CTRL_REG fifo_ctrl_reg{};
        FIFO_SRC_REG fifo_src_reg{};
        INT1_CFG int1_cfg{};
        INT1_SRC int1_src{};
        INT1_THS_X int1_ths_x{};
        INT1_THS_Y int1_ths_y{};
        INT1_THS_Z int1_ths_z{};
        INT1_DURATION int1_duration{};
    };

    auto constexpr DEVICE_ID = 0b11010011;

    inline float range_to_scale(Range const range) noexcept
    {
        switch (range) {
            case Range::FS_250DPS:
                return 0.00762F;
            case Range::FS_500DPS:
                return 0.01524F;
            case Range::FS_2000DPS:
                return 0.06096F;
            default:
                return 0.0F;
        }
    }

    inline float config_to_scale(Config const& config) noexcept
    {
        return range_to_scale(static_cast<Range>(config.ctrl_reg4.fs));
    }

    inline float temp_raw_to_scaled(std::int8_t const raw) noexcept
    {
        return Utility::rescale(raw, static_cast<std::int8_t>(-127U), static_cast<std::int8_t>(127U), -40.0F, 85.0F);
    }

}; // namespace L3G4200D

#endif // L3G4200D_CONFIG_HPP