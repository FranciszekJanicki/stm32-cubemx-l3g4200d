#ifndef L3G4200D_HPP
#define L3G4200D_HPP

#include "l3g4200d_config.hpp"
#include "l3g4200d_registers.hpp"
#include <optional>

namespace L3G4200D {

    struct L3G4200D {
    public:
        L3G4200D() noexcept = default;
        L3G4200D(I2CDevice&& i2c_device, Config const& config) noexcept;

        L3G4200D(L3G4200D const& other) = delete;
        L3G4200D(L3G4200D&& other) noexcept = default;

        L3G4200D& operator=(L3G4200D const& other) = delete;
        L3G4200D& operator=(L3G4200D&& other) noexcept = default;

        ~L3G4200D() noexcept;

        std::optional<float> get_temperature_scaled() const noexcept;

        std::optional<float> get_rotation_x_scaled() const noexcept;
        std::optional<float> get_rotation_y_scaled() const noexcept;
        std::optional<float> get_rotation_z_scaled() const noexcept;
        std::optional<Vec3D<float>> get_rotation_scaled() const noexcept;

        bool is_data_ready() const noexcept;

    private:
        void write_byte(std::uint8_t const reg_address, std::uint8_t const byte) const noexcept;

        template <std::size_t SIZE>
        void write_bytes(std::uint8_t const reg_address, std::array<std::uint8_t, SIZE> const& bytes) const noexcept;

        std::uint8_t read_byte(std::uint8_t const reg_address) const noexcept;

        template <std::size_t SIZE>
        std::array<std::uint8_t, SIZE> read_bytes(std::uint8_t const reg_address) const noexcept;

        std::optional<std::int8_t> get_temperature_raw() const noexcept;

        std::optional<std::int16_t> get_rotation_x_raw() const noexcept;
        std::optional<std::int16_t> get_rotation_y_raw() const noexcept;
        std::optional<std::int16_t> get_rotation_z_raw() const noexcept;
        std::optional<Vec3D<std::int16_t>> get_rotation_raw() const noexcept;

        void initialize(Config const& config) noexcept;
        void deinitialize() noexcept;

        bool is_valid_device_id() const noexcept;
        std::uint8_t get_device_id() const noexcept;

        WHO_AM_I get_who_am_i_register() const noexcept;

        CTRL_REG1 get_ctrl_reg1_register() const noexcept;
        void set_ctrl_reg1_register(CTRL_REG1 const& ctrl_reg1) const noexcept;

        CTRL_REG2 get_ctrl_reg2_register() const noexcept;
        void set_ctrl_reg2_register(CTRL_REG2 const ctrl_reg2) const noexcept;

        CTRL_REG3 get_ctrl_reg3_register() const noexcept;
        void set_ctrl_reg3_register(CTRL_REG3 const ctrl_reg3) const noexcept;

        CTRL_REG4 get_ctrl_reg4_register() const noexcept;
        void set_ctrl_reg4_register(CTRL_REG4 const ctrl_reg4) const noexcept;

        CTRL_REG5 get_ctrl_reg5_register() const noexcept;
        void set_ctrl_reg5_register(CTRL_REG5 const ctrl_reg5) const noexcept;

        REFERENCE get_reference_register() const noexcept;

        OUT_TEMP get_out_temp_register() const noexcept;

        STATUS_REG get_status_reg_register() const noexcept;

        OUT_X get_out_x_registers() const noexcept;

        OUT_Y get_out_y_registers() const noexcept;

        OUT_Z get_out_z_registers() const noexcept;

        OUT get_out_registers() const noexcept;

        FIFO_CTRL_REG get_fifo_ctrl_reg_register() const noexcept;
        void set_fifo_ctrl_reg_register(FIFO_CTRL_REG const fifo_ctrl_reg) const noexcept;

        FIFO_SRC_REG get_fifo_src_reg_register() const noexcept;
        void set_fifo_src_reg_register(FIFO_SRC_REG const fifo_src_reg) const noexcept;

        INT1_CFG get_int1_cfg_register() const noexcept;
        void set_int1_cfg_register(INT1_CFG const int1_cfg) const noexcept;

        INT1_SRC get_int1_src_register() const noexcept;
        void set_int1_src_register(INT1_SRC const in1_src) const noexcept;

        INT1_THS_X get_int1_ths_x_register() const noexcept;
        void set_int1_ths_x_register(INT1_THS_X const int1_ths_x) const noexcept;

        INT1_THS_Y get_int1_ths_y_register() const noexcept;
        void set_int1_ths_y_register(INT1_THS_Y const int1_ths_y) const noexcept;

        INT1_THS_Z get_int1_ths_z_register() const noexcept;
        void set_int1_ths_z_register(INT1_THS_Z const int1_ths_z) const noexcept;

        INT1_DURATION get_int1_duration_register() const noexcept;
        void set_int1_duration(INT1_DURATION const in1_duration) const noexcept;

        bool initialized_{false};

        float scale_{};

        I2CDevice i2c_device_{};
    };

    template <std::size_t SIZE>
    inline void L3G4200D::write_bytes(std::uint8_t const reg_address,
                                      std::array<std::uint8_t, SIZE> const& bytes) const noexcept
    {
        this->i2c_device_.write_bytes(reg_address, bytes);
    }

    template <std::size_t SIZE>
    inline std::array<std::uint8_t, SIZE> L3G4200D::read_bytes(std::uint8_t const reg_address) const noexcept
    {
        return this->i2c_device_.read_bytes<SIZE>(reg_address);
    }

}; // namespace L3G4200D

#endif // L3G4200D_HPP