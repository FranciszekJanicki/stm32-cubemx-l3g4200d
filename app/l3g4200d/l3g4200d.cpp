#include "l3g4200d.hpp"
#include "utility.hpp"

namespace L3G4200D {

    L3G4200D::L3G4200D(I2CDevice&& i2c_device, Config const& config) noexcept :
        scale_{config_to_scale(config)}, i2c_device_{std::forward<I2CDevice>(i2c_device)}
    {
        this->initialize(config);
    }

    L3G4200D::~L3G4200D() noexcept
    {
        this->deinitialize();
    }

    std::optional<float> L3G4200D::get_temperature_scaled() const noexcept
    {
        return this->get_temperature_raw().transform(&temp_raw_to_scaled);
    }

    std::optional<float> L3G4200D::get_rotation_x_scaled() const noexcept
    {
        return this->get_rotation_x_raw().transform(
            [this](std::int16_t const raw) { return static_cast<float>(raw) * this->scale_; });
    }

    std::optional<float> L3G4200D::get_rotation_y_scaled() const noexcept
    {
        return this->get_rotation_z_raw().transform(
            [this](std::int16_t const raw) { return static_cast<float>(raw) * this->scale_; });
    }

    std::optional<float> L3G4200D::get_rotation_z_scaled() const noexcept
    {
        return this->get_rotation_y_raw().transform(
            [this](std::int16_t const raw) { return static_cast<float>(raw) * this->scale_; });
    }

    std::optional<Vec3D<float>> L3G4200D::get_rotation_scaled() const noexcept
    {
        return this->get_rotation_raw().transform(
            [this](Vec3D<std::int16_t> const& raw) { return static_cast<Vec3D<float>>(raw) * this->scale_; });
    }

    bool L3G4200D::is_data_ready() const noexcept
    {
        return this->get_status_reg_register().zyx_da;
    }

    void L3G4200D::write_byte(std::uint8_t const reg_address, std::uint8_t const byte) const noexcept
    {
        this->i2c_device_.write_byte(reg_address, byte);
    }

    std::uint8_t L3G4200D::read_byte(std::uint8_t const reg_address) const noexcept
    {
        return this->i2c_device_.read_byte(reg_address);
    }

    std::optional<std::int8_t> L3G4200D::get_temperature_raw() const noexcept
    {
        return this->initialized_ ? std::optional<std::int8_t>{
                                        std::bit_cast<std::int8_t>(this->get_out_temp_register()),
                                    } : std::optional<std::int8_t>{std::nullopt};
    }

    std::optional<std::int16_t> L3G4200D::get_rotation_x_raw() const noexcept
    {
        return this->initialized_ ? std::optional<std::int16_t>{
                                        std::byteswap(std::bit_cast<std::int16_t>(this->get_out_x_registers())),
                                    } : std::optional<std::int16_t>{std::nullopt};
    }

    std::optional<std::int16_t> L3G4200D::get_rotation_y_raw() const noexcept
    {
        return this->initialized_ ? std::optional<std::int16_t>{
                                        std::byteswap(std::bit_cast<std::int16_t>(this->get_out_y_registers())),
                                    } : std::optional<std::int16_t>{std::nullopt};
    }

    std::optional<std::int16_t> L3G4200D::get_rotation_z_raw() const noexcept
    {
        return this->initialized_ ? std::optional<std::int16_t>{
                                        std::byteswap(std::bit_cast<std::int16_t>(this->get_out_z_registers())),
                                    } : std::optional<std::int16_t>{std::nullopt};
    }

    std::optional<Vec3D<std::int16_t>> L3G4200D::get_rotation_raw() const noexcept
    {
        auto const out = this->get_out_registers();
        return this->initialized_ ? std::optional<Vec3D<std::int16_t>>{
                                        std::in_place,
                                        std::byteswap(std::bit_cast<std::int16_t>(out.out_x)),
                                        std::byteswap(std::bit_cast<std::int16_t>(out.out_y)),
                                        std::byteswap(std::bit_cast<std::int16_t>(out.out_z)),
                                    } : std::optional<Vec3D<std::int16_t>>{std::nullopt};
    }

    void L3G4200D::initialize(Config const& config) noexcept
    {
        if (this->is_valid_device_id()) {
            this->set_ctrl_reg1_register(config.ctrl_reg1);
            this->set_ctrl_reg2_register(config.ctrl_reg2);
            this->set_ctrl_reg3_register(config.ctrl_reg3);
            this->set_ctrl_reg4_register(config.ctrl_reg4);
            this->set_ctrl_reg5_register(config.ctrl_reg5);
            this->set_fifo_ctrl_reg_register(config.fifo_ctrl_reg);
            this->set_fifo_src_reg_register(config.fifo_src_reg);
            this->set_int1_cfg_register(config.int1_cfg);
            this->set_int1_src_register(config.int1_src);
            this->set_int1_ths_x_register(config.int1_ths_x);
            this->set_int1_ths_y_register(config.int1_ths_y);
            this->set_int1_ths_z_register(config.int1_ths_z);
            this->set_int1_duration(config.int1_duration);
            this->initialized_ = true;
        }
    }

    void L3G4200D::deinitialize() noexcept
    {
        if (this->is_valid_device_id()) {
            this->initialized_ = false;
        }
    }

    bool L3G4200D::is_valid_device_id() const noexcept
    {
        return this->get_device_id() == DEVICE_ID;
    }

    std::uint8_t L3G4200D::get_device_id() const noexcept
    {
        return std::bit_cast<std::uint8_t>(this->get_who_am_i_register());
    }

    WHO_AM_I L3G4200D::get_who_am_i_register() const noexcept
    {
        return std::bit_cast<WHO_AM_I>(this->read_byte(std::to_underlying(RA::WHO_AM_I)));
    }

    CTRL_REG1 L3G4200D::get_ctrl_reg1_register() const noexcept
    {
        return std::bit_cast<CTRL_REG1>(this->read_byte(std::to_underlying(RA::CTRL_REG1)));
    }

    void L3G4200D::set_ctrl_reg1_register(CTRL_REG1 const& ctrl_reg1) const noexcept
    {
        this->write_byte(std::to_underlying(RA::CTRL_REG1), std::bit_cast<std::uint8_t>(ctrl_reg1));
    }

    CTRL_REG2 L3G4200D::get_ctrl_reg2_register() const noexcept
    {
        return std::bit_cast<CTRL_REG2>(this->read_byte(std::to_underlying(RA::CTRL_REG2)));
    }

    void L3G4200D::set_ctrl_reg2_register(CTRL_REG2 const ctrl_reg2) const noexcept
    {
        this->write_byte(std::to_underlying(RA::CTRL_REG2), std::bit_cast<std::uint8_t>(ctrl_reg2));
    }

    CTRL_REG3 L3G4200D::get_ctrl_reg3_register() const noexcept
    {
        return std::bit_cast<CTRL_REG3>(this->read_byte(std::to_underlying(RA::CTRL_REG3)));
    }

    void L3G4200D::set_ctrl_reg3_register(CTRL_REG3 const ctrl_reg3) const noexcept
    {
        this->write_byte(std::to_underlying(RA::CTRL_REG3), std::bit_cast<std::uint8_t>(ctrl_reg3));
    }

    CTRL_REG4 L3G4200D::get_ctrl_reg4_register() const noexcept
    {
        return std::bit_cast<CTRL_REG4>(this->read_byte(std::to_underlying(RA::CTRL_REG4)));
    }

    void L3G4200D::set_ctrl_reg4_register(CTRL_REG4 const ctrl_reg4) const noexcept
    {
        this->write_byte(std::to_underlying(RA::CTRL_REG4), std::bit_cast<std::uint8_t>(ctrl_reg4));
    }

    CTRL_REG5 L3G4200D::get_ctrl_reg5_register() const noexcept
    {
        return std::bit_cast<CTRL_REG5>(this->read_byte(std::to_underlying(RA::CTRL_REG5)));
    }

    void L3G4200D::set_ctrl_reg5_register(CTRL_REG5 const ctrl_reg5) const noexcept
    {
        this->write_byte(std::to_underlying(RA::CTRL_REG5), std::bit_cast<std::uint8_t>(ctrl_reg5));
    }

    REFERENCE L3G4200D::get_reference_register() const noexcept
    {
        return std::bit_cast<REFERENCE>(this->read_byte(std::to_underlying(RA::REFERENCE)));
    }

    OUT_TEMP L3G4200D::get_out_temp_register() const noexcept
    {
        return std::bit_cast<OUT_TEMP>(this->read_byte(std::to_underlying(RA::OUT_TEMP)));
    }

    STATUS_REG L3G4200D::get_status_reg_register() const noexcept
    {
        return std::bit_cast<STATUS_REG>(this->read_byte(std::to_underlying(RA::STATUS_REG)));
    }

    OUT_X L3G4200D::get_out_x_registers() const noexcept
    {
        return std::bit_cast<OUT_X>(this->read_bytes<sizeof(OUT_X)>(std::to_underlying(RA::OUT_X_L)));
    }

    OUT_Y L3G4200D::get_out_y_registers() const noexcept
    {
        return std::bit_cast<OUT_Y>(this->read_bytes<sizeof(OUT_Y)>(std::to_underlying(RA::OUT_Y_L)));
    }

    OUT_Z L3G4200D::get_out_z_registers() const noexcept
    {
        return std::bit_cast<OUT_Z>(this->read_bytes<sizeof(OUT_Z)>(std::to_underlying(RA::OUT_Z_L)));
    }

    OUT L3G4200D::get_out_registers() const noexcept
    {
        return std::bit_cast<OUT>(this->read_bytes<sizeof(OUT)>(std::to_underlying(RA::OUT_X_L)));
    }

    FIFO_CTRL_REG L3G4200D::get_fifo_ctrl_reg_register() const noexcept
    {
        return std::bit_cast<FIFO_CTRL_REG>(this->read_byte(std::to_underlying(RA::FIFO_CTRL_REG)));
    }

    void L3G4200D::set_fifo_ctrl_reg_register(FIFO_CTRL_REG const fifo_ctrl_reg) const noexcept
    {
        this->write_byte(std::to_underlying(RA::FIFO_CTRL_REG), std::bit_cast<std::uint8_t>(fifo_ctrl_reg));
    }

    FIFO_SRC_REG L3G4200D::get_fifo_src_reg_register() const noexcept
    {
        return std::bit_cast<FIFO_SRC_REG>(this->read_byte(std::to_underlying(RA::FIFO_SRC_REG)));
    }

    void L3G4200D::set_fifo_src_reg_register(FIFO_SRC_REG const fifo_src_reg) const noexcept
    {
        this->write_byte(std::to_underlying(RA::FIFO_SRC_REG), std::bit_cast<std::uint8_t>(fifo_src_reg));
    }

    INT1_CFG L3G4200D::get_int1_cfg_register() const noexcept
    {
        return std::bit_cast<INT1_CFG>(this->read_byte(std::to_underlying(RA::INT1_CFG)));
    }

    void L3G4200D::set_int1_cfg_register(INT1_CFG const int1_cfg) const noexcept
    {
        this->write_byte(std::to_underlying(RA::INT1_CFG), std::bit_cast<std::uint8_t>(int1_cfg));
    }

    INT1_SRC L3G4200D::get_int1_src_register() const noexcept
    {
        return std::bit_cast<INT1_SRC>(this->read_byte(std::to_underlying(RA::INT1_SRC)));
    }

    void L3G4200D::set_int1_src_register(INT1_SRC const in1_src) const noexcept
    {
        this->write_byte(std::to_underlying(RA::INT1_SRC), std::bit_cast<std::uint8_t>(in1_src));
    }

    INT1_THS_X L3G4200D::get_int1_ths_x_register() const noexcept
    {
        return std::bit_cast<INT1_THS_X>(this->read_bytes<sizeof(INT1_THS_X)>(std::to_underlying(RA::INT1_THS_XH)));
    }

    void L3G4200D::set_int1_ths_x_register(INT1_THS_X const int1_ths_x) const noexcept
    {
        this->write_bytes(std::to_underlying(RA::INT1_THS_XH),
                          std::bit_cast<std::array<std::uint8_t, sizeof(INT1_THS_X)>>(int1_ths_x));
    }

    INT1_THS_Y L3G4200D::get_int1_ths_y_register() const noexcept
    {
        return std::bit_cast<INT1_THS_Y>(this->read_bytes<sizeof(INT1_THS_Y)>(std::to_underlying(RA::INT1_THS_YH)));
    }

    void L3G4200D::set_int1_ths_y_register(INT1_THS_Y const int1_ths_y) const noexcept
    {
        this->write_bytes(std::to_underlying(RA::INT1_THS_YH),
                          std::bit_cast<std::array<std::uint8_t, sizeof(INT1_THS_Y)>>(int1_ths_y));
    }

    INT1_THS_Z L3G4200D::get_int1_ths_z_register() const noexcept
    {
        return std::bit_cast<INT1_THS_Z>(this->read_bytes<sizeof(INT1_THS_Z)>(std::to_underlying(RA::INT1_THS_ZH)));
    }

    void L3G4200D::set_int1_ths_z_register(INT1_THS_Z const int1_ths_z) const noexcept
    {
        this->write_bytes(std::to_underlying(RA::INT1_THS_ZH),
                          std::bit_cast<std::array<std::uint8_t, sizeof(INT1_THS_Z)>>(int1_ths_z));
    }

    INT1_DURATION L3G4200D::get_int1_duration_register() const noexcept
    {
        return std::bit_cast<INT1_DURATION>(this->read_byte(std::to_underlying(RA::INT1_DURATION)));
    }

    void L3G4200D::set_int1_duration(INT1_DURATION const in1_duration) const noexcept
    {
        this->write_byte(std::to_underlying(RA::INT1_DURATION), std::bit_cast<std::uint8_t>(in1_duration));
    }

}; // namespace L3G4200D