#ifndef L3G4200D_REGISTERS_HPP
#define L3G4200D_REGISTERS_HPP

#include <cstdint>

#define PACKED __attribute__((__packed__))

namespace L3G4200D {

    struct WHO_AM_I {
        std::uint8_t who_am_i : 8;
    } PACKED;

    struct CTRL_REG1 {
        std::uint8_t dr : 2;
        std::uint8_t bw : 2;
        std::uint8_t pd : 1;
        std::uint8_t z_en : 1;
        std::uint8_t y_en : 1;
        std::uint8_t x_en : 1;
    } PACKED;

    struct CTRL_REG2 {
        std::uint8_t : 2;
        std::uint8_t hpm : 2;
        std::uint8_t hpcf : 4;
    } PACKED;

    struct CTRL_REG3 {
        std::uint8_t i1_int1 : 1;
        std::uint8_t i1_boot : 1;
        std::uint8_t hl_active : 1;
        std::uint8_t pp_od : 1;
        std::uint8_t i2_drdy : 1;
        std::uint8_t i2_wtm : 1;
        std::uint8_t i2_orun : 1;
        std::uint8_t i2_empty : 1;
    } PACKED;

    struct CTRL_REG4 {
        std::uint8_t bdu : 1;
        std::uint8_t ble : 1;
        std::uint8_t fs : 2;
        std::uint8_t : 1;
        std::uint8_t st : 2;
        std::uint8_t sim : 1;
    } PACKED;

    struct CTRL_REG5 {
        std::uint8_t boot : 1;
        std::uint8_t fifo_en : 1;
        std::uint8_t : 1;
        std::uint8_t hp_en : 1;
        std::uint8_t int1_sel : 2;
        std::uint8_t out_sel : 2;
    } PACKED;

    struct REFERENCE {
        std::uint8_t : 8;
    } PACKED;

    struct OUT_TEMP {
        std::uint8_t temp : 8;
    } PACKED;

    struct STATUS_REG {
        std::uint8_t zyx_or : 1;
        std::uint8_t z_or : 1;
        std::uint8_t y_or : 1;
        std::uint8_t x_or : 1;
        std::uint8_t zyx_da : 1;
        std::uint8_t z_da : 1;
        std::uint8_t y_da : 1;
        std::uint8_t x_da : 1;
    } PACKED;

    struct OUT_X {
        std::uint16_t out_x : 16;
    } PACKED;

    struct OUT_Y {
        std::uint16_t out_y : 16;
    } PACKED;

    struct OUT_Z {
        std::uint16_t out_z : 16;
    } PACKED;

    struct OUT {
        OUT_X out_x{};
        OUT_Y out_y{};
        OUT_Z out_z{};
    } PACKED;

    struct FIFO_CTRL_REG {
        std::uint8_t fm : 3;
        std::uint8_t wtm : 5;
    } PACKED;

    struct FIFO_SRC_REG {
        std::uint8_t wtm : 1;
        std::uint8_t ovrn : 1;
        std::uint8_t empty : 1;
        std::uint8_t fss : 5;
    } PACKED;

    struct INT1_CFG {
        std::uint8_t and_or : 1;
        std::uint8_t lir : 1;
        std::uint8_t z_hie : 1;
        std::uint8_t z_lie : 1;
        std::uint8_t y_hie : 1;
        std::uint8_t y_lie : 1;
        std::uint8_t x_hie : 1;
        std::uint8_t x_lie : 1;
    } PACKED;

    struct INT1_SRC {
        std::uint8_t : 1;
        std::uint8_t ia : 1;
        std::uint8_t z_h : 1;
        std::uint8_t z_l : 1;
        std::uint8_t y_h : 1;
        std::uint8_t y_k : 1;
        std::uint8_t x_h : 1;
        std::uint8_t x_l : 1;
    } PACKED;

    struct INT1_THS_X {
        std::uint8_t : 1;
        std::uint8_t ths_x : 15;
    } PACKED;

    struct INT1_THS_Y {
        std::uint8_t : 1;
        std::uint8_t ths_y : 15;
    } PACKED;

    struct INT1_THS_Z {
        std::uint8_t : 1;
        std::uint8_t ths_z : 15;
    } PACKED;

    struct INT1_DURATION {
        std::uint8_t wait : 1;
        std::uint8_t d : 7;
    } PACKED;

}; // namespace L3G4200D

#undef PACKED

#endif // L3G4200D_REGISTERS_HPP