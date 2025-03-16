#include "main.h"
#include "gpio.h"
#include "i2c.h"
#include "l3g4200d.hpp"
#include "usart.h"

namespace {

    volatile auto interrupt = false;
};

void HAL_GPIO_EXIT_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == 1 << 5) {
        interrupt = true;
    }
}

int main()
{
    HAL_Init();
    SystemClock_Config();

    MX_USART2_UART_Init();
    MX_I2C1_Init();
    MX_GPIO_Init();

    using namespace L3G4200D;

    auto i2c_device = I2CDevice{};

    auto config = Config{};

    auto l3g4200d = L3G4200D::L3G4200D{std::move(i2c_device), config};

    while (true) {
        if (interrupt && l3g4200d.is_data_ready()) {
            auto const& [x, y, z] = l3g4200d.get_rotation_scaled().value();
            std::printf("Rotation x: %.2f, y: %.2f, z: %.2f\n\r", x, y, z);
            interrupt = false;
        }
    }

    return 0;
}
