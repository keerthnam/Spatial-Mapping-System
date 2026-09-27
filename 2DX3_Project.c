


#include <stdint.h>
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"
#include "tm4c1294ncpdt.h"
#include "VL53L1X_api.h"

#define I2C_MCS_ACK             0x00000008
#define I2C_MCS_DATACK          0x00000008
#define I2C_MCS_ADRACK          0x00000004
#define I2C_MCS_STOP            0x00000004
#define I2C_MCS_START           0x00000002
#define I2C_MCS_ERROR           0x00000002
#define I2C_MCS_RUN             0x00000001
#define I2C_MCS_BUSY            0x00000001
#define I2C_MCR_MFE             0x00000010

#define MAXRETRIES              5


// CW delay: 2 x 120000 ticks = ~2ms per step (scanning speed)
// CCW delay: 2 x 240000 ticks = ~4ms per step (homing, slower = more torque)
#define CW_DELAY  120000
#define CCW_DELAY 240000

void I2C_Init(void){
    SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;
    while((SYSCTL_PRGPIO_R&0x0002) == 0){};
    GPIO_PORTB_AFSEL_R |= 0x0C;
    GPIO_PORTB_ODR_R |= 0x08;
    GPIO_PORTB_DEN_R |= 0x0C;
    GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00002200;
    I2C0_MCR_R = I2C_MCR_MFE;
    I2C0_MTPR_R = 0b0000000000000101000000000111011;
}

void PortG_Init(void){
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R6) == 0){};
    GPIO_PORTG_DIR_R &= 0x00;
    GPIO_PORTG_AFSEL_R &= ~0x01;
    GPIO_PORTG_DEN_R |= 0x01;
    GPIO_PORTG_AMSEL_R &= ~0x01;
    return;
}

void VL53L1X_XSHUT(void){
    GPIO_PORTG_DIR_R |= 0x01;
    GPIO_PORTG_DATA_R &= 0b11111110;
    FlashAllLEDs();
    SysTick_Wait10ms(10);
    GPIO_PORTG_DIR_R &= ~0x01;
}

void PortH_Init(void){
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7;
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R7) == 0){};
    GPIO_PORTH_DIR_R |= 0x0F;
    GPIO_PORTH_AFSEL_R &= ~0x0F;
    GPIO_PORTH_DEN_R |= 0x0F;
    GPIO_PORTH_AMSEL_R &= ~0x0F;
    return;
}

void PortJ_Init(void){
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R8;
    while((SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R8) == 0){};
    GPIO_PORTJ_CR_R |= 0x01;
    GPIO_PORTJ_DIR_R = 0b00000000;
    GPIO_PORTJ_DEN_R = 0b00000011;
    GPIO_PORTJ_AMSEL_R &= ~0x03;
    GPIO_PORTJ_PUR_R |= 0x03;
    return;
}


// direction =  1 -> CW  (forward scan)
// direction = -1 -> CCW (homing)
// delay = SysTick ticks to wait per step
//         uses CW_DELAY for scanning, CCW_DELAY for homing

// Slower delay = more torque = motor less likely to vibrate and stall.
void spin(int direction, uint32_t delay){
    static int curr_step = 0;

    // Coil sequence for CW: 0011 -> 0110 -> 1100 -> 1001
    uint8_t coils[4] = {0b00000011, 0b00000110, 0b00001100, 0b00001001};

    // Drive current coil position
    GPIO_PORTH_DATA_R = coils[curr_step];

    // Advance in requested direction
    if(direction == 1){
        curr_step = (curr_step + 1) % 4;   // CW
    } else {
        curr_step = (curr_step + 3) % 4;   // CCW (+3 mod 4 = -1 mod 4)
    }

    SysTick_Wait(delay);
    SysTick_Wait(delay);
}

int motor_on = 0;
int last_state_0 = 1;
int step_count = 0;
int blink_interval = 64;

void button_0(void){
    uint8_t button = GPIO_PORTJ_DATA_R & 0b00000001;
    if(last_state_0 == 1 && button == 0){
        motor_on ^= 1;
        if(motor_on == 1){
            GPIO_PORTN_DATA_R |= 0b00000001;   // PN0 on - running
        } else {
            GPIO_PORTN_DATA_R &= ~0b00000001;  // PN0 off - stopped
        }
    }
    last_state_0 = button;
    SysTick_Wait10ms(1);
}

uint16_t dev = 0x29;
int status = 0;

int main(void) {
    uint8_t sensorState = 0;
    uint16_t wordData;
    uint16_t Distance;
    uint16_t SignalRate;
    uint16_t AmbientRate;
    uint16_t SpadNum;
    uint8_t RangeStatus;
    uint8_t dataReady = 0;
    uint8_t modelID, moduleType;

   
    PLL_Init();
    SysTick_Init();
    onboardLEDs_Init();
    I2C_Init();
    UART_Init();
    PortH_Init();
    PortJ_Init();

    status = VL53L1_RdByte(dev, 0x010F, &modelID);
    status = VL53L1_RdByte(dev, 0x0110, &moduleType);
    status = VL53L1_RdWord(dev, 0x010F, &wordData);
    status = VL53L1X_GetSensorId(dev, &wordData);

    while(sensorState == 0){
        status = VL53L1X_BootState(dev, &sensorState);
        SysTick_Wait10ms(10);
    }
    FlashAllLEDs();   // boot confirmed

    status = VL53L1X_ClearInterrupt(dev);
    status = VL53L1X_SensorInit(dev);
    Status_Check("SensorInit", status);
    status = VL53L1X_SetDistanceMode(dev, 2);
    status = VL53L1X_SetTimingBudgetInMs(dev, 200);
    status = VL53L1X_StartRanging(dev);

    while(1){

        button_0();

        if(motor_on == 1){

            // SCAN: one CW step at normal speed
            spin(1, CW_DELAY);
            step_count++;

            int position_in_cycle = step_count % blink_interval;

            // Every 11.25 degrees: measure and transmit
            if(position_in_cycle == 0){

                // PN1 on - measurement status LED
                GPIO_PORTN_DATA_R |= 0b00000010;

                dataReady = 0;
                while(dataReady == 0){
                    status = VL53L1X_CheckForDataReady(dev, &dataReady);
                    VL53L1_WaitMs(dev, 5);
                }

                status = VL53L1X_GetRangeStatus(dev, &RangeStatus);
                status = VL53L1X_GetDistance(dev, &Distance);
                status = VL53L1X_GetSignalRate(dev, &SignalRate);
                status = VL53L1X_GetAmbientRate(dev, &AmbientRate);
                status = VL53L1X_GetSpadNb(dev, &SpadNum);
                status = VL53L1X_ClearInterrupt(dev);

                // PN1 off
                GPIO_PORTN_DATA_R &= ~0b00000010;

                // PF4 flash - UART Tx status LED
                GPIO_PORTF_DATA_R |= 0b00010000;
                SysTick_Wait10ms(1);
                GPIO_PORTF_DATA_R &= ~0b00010000;

                sprintf(printf_buffer, "%u, %u, %u, %u, %u\r\n",
                        RangeStatus, Distance, SignalRate, AmbientRate, SpadNum);
                UART_printf(printf_buffer);
                SysTick_Wait10ms(80);
            }

    
            if(step_count == 2048){
                motor_on = 0;
                step_count = 0;

                // HOME: spin CCW 2048 steps at slower speed to unwind wires
               
                GPIO_PORTN_DATA_R |= 0b00000001;

                for(int j = 0; j < 2048; j++){
                    spin(-1, CCW_DELAY);   // CCW, slower than CW

                    if(j % 256 == 0){
                        GPIO_PORTN_DATA_R ^= 0b00000001;
                    }
                }

                // Power off coils
                GPIO_PORTH_DATA_R = 0x00;

                // PN0 off
                GPIO_PORTN_DATA_R &= ~0b00000001;

                // All LEDs flash - homing done
                FlashAllLEDs();
            }
        }

      
    }

    VL53L1X_StopRanging(dev);
    while(1){}
}
