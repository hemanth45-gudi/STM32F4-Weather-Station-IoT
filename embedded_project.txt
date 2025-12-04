#include "stm32f4xx.h"
#include <stdio.h>

#define PCF8574_ADDR 0x4E
#define DHT22_PIN 4
#define LDR_PIN 0
#define RAIN_ANALOG_PIN 1
#define LED_PIN 13
#define BUZZER_PIN 14   // PC14

volatile uint32_t systick_count = 0;
volatile uint32_t sensor_read_time = 0;
volatile uint8_t sensor_read_flag = 0;

typedef struct {
    float temperature;
    float humidity;
    uint16_t ldr_value;
    uint16_t rain_analog_value;
    uint8_t rain_digital_status;
    uint8_t dht_status;
} SensorData;

SensorData sensor_data = {0};

// ------------------ Delay ------------------
void delay_ms(uint32_t ms) { for(uint32_t i=0;i<ms*8000;i++) __NOP(); }
void delay_us(uint32_t us) { for(volatile uint32_t i=0;i<us*2;i++); }

// ------------------ I2C & LCD ------------------
void I2C1_Init(void) {
    RCC->AHB1ENR |= (1<<1);
    RCC->APB1ENR |= (1<<21);
    GPIOB->MODER &= ~((3<<12)|(3<<14));
    GPIOB->MODER |=  ((2<<12)|(2<<14));
    GPIOB->OTYPER |= (1<<6)|(1<<7);
    GPIOB->OSPEEDR |= (3<<12)|(3<<14);
    GPIOB->PUPDR   |= (1<<12)|(1<<14);
    GPIOB->AFR[0]  |= (4<<24)|(4<<28);
    I2C1->CR1 |= (1<<15);
    I2C1->CR1 &= ~(1<<15);
    I2C1->CR2 = 16;
    I2C1->CCR = 80;
    I2C1->TRISE = 17;
    I2C1->CR1 |= (1<<0);
}

void I2C1_Write(uint8_t addr, uint8_t data) {
    while(I2C1->SR2 & (1<<1));
    I2C1->CR1 |= (1<<8);
    while(!(I2C1->SR1 & (1<<0)));
    I2C1->DR = addr;
    while(!(I2C1->SR1 & (1<<1)));
    (void)I2C1->SR2;
    while(!(I2C1->SR1 & (1<<7)));
    I2C1->DR = data;
    while(!(I2C1->SR1 & (1<<2)));
    I2C1->CR1 |= (1<<9);
}

void LCD_SendCmd(uint8_t cmd) {
    uint8_t high = (cmd & 0xF0);
    uint8_t low  = ((cmd << 4) & 0xF0);
    I2C1_Write(PCF8574_ADDR, high | 0x0C);
    delay_ms(1);
    I2C1_Write(PCF8574_ADDR, high | 0x08);
    I2C1_Write(PCF8574_ADDR, low | 0x0C);
    delay_ms(1);
    I2C1_Write(PCF8574_ADDR, low | 0x08);
}

void LCD_SendData(uint8_t data) {
    uint8_t high = (data & 0xF0);
    uint8_t low  = ((data << 4) & 0xF0);
    I2C1_Write(PCF8574_ADDR, high | 0x0D);
    delay_ms(1);
    I2C1_Write(PCF8574_ADDR, high | 0x09);
    I2C1_Write(PCF8574_ADDR, low | 0x0D);
    delay_ms(1);
    I2C1_Write(PCF8574_ADDR, low | 0x09);
}

void LCD_Init(void) {
    delay_ms(50);
    LCD_SendCmd(0x33);
    LCD_SendCmd(0x32);
    LCD_SendCmd(0x28);
    LCD_SendCmd(0x0C);
    LCD_SendCmd(0x06);
    LCD_SendCmd(0x01);
    delay_ms(5);
}

void LCD_SetCursor(uint8_t row, uint8_t col) {
    uint8_t addr = (row == 0 ? 0x80 : 0xC0) + col;
    LCD_SendCmd(addr);
}

void LCD_Print(char *str) { while(*str) LCD_SendData(*str++); }

// ------------------ SysTick ------------------
void SysTick_Handler(void) {
    systick_count++;
    if (systick_count - sensor_read_time >= 3000) {
        sensor_read_flag = 1;
        sensor_read_time = systick_count;
    }
}

void SysTick_Init(uint32_t ticks) {
    SysTick->CTRL = 0;
    SysTick->LOAD = ticks - 1;
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
}

// ------------------ DHT22 ------------------
uint8_t Read_DHT22(float *temperature, float *humidity) {
    uint8_t data[5] = {0};
    uint32_t timeout;
    RCC->AHB1ENR |= (1 << 1);
    GPIOB->MODER &= ~(3UL << (2 * DHT22_PIN));
    GPIOB->MODER |=  (1UL << (2 * DHT22_PIN));
    GPIOB->OTYPER &= ~(1 << DHT22_PIN);
    GPIOB->OSPEEDR |= (3UL << (2 * DHT22_PIN));
    GPIOB->BSRR = (1 << (DHT22_PIN + 16));
    delay_ms(1);
    GPIOB->BSRR = (1 << DHT22_PIN);
    delay_us(30);
    GPIOB->MODER &= ~(3UL << (2 * DHT22_PIN));
    GPIOB->PUPDR &= ~(3UL << (2 * DHT22_PIN));
    GPIOB->PUPDR |= (1UL << (2 * DHT22_PIN));

    timeout = 100;
    while(GPIOB->IDR & (1 << DHT22_PIN)) { if(--timeout == 0) return 1; delay_us(1); }
    timeout = 100;
    while(!(GPIOB->IDR & (1 << DHT22_PIN))) { if(--timeout == 0) return 2; delay_us(1); }
    timeout = 100;
    while(GPIOB->IDR & (1 << DHT22_PIN)) { if(--timeout == 0) return 3; delay_us(1); }

    for(int i=0;i<5;i++){
        for(int j=0;j<8;j++){
            timeout = 100;
            while(!(GPIOB->IDR & (1 << DHT22_PIN))) { if(--timeout == 0) return 4; delay_us(1); }
            uint32_t width=0;
            while(GPIOB->IDR & (1 << DHT22_PIN)) { width++; delay_us(1); if(width>100) break; }
            if(width > 40) data[i] |= (1 << (7 - j));
        }
    }

    uint8_t checksum = data[0]+data[1]+data[2]+data[3];
    if(data[4] != checksum) return 5;

    int16_t raw_h = (data[0]<<8)|data[1];
    int16_t raw_t = (data[2]<<8)|data[3];
    if(raw_t & 0x8000) raw_t = -(raw_t & 0x7FFF);
    *humidity = raw_h / 10.0f;
    *temperature = raw_t / 10.0f;
    return 0;
}

// ------------------ ADC ------------------
void ADC_Init(void) {
    RCC->APB2ENR |= (1<<8);
    ADC1->CR2 = 0;
    ADC1->CR1 = 0;
    ADC1->CR2 |= (1<<0);
    delay_ms(1);
}

uint16_t Read_ADC(uint8_t channel) {
    ADC1->SQR3 = channel;
    ADC1->SQR1 = 0;
    ADC1->CR2 |= (1<<30);
    while(!(ADC1->SR & (1<<1)));
    return ADC1->DR;
}

uint8_t ADC_to_Percent(uint16_t v) {
    uint32_t p = (4095 - v)*100/4095;
    if(p>100) p=100;
    return (uint8_t)p;
}

// ------------------ UART ------------------
void UART1_Init(void) {
    RCC->AHB1ENR |= (1<<0);
    RCC->APB2ENR |= (1<<4);
    GPIOA->MODER &= ~((3<<(9*2))|(3<<(10*2)));
    GPIOA->MODER |=  ((2<<(9*2))|(2<<(10*2)));
    GPIOA->AFR[1] &= ~((0xF<<((9-8)*4))|(0xF<<((10-8)*4)));
    GPIOA->AFR[1] |=  ((7<<((9-8)*4))|(7<<((10-8)*4)));
    USART1->BRR = 0x683; // 9600 baud @16MHz
    USART1->CR1 = (1<<13)|(1<<3)|(1<<2);
}

void UART1_SendString(char *s) {
    while(*s){
        while(!(USART1->SR & (1<<7)));
        USART1->DR = *s++;
    }
}

// ------------------ LED & BUZZER ------------------
void LED_Init(void) {
    RCC->AHB1ENR |= (1<<2);
    GPIOC->MODER &= ~(3<<(13*2));
    GPIOC->MODER |=  (1<<(13*2));
}

void Buzzer_Init(void) {
    RCC->AHB1ENR |= (1<<2);
    GPIOC->MODER &= ~(3<<(BUZZER_PIN*2));
    GPIOC->MODER |=  (1<<(BUZZER_PIN*2));
}

// ------------------ Sensors ------------------
const char* Get_Rain_Status(uint8_t rain_percent) {
    if(rain_percent < 20) return "HEAVY RAIN";
    else if(rain_percent < 40) return "LIGHT RAIN";
    else if(rain_percent < 70) return "DRIZZLE";
    else return "CLEAR";
}

void Read_All_Sensors(void) {
    sensor_data.dht_status = Read_DHT22(&sensor_data.temperature, &sensor_data.humidity);
    sensor_data.ldr_value = Read_ADC(LDR_PIN);
    sensor_data.rain_analog_value = Read_ADC(RAIN_ANALOG_PIN);
    uint8_t rain_percent = ADC_to_Percent(sensor_data.rain_analog_value);
    sensor_data.rain_digital_status = (rain_percent < 40) ? 1 : 0; // Only heavy/light rain
}

void Display_Sensor_Data_LCD(void) {
    char buf[16];
    LCD_SendCmd(0x01);
    LCD_SetCursor(0,0);
    if(sensor_data.dht_status==0) {
        uint8_t temp_percent = (uint8_t)((sensor_data.temperature/50.0f)*100);
        if(temp_percent > 100) temp_percent = 100;
        sprintf(buf,"T:%3d%% H:%3d%%",(int)temp_percent,(int)sensor_data.humidity);
    } else {
        sprintf(buf,"DHT Err:%d",sensor_data.dht_status);
    }
    LCD_Print(buf);

    LCD_SetCursor(1,0);
    uint8_t light_p=ADC_to_Percent(sensor_data.ldr_value);
    uint8_t rain_p=ADC_to_Percent(sensor_data.rain_analog_value);
    sprintf(buf,"L:%3d%% R:%3d%%",light_p,rain_p);
    LCD_Print(buf);
}

// ------------------ MAIN ------------------
int main(void) {
    I2C1_Init();
    LCD_Init();
    LED_Init();
    Buzzer_Init();
    UART1_Init();
    SysTick_Init(16000);
    RCC->AHB1ENR |= (1<<0)|(1<<1)|(1<<2);
    GPIOA->MODER |= (3UL<<(2*LDR_PIN));
    GPIOA->MODER |= (3UL<<(2*RAIN_ANALOG_PIN));
    ADC_Init();
    __enable_irq();

    LCD_SendCmd(0x01);
    LCD_SetCursor(0,0); LCD_Print("Weather Station");
    LCD_SetCursor(1,0); LCD_Print("System Ready");
    for(int i=0;i<3;i++){ GPIOC->ODR^=(1<<13); delay_ms(200); }

    while(1){
        if(sensor_read_flag){
            sensor_read_flag=0;
            GPIOC->ODR^=(1<<13);
            Read_All_Sensors();
            Display_Sensor_Data_LCD();

            uint8_t rain_p = ADC_to_Percent(sensor_data.rain_analog_value);
            const char* rain_status = Get_Rain_Status(rain_p);

            // ?? Buzzer control: ON only for heavy or light rain
            if(rain_p < 40) 
                GPIOC->ODR |= (1<<BUZZER_PIN);
            else 
                GPIOC->ODR &= ~(1<<BUZZER_PIN);

            char txbuf[128];
            sprintf(txbuf,"Temp:%.1fC Hum:%.1f%% Light:%d%% Rain:%d%% Status:%s\r\n",
                    sensor_data.temperature, sensor_data.humidity,
                    ADC_to_Percent(sensor_data.ldr_value),
                    rain_p, rain_status);
            UART1_SendString(txbuf);
        }
        delay_ms(100);
    }
}