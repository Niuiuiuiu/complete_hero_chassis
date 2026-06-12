#include "bsp_imu.h"

#include "spi.h"

void BMI088_Init(BMI088_Handle_t *imu_handle) {

    imu_handle->hspi = &hspi1;

    imu_handle->acc_cs.port = GPIOA;
    imu_handle->acc_cs.pin = GPIO_PIN_4;

    imu_handle->gyro_cs.port = GPIOB;
    imu_handle->gyro_cs.pin = GPIO_PIN_0;
	
	imu_handle->acc_lsb_to_g = 6.0f / 32768.0f;
	imu_handle->gyro_lsb_to_dps = 2000.0f / 32768.0f * 0.01745329f;

    imu_handle->quat.q0 = 1.0f;
    imu_handle->quat.q1 = 0.0f;
    imu_handle->quat.q2 = 0.0f;
    imu_handle->quat.q3 = 0.0f;
	

}


//void BMI088_Start(BMI088_Handle_t *imu_handle) {
//    uint8_t dummy;


//    HAL_GPIO_WritePin(imu_handle->acc_cs.port, imu_handle->acc_cs.pin, GPIO_PIN_SET);
//    HAL_Delay(1);
//    HAL_GPIO_WritePin(imu_handle->acc_cs.port, imu_handle->acc_cs.pin, GPIO_PIN_RESET);
//    HAL_SPI_Receive(imu_handle->hspi, &dummy, 1, 10);
//    HAL_GPIO_WritePin(imu_handle->acc_cs.port, imu_handle->acc_cs.pin, GPIO_PIN_SET);
//    HAL_Delay(1);


//    BMI088_Write_Acc(imu_handle, 0x7E, 0xB6);
//    HAL_Delay(50);


//    BMI088_Write_Acc(imu_handle, 0x7D, 0x04);
//    HAL_Delay(50); 
//    
//    BMI088_Write_Acc(imu_handle, 0x7C, 0x00); 
//    HAL_Delay(50);

//    BMI088_Write_Acc(imu_handle, 0x41, 0x01); 
//    BMI088_Write_Acc(imu_handle, 0x40, 0xA2);


//    BMI088_Write_Acc(imu_handle, 0x58, 0x04); 

//    BMI088_Write_Acc(imu_handle, 0x53, 0x0A); 

//    BMI088_Write_Gyro(imu_handle, 0x14, 0xB6); 
//    HAL_Delay(50);

//    BMI088_Write_Gyro(imu_handle, 0x0F, 0x00);
//    BMI088_Write_Gyro(imu_handle, 0x10, 0x03);


//    BMI088_Write_Gyro(imu_handle, 0x15, 0x80);
//    

//    BMI088_Write_Gyro(imu_handle, 0x18, 0x01); 

//    BMI088_Write_Gyro(imu_handle, 0x16, 0x01);
//}

void BMI088_Start(BMI088_Handle_t *imu_handle) {
    uint8_t dummy;

    HAL_GPIO_WritePin(imu_handle->acc_cs.port, imu_handle->acc_cs.pin, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(imu_handle->acc_cs.port, imu_handle->acc_cs.pin, GPIO_PIN_RESET);

    HAL_SPI_Receive(imu_handle->hspi, &dummy, 1, 10);
    HAL_GPIO_WritePin(imu_handle->acc_cs.port, imu_handle->acc_cs.pin, GPIO_PIN_SET);
    HAL_Delay(50);

    BMI088_Write_Acc(imu_handle, 0x7E, 0xB6); 
    HAL_Delay(150);

    HAL_GPIO_WritePin(imu_handle->acc_cs.port, imu_handle->acc_cs.pin, GPIO_PIN_RESET);
    HAL_SPI_Receive(imu_handle->hspi, &dummy, 1, 10);
    HAL_GPIO_WritePin(imu_handle->acc_cs.port, imu_handle->acc_cs.pin, GPIO_PIN_SET);
    HAL_Delay(10);

    BMI088_Write_Acc(imu_handle, 0x7D, 0x04);
    HAL_Delay(50); 


    BMI088_Write_Acc(imu_handle, 0x41, 0x01);
    BMI088_Write_Acc(imu_handle, 0x40, 0x9B);

    BMI088_Write_Acc(imu_handle, 0x53, 0x0A);
    BMI088_Write_Acc(imu_handle, 0x58, 0x04); 

    BMI088_Write_Gyro(imu_handle, 0x14, 0xB6); 
    HAL_Delay(80); 
    BMI088_Write_Gyro(imu_handle, 0x0F, 0x00); 
    BMI088_Write_Gyro(imu_handle, 0x10, 0x03); 
    BMI088_Write_Gyro(imu_handle, 0x15, 0x80); 
    BMI088_Write_Gyro(imu_handle, 0x16, 0x0C);
    BMI088_Write_Gyro(imu_handle, 0x18, 0x01); 
}

uint8_t BMI088_Read_Acc(BMI088_Handle_t *imu_handle, uint8_t reg) {

    uint8_t addr = reg | 0x80;
    uint8_t res = 0;
    uint8_t dummy_buf[2];

    HAL_GPIO_WritePin(imu_handle->acc_cs.port, imu_handle->acc_cs.pin, GPIO_PIN_RESET);

    HAL_SPI_Transmit(imu_handle->hspi, &addr, 1, HAL_MAX_DELAY);

    HAL_SPI_Receive(imu_handle->hspi, dummy_buf, 2, HAL_MAX_DELAY);
    res = dummy_buf[1];

    HAL_GPIO_WritePin(imu_handle->acc_cs.port, imu_handle->acc_cs.pin, GPIO_PIN_SET);

    return res;
}

uint8_t BMI088_Read_Gyro(BMI088_Handle_t *imu_handle, uint8_t reg) {

    uint8_t addr = reg | 0x80;
    uint8_t res = 0;
    HAL_GPIO_WritePin(imu_handle->gyro_cs.port, imu_handle->gyro_cs.pin, GPIO_PIN_RESET);

    HAL_SPI_Transmit(imu_handle->hspi, &addr, 1, HAL_MAX_DELAY);

    HAL_SPI_Receive(imu_handle->hspi, &res, 1, HAL_MAX_DELAY);

    HAL_GPIO_WritePin(imu_handle->gyro_cs.port, imu_handle->gyro_cs.pin, GPIO_PIN_SET);

    return res;
}

void BMI088_Write_Acc(BMI088_Handle_t *imu_handle, uint8_t reg, uint8_t data) {

    uint8_t pData[2];

    pData[0] = reg & 0x7F;
    pData[1] = data;

    HAL_GPIO_WritePin(imu_handle->acc_cs.port, imu_handle->acc_cs.pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(imu_handle->hspi, pData, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(imu_handle->acc_cs.port, imu_handle->acc_cs.pin, GPIO_PIN_SET);
}

void BMI088_Write_Gyro(BMI088_Handle_t *imu_handle, uint8_t reg, uint8_t data) {

    uint8_t pData[2];

    pData[0] = reg & 0x7F;
    pData[1] = data;

    HAL_GPIO_WritePin(imu_handle->gyro_cs.port, imu_handle->gyro_cs.pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(imu_handle->hspi, pData, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(imu_handle->gyro_cs.port, imu_handle->gyro_cs.pin, GPIO_PIN_SET);
}


void BMI088_Read_Acc_Raw(BMI088_Handle_t *imu_handle) {
    uint8_t tx_buf[8] = {0x12 | 0x80, 0, 0, 0, 0, 0, 0, 0};
    uint8_t rx_buf[8] = {0};

    HAL_GPIO_WritePin(imu_handle->acc_cs.port, imu_handle->acc_cs.pin, GPIO_PIN_RESET);

    HAL_SPI_TransmitReceive(imu_handle->hspi, tx_buf, rx_buf, 8, 10);
    HAL_GPIO_WritePin(imu_handle->acc_cs.port, imu_handle->acc_cs.pin, GPIO_PIN_SET);


    imu_handle->acc_raw.x = (int16_t)((rx_buf[3] << 8) | rx_buf[2]);
    imu_handle->acc_raw.y = (int16_t)((rx_buf[5] << 8) | rx_buf[4]);
    imu_handle->acc_raw.z = (int16_t)((rx_buf[7] << 8) | rx_buf[6]);
}

void BMI088_Read_Gyro_Raw(BMI088_Handle_t *imu_handle) {
    uint8_t tx_buf[7] = {0};
    uint8_t rx_buf[7] = {0};

    tx_buf[0] = 0x02 | 0x80;

    HAL_GPIO_WritePin(imu_handle->gyro_cs.port, imu_handle->gyro_cs.pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(imu_handle->hspi, tx_buf, rx_buf, 7, 10);
    HAL_GPIO_WritePin(imu_handle->gyro_cs.port, imu_handle->gyro_cs.pin, GPIO_PIN_SET);
    
    imu_handle->gyro_raw.x = (int16_t)((rx_buf[2] << 8) | rx_buf[1]);
    imu_handle->gyro_raw.y = (int16_t)((rx_buf[4] << 8) | rx_buf[3]);
    imu_handle->gyro_raw.z = (int16_t)((rx_buf[6] << 8) | rx_buf[5]);
}

void BMI088_Restart(BMI088_Handle_t *imu_handle) {
    BMI088_Write_Acc(imu_handle, 0x7E, 0xB6);
    BMI088_Write_Gyro(imu_handle, 0x14, 0xB6);

    HAL_Delay(100);

    BMI088_Start(imu_handle);
}




void BMI088_Data_Convert(BMI088_Handle_t *imu) {
	
    imu->acc.x = (float)imu->acc_raw.x * imu->acc_lsb_to_g;
    imu->acc.y = (float)imu->acc_raw.y * imu->acc_lsb_to_g;
    imu->acc.z = (float)imu->acc_raw.z * imu->acc_lsb_to_g;

    imu->gyro.x = (float)imu->gyro_raw.x * imu->gyro_lsb_to_dps;
    imu->gyro.y = (float)imu->gyro_raw.y * imu->gyro_lsb_to_dps;
    imu->gyro.z = (float)imu->gyro_raw.z * imu->gyro_lsb_to_dps;
}


void BMI088_Read_Temp(BMI088_Handle_t *imu_handle) {
    uint8_t buf[2];
    uint16_t temp_raw;
    
    buf[0] = BMI088_Read_Acc(imu_handle, 0x22); 
    buf[1] = BMI088_Read_Acc(imu_handle, 0x23);

    temp_raw = (uint16_t)((buf[0] << 3) | (buf[1] >> 5));


    float temperature = (float)temp_raw * 0.125f + 23.0f;
    
    imu_handle->temperature = temperature;
}



void Temp_Control_Task(BMI088_Handle_t *imu_handle) {
    float current_temp = imu_handle->temperature;
    
    if (current_temp < 39.5f) {
        __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, 1500);
    } 
    else if (current_temp > 40.5f) {
        __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, 0);
    }
}





