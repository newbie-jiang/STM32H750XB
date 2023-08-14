

# 项目记录

## 环境搭建：

- 硬件平台：STM32H750XBH6
- 开发环境：STM32CubeMX V6.8.1+KEIL V5.28.0.0
- STM32H750固件版本：package V1.11.0
- 仿真下载驱动：ST-Link



# 使用串口中断方式，

使用类似json数据格式来控制LED，扩展其他外设依葫芦画瓢

1. **首先**, 配置USART为中断模式。使用STM32CubeMX或手动配置。
2. **初始化代码和变量**:

```
UART_HandleTypeDef huart4; // Assuming USART1 is being used
uint8_t rx_data;
uint8_t rx_buffer[128];
uint32_t rx_index = 0;
```

1. **开启UART中断模式以接收数据**:

```
cCopy code
HAL_UART_Receive_IT(&huart4, &rx_data, 1);
```

1. **在中断回调中处理数据**:

```
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART4) 
    {
        // Add received byte to the buffer
        rx_buffer[rx_index++] = rx_data;

        // Check if received a complete JSON message e.g., {"LED":"ON"} or {"LED":"OFF"}
        if (rx_data == '}' && rx_index < sizeof(rx_buffer)) 
        {
            rx_buffer[rx_index] = '\0';  // Null terminate the string
            if (strstr((char *)rx_buffer, "{\"LED\":\"ON\"}")) 
            {
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET); // Assuming LED is on pin GPIOB_PIN_0
            } 
            else if (strstr((char *)rx_buffer, "{\"LED\":\"OFF\"}")) 
            {
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
            }
            HAL_UART_Transmit_IT(&huart4,(uint8_t*)rx_buffer,sizeof(rx_buffer));/*数据回传*/
            rx_index = 0; // Reset the buffer index
        }

        // Restart interrupt for next byte
        HAL_UART_Receive_IT(&huart4, &rx_data, 1);
    }
}
```

1. **确保你已正确配置LED的GPIO**。上面的示例假设LED连接到GPIOB_PIN_0。
2. **发送JSON数据**:

使用一个串口终端工具（例如Putty、Tera Term或其他）发送以下JSON字符串来控制LED:

```
{"LED":"ON"}
{"LED":"OFF"}
```

