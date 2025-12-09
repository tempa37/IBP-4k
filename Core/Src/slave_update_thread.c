#include "main.h"
#include "cmsis_os.h"
#include "stdbool.h"
#include "string.h"
#include "modbuc.h"
#include "uart.h"
#include "slave_update_thread.h"





//extern SlaveSystem_t slave = {0};










/**
 * @brief Отправка данных слейву по UART с CRC16 (Modbus)
 * @param slave_num: номер слейва
 * @param data: указатель на данные
 * @param length: длина данных
 */
void Slave_SendBootloaderCommand(uint8_t slave_num, const uint8_t *data, uint8_t length)
{
    if (slave_num >= UART_CHANNEL_COUNT) return;
    
    UartContext_t *ctx = &uartContexts[slave_num];
    if (!ctx || !ctx->handle) return;
    
    if (osMutexWait(ctx->mutex, osWaitForever) == osOK)
    {
        HAL_UART_Transmit(ctx->handle, (uint8_t *)data, length, 1000u);
        osMutexRelease(ctx->mutex);
    }
}

/**
 * @brief Проверка получения ACK от слейва (0x79)
 * @param slave_num: номер слейва
 * @return true если ACK получен
 */
bool Slave_CheckACK(uint8_t slave_num)
{
    if (slave_num >= UART_CHANNEL_COUNT) return false;
    
    // Проверяем, получили ли мы ACK (0x79)
    if (slave.device[slave_num].ack == 0x79)
    {
        slave.device[slave_num].ack = 0;  // Сбрасываем флаг
        return true;
    }
    
    return false;
}

/**
 * @brief Переход к следующему блоку данных
 * @param ctx: указатель на контекст обновления
 */
void Slave_NextDataBlock(SlaveUpdateContext_t *ctx)
{
    uint32_t bytes_remaining = ctx->total_bytes - ctx->bytes_written;
    
    if (bytes_remaining > 0)
    {
        // Читаем следующий блок из флеша master (где хранится FW slave)
        // В данном примере используем фиксированный размер блока
        ctx->write_buffer_size = (bytes_remaining > SLAVE_FW_BLOCK_SIZE) 
                                  ? SLAVE_FW_BLOCK_SIZE 
                                  : bytes_remaining;
        
        // Копируем данные из памяти flash мастера
        // Здесь предполагается наличие памяти с прошивкой slave
        // Пример: memcpy(ctx->write_buffer, (void *)(addr), ctx->write_buffer_size);
        
        ctx->phase = SLAVE_UPD_SEND_ADDRESS;
    }
    else
    {
        ctx->phase = SLAVE_UPD_DONE;
    }
}

/**
 * @brief Основная функция обработки обновления слейва (запускается в Task04)
 * Вызывается периодически из StartTask04
 */
void Slave_UpdateProcess(void)
{
    uint32_t current_tick = HAL_GetTick();
    
    for (uint8_t slave_num = 0; slave_num < UART_CHANNEL_COUNT; slave_num++)
    {
        SlaveUpdateContext_t *ctx = &slave_update_ctx[slave_num];
        
        // Проверяем, нужно ли начинать обновление
        if (slave.device[slave_num].need_update && slave.os_in_flash)
        {
            if (ctx->phase == SLAVE_UPD_IDLE)
            {
                // Инициализируем обновление
                memset(ctx, 0, sizeof(SlaveUpdateContext_t));
                ctx->slave_number = slave_num;
                ctx->current_address = FLASH_SLAVEOS_START_ADDR;
                ctx->total_bytes = SLAVE_OS_SIZE;
                ctx->phase = SLAVE_UPD_SEND_ENTER_BOOTLOADER;
                ctx->last_command_tick = current_tick;
            }
        }
        
        // Обработка таймаута
        if ((current_tick - ctx->last_command_tick) > SLAVE_UPDATE_TIMEOUT_MS &&
            ctx->phase != SLAVE_UPD_IDLE &&
            ctx->phase != SLAVE_UPD_DONE &&
            ctx->phase != SLAVE_UPD_ERROR)
        {
            if (ctx->retry_count < SLAVE_UPDATE_RETRY_MAX)
            {
                ctx->retry_count++;
                ctx->last_command_tick = current_tick;
                // Не меняем фазу - повторим команду
            }
            else
            {
                ctx->phase = SLAVE_UPD_ERROR;
            }
        }
        
        // Конечный автомат обновления
        switch (ctx->phase)
        {
            // ============================================================
            // 1) ВХОД В BOOTLOADER
            // Адрес 0x01, запись 0x06, адрес регистра 0x0000, значение 0x1234
            // ============================================================
            case SLAVE_UPD_SEND_ENTER_BOOTLOADER:
            {
                // Используем Modbus для отправки команды входа в bootloader
                uint8_t modbus_cmd[8];
                modbus_cmd[0] = 0x01;              // Slave ID
                modbus_cmd[1] = 0x06;              // Function code (Write Single Register)
                modbus_cmd[2] = 0x00;              // Register address (high byte)
                modbus_cmd[3] = 0x00;              // Register address (low byte)
                modbus_cmd[4] = 0x12;              // Value (high byte) = 0x1234
                modbus_cmd[5] = 0x34;              // Value (low byte)
                // Добавляем CRC16
                uint16_t crc = Modbus_CalculateCRC(modbus_cmd, 6);
                modbus_cmd[6] = crc & 0xFF;
                modbus_cmd[7] = (crc >> 8) & 0xFF;
                
                Slave_SendBootloaderCommand(slave_num, modbus_cmd, 8);
                ctx->phase = SLAVE_UPD_WAIT_ACK_BOOTLOADER;
                ctx->last_command_tick = current_tick;
                break;
            }
            
            case SLAVE_UPD_WAIT_ACK_BOOTLOADER:
            {
                if (Slave_CheckACK(slave_num))
                {
                    ctx->phase = SLAVE_UPD_SEND_SYNC;
                    ctx->retry_count = 0;
                }
                break;
            }
            
            // ============================================================
            // 2) СИНХРОНИЗАЦИЯ (BIT SYNCHRONIZATION) - 0x7F
            // ============================================================
            case SLAVE_UPD_SEND_SYNC:
            {
                uint8_t sync_cmd = 0x7F;
                Slave_SendBootloaderCommand(slave_num, &sync_cmd, 1);
                ctx->phase = SLAVE_UPD_WAIT_ACK_SYNC;
                ctx->last_command_tick = current_tick;
                break;
            }
            
            case SLAVE_UPD_WAIT_ACK_SYNC:
            {
                if (Slave_CheckACK(slave_num))
                {
                    ctx->phase = SLAVE_UPD_SEND_ERASE;
                    ctx->retry_count = 0;
                }
                break;
            }
            
            // ============================================================
            // 3) СТИРАНИЕ (ERASE) - 0x44 + подтверждение 0xBB
            // ============================================================
            case SLAVE_UPD_SEND_ERASE:
            {
                uint8_t erase_cmd = 0x44;
                Slave_SendBootloaderCommand(slave_num, &erase_cmd, 1);
                ctx->phase = SLAVE_UPD_WAIT_ACK_ERASE;
                ctx->last_command_tick = current_tick;
                break;
            }
            
            case SLAVE_UPD_WAIT_ACK_ERASE:
            {
                if (Slave_CheckACK(slave_num))
                {
                    ctx->phase = SLAVE_UPD_SEND_ERASE_CONFIRM;
                    ctx->retry_count = 0;
                }
                break;
            }
            
            case SLAVE_UPD_SEND_ERASE_CONFIRM:
            {
                uint8_t erase_confirm = 0xBB;
                Slave_SendBootloaderCommand(slave_num, &erase_confirm, 1);
                ctx->phase = SLAVE_UPD_WAIT_ACK_ERASE_CONF;
                ctx->last_command_tick = current_tick;
                break;
            }
            
            case SLAVE_UPD_WAIT_ACK_ERASE_CONF:
            {
                if (Slave_CheckACK(slave_num))
                {
                    ctx->phase = SLAVE_UPD_SEND_MASS_ERASE;
                    ctx->retry_count = 0;
                }
                break;
            }
            
            // ============================================================
            // 4) ПОЛНОЕ СТИРАНИЕ (MASS ERASE) - 0xFF 0xFF + подтверждение 0x00
            // ============================================================
            case SLAVE_UPD_SEND_MASS_ERASE:
            {
                uint8_t mass_erase_cmd[2] = {0xFF, 0xFF};
                Slave_SendBootloaderCommand(slave_num, mass_erase_cmd, 2);
                ctx->phase = SLAVE_UPD_WAIT_ACK_MASS_ERASE;
                ctx->last_command_tick = current_tick;
                break;
            }
            
            case SLAVE_UPD_WAIT_ACK_MASS_ERASE:
            {
                if (Slave_CheckACK(slave_num))
                {
                    ctx->phase = SLAVE_UPD_SEND_MASS_ERASE_CONF;
                    ctx->retry_count = 0;
                }
                break;
            }
            
            case SLAVE_UPD_SEND_MASS_ERASE_CONF:
            {
                uint8_t mass_erase_conf = 0x00;
                Slave_SendBootloaderCommand(slave_num, &mass_erase_conf, 1);
                ctx->phase = SLAVE_UPD_WAIT_ACK_MASS_ERASE_CONF;
                ctx->last_command_tick = current_tick;
                break;
            }
            
            case SLAVE_UPD_WAIT_ACK_MASS_ERASE_CONF:
            {
                if (Slave_CheckACK(slave_num))
                {
                    ctx->phase = SLAVE_UPD_SEND_WRITE_CMD;
                    ctx->retry_count = 0;
                }
                break;
            }
            
            // ============================================================
            // 5) КОМАНДА ЗАПИСИ - 0x31 + подтверждение 0xCE
            // ============================================================
            case SLAVE_UPD_SEND_WRITE_CMD:
            {
                uint8_t write_cmd = 0x31;
                Slave_SendBootloaderCommand(slave_num, &write_cmd, 1);
                ctx->phase = SLAVE_UPD_WAIT_ACK_WRITE_CMD;
                ctx->last_command_tick = current_tick;
                break;
            }
            
            case SLAVE_UPD_WAIT_ACK_WRITE_CMD:
            {
                if (Slave_CheckACK(slave_num))
                {
                    ctx->phase = SLAVE_UPD_SEND_WRITE_CONFIRM;
                    ctx->retry_count = 0;
                }
                break;
            }
            
            case SLAVE_UPD_SEND_WRITE_CONFIRM:
            {
                uint8_t write_confirm = 0xCE;
                Slave_SendBootloaderCommand(slave_num, &write_confirm, 1);
                ctx->phase = SLAVE_UPD_WAIT_ACK_WRITE_CONFIRM;
                ctx->last_command_tick = current_tick;
                break;
            }
            
            case SLAVE_UPD_WAIT_ACK_WRITE_CONFIRM:
            {
                if (Slave_CheckACK(slave_num))
                {
                    ctx->phase = SLAVE_UPD_SEND_ADDRESS;
                    ctx->retry_count = 0;
                    Slave_NextDataBlock(ctx);  // Подготавливаем первый блок данных
                }
                break;
            }
            
            // ============================================================
            // 6) ОТПРАВКА АДРЕСА - 0x08 0x00 0x00 0x00 + подтверждение 0x08
            // ============================================================
            case SLAVE_UPD_SEND_ADDRESS:
            {
                uint8_t addr_cmd[5];
                // Преобразуем адрес в big-endian (0x08000000 для начала)
                addr_cmd[0] = (ctx->current_address >> 24) & 0xFF;  // 0x08
                addr_cmd[1] = (ctx->current_address >> 16) & 0xFF;  // 0x00
                addr_cmd[2] = (ctx->current_address >> 8) & 0xFF;   // 0x00
                addr_cmd[3] = (ctx->current_address >> 0) & 0xFF;   // 0x00
                
                // Контрольная сумма (XOR всех байт адреса)
                addr_cmd[4] = addr_cmd[0] ^ addr_cmd[1] ^ addr_cmd[2] ^ addr_cmd[3];
                
                Slave_SendBootloaderCommand(slave_num, addr_cmd, 5);
                ctx->phase = SLAVE_UPD_WAIT_ACK_ADDRESS;
                ctx->last_command_tick = current_tick;
                break;
            }
            
            case SLAVE_UPD_WAIT_ACK_ADDRESS:
            {
                if (Slave_CheckACK(slave_num))
                {
                    ctx->phase = SLAVE_UPD_SEND_DATA_SIZE;
                    ctx->retry_count = 0;
                }
                break;
            }
            
            // ============================================================
            // 7) КОЛИЧЕСТВО БАЙТ - (N-1) + данные
            // ============================================================
            case SLAVE_UPD_SEND_DATA_SIZE:
            {
                uint8_t size_cmd[1];
                // Отправляем (количество_байт - 1)
                size_cmd[0] = (ctx->write_buffer_size - 1) & 0xFF;
                
                Slave_SendBootloaderCommand(slave_num, size_cmd, 1);
                ctx->phase = SLAVE_UPD_WAIT_ACK_DATA_SIZE;
                ctx->last_command_tick = current_tick;
                break;
            }
            
            case SLAVE_UPD_WAIT_ACK_DATA_SIZE:
            {
                if (Slave_CheckACK(slave_num))
                {
                    ctx->phase = SLAVE_UPD_SEND_DATA;
                    ctx->retry_count = 0;
                }
                break;
            }
            
            // ============================================================
            // 7 продолжение) ОТПРАВКА ДАННЫХ
            // ============================================================
            case SLAVE_UPD_SEND_DATA:
            {
                uint8_t data_packet[256 + 1];  // Данные + контрольная сумма
                
                // Копируем данные
                for (int i = 0; i < ctx->write_buffer_size; i++)
                {
                    data_packet[i] = ctx->write_buffer[i];
                }
                
                // Вычисляем контрольную сумму (XOR всех байт данных)
                uint8_t checksum = 0;
                for (int i = 0; i < ctx->write_buffer_size; i++)
                {
                    checksum ^= data_packet[i];
                }
                data_packet[ctx->write_buffer_size] = checksum;
                
                Slave_SendBootloaderCommand(slave_num, data_packet, ctx->write_buffer_size + 1);
                ctx->phase = SLAVE_UPD_WAIT_ACK_DATA;
                ctx->last_command_tick = current_tick;
                break;
            }
            
            case SLAVE_UPD_WAIT_ACK_DATA:
            {
                if (Slave_CheckACK(slave_num))
                {
                    ctx->bytes_written += ctx->write_buffer_size;
                    ctx->current_address += ctx->write_buffer_size;
                    ctx->phase = SLAVE_UPD_NEXT_BLOCK;
                    ctx->retry_count = 0;
                }
                break;
            }
            
            // ============================================================
            // 8) ПЕРЕХОД К СЛЕДУЮЩЕМУ БЛОКУ
            // ============================================================
            case SLAVE_UPD_NEXT_BLOCK:
            {
                if (ctx->bytes_written < ctx->total_bytes)
                {
                    // Подготавливаем следующий блок
                    Slave_NextDataBlock(ctx);
                    // ctx->phase устанавливается в SLAVE_UPD_SEND_ADDRESS в Slave_NextDataBlock
                }
                else
                {
                    ctx->phase = SLAVE_UPD_DONE;
                }
                break;
            }
            
            // ============================================================
            // ЗАВЕРШЕНИЕ ОБНОВЛЕНИЯ
            // ============================================================
            case SLAVE_UPD_DONE:
            {
                // Обновление завершено успешно
                slave.device[slave_num].need_update = 0;
                ctx->phase = SLAVE_UPD_IDLE;
                break;
            }
            
            case SLAVE_UPD_ERROR:
            {
                // Ошибка при обновлении
                slave.device[slave_num].need_update = 0;
                ctx->phase = SLAVE_UPD_IDLE;
                break;
            }
            
            case SLAVE_UPD_IDLE:
            default:
                break;
        }
    }
}

