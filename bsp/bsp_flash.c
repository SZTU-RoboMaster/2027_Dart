#include "bsp_flash.h"
#include "main.h"
#include "string.h"

/* 内部地址换算声明；实现与公开兼容函数使用同一扇区表。 */
static uint32_t get_sector(uint32_t address);

/**
  * @说明 从指定地址所属扇区开始，擦除连续的若干扇区。
  * @参数 address 任意位于首个目标扇区内的地址。
  * @参数 len 连续擦除的扇区数量，不是字节数。
  * @返回值 无；底层擦除错误由后续参数校验发现。
  */
void flash_erase_address(uint32_t address, uint16_t len)
{
    FLASH_EraseInitTypeDef flash_erase;
    uint32_t error;

    flash_erase.Sector = ger_sector(address);
    flash_erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    flash_erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    flash_erase.NbSectors = len;

    HAL_FLASH_Unlock();
    HAL_FLASHEx_Erase(&flash_erase, &error);
    HAL_FLASH_Lock();
}

/**
  * @说明 在起始地址所属扇区内顺序写入三十二位数据。
  * @参数 start_address 首个写入地址，必须按四字节对齐。
  * @参数 buf 待写入数据首地址。
  * @参数 len 待写入的三十二位字数量。
  * @返回值 成功返回零，任一字写入失败返回负一。
  */
int8_t flash_write_single_address(uint32_t start_address, uint32_t *buf, uint32_t len)
{
    static uint32_t uw_address;
    static uint32_t end_address;
    static uint32_t *data_buf;
    static uint32_t data_len;

    HAL_FLASH_Unlock();

    uw_address = start_address;
    end_address = get_next_flash_address(start_address);
    data_buf = buf;
    data_len = 0;

    while (uw_address <= end_address)
    {

        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,uw_address, *data_buf) == HAL_OK)
        {
            uw_address += 4;
            data_buf++;
            data_len++;
            if (data_len == len)
            {
                break;
            }
        }
        else
        {
            HAL_FLASH_Lock();
            return -1;
        }
    }

    HAL_FLASH_Lock();
    return 0;
}

/**
  * @说明 在给定闭区间内顺序写入三十二位数据，可跨越多个扇区。
  * @参数 start_address 首个写入地址。
  * @参数 end_address 允许写入的最后地址。
  * @参数 buf 待写入数据首地址。
  * @参数 len 待写入的三十二位字数量。
  * @返回值 成功返回零，任一字写入失败返回负一。
  */
int8_t flash_write_muli_address(uint32_t start_address, uint32_t end_address, uint32_t *buf, uint32_t len)
{
    uint32_t uw_address = 0;
    uint32_t *data_buf;
    uint32_t data_len;

    HAL_FLASH_Unlock();

    uw_address = start_address;
    data_buf = buf;
    data_len = 0;
    while (uw_address <= end_address)
    {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,uw_address, *data_buf) == HAL_OK)
        {
            uw_address += 4;
            data_buf++;
            data_len++;
            if (data_len == len)
            {
                break;
            }
        }
        else
        {
            HAL_FLASH_Lock();
            return -1;
        }
    }

    HAL_FLASH_Lock(); 
    return 0;
}

/**
  * @说明 利用芯片的存储器映射读取连续三十二位数据。
  * @参数 address 首个读取地址。
  * @参数 buf 接收数据的目标缓冲区。
  * @参数 len 读取的三十二位字数量。
  * @返回值 无。
  */
void flash_read(uint32_t address, uint32_t *buf, uint32_t len)
{
    memcpy(buf, (void*)address, len *4);
}


/**
  * @说明 把第一存储体地址换算成硬件抽象层要求的扇区编号。
  * @参数 address 待查询地址。
  * @返回值 地址所在扇区的硬件编号；越界时返回第十一扇区作为安全兼容值。
  */
uint32_t ger_sector(uint32_t address)
{
    uint32_t sector = 0;
    if ((address < ADDR_FLASH_SECTOR_1) && (address >= ADDR_FLASH_SECTOR_0))
    {
        sector = FLASH_SECTOR_0;
    }
    else if ((address < ADDR_FLASH_SECTOR_2) && (address >= ADDR_FLASH_SECTOR_1))
    {
        sector = FLASH_SECTOR_1;
    }
    else if ((address < ADDR_FLASH_SECTOR_3) && (address >= ADDR_FLASH_SECTOR_2))
    {
        sector = FLASH_SECTOR_2;
    }
    else if ((address < ADDR_FLASH_SECTOR_4) && (address >= ADDR_FLASH_SECTOR_3))
    {
        sector = FLASH_SECTOR_3;
    }
    else if ((address < ADDR_FLASH_SECTOR_5) && (address >= ADDR_FLASH_SECTOR_4))
    {
        sector = FLASH_SECTOR_4;
    }
    else if ((address < ADDR_FLASH_SECTOR_6) && (address >= ADDR_FLASH_SECTOR_5))
    {
        sector = FLASH_SECTOR_5;
    }
    else if ((address < ADDR_FLASH_SECTOR_7) && (address >= ADDR_FLASH_SECTOR_6))
    {
        sector = FLASH_SECTOR_6;
    }
    else if ((address < ADDR_FLASH_SECTOR_8) && (address >= ADDR_FLASH_SECTOR_7))
    {
        sector = FLASH_SECTOR_7;
    }
    else if ((address < ADDR_FLASH_SECTOR_9) && (address >= ADDR_FLASH_SECTOR_8))
    {
        sector = FLASH_SECTOR_8;
    }
    else if ((address < ADDR_FLASH_SECTOR_10) && (address >= ADDR_FLASH_SECTOR_9))
    {
        sector = FLASH_SECTOR_9;
    }
    else if ((address < ADDR_FLASH_SECTOR_11) && (address >= ADDR_FLASH_SECTOR_10))
    {
        sector = FLASH_SECTOR_10;
    }
    else if ((address < ADDR_FLASH_SECTOR_12) && (address >= ADDR_FLASH_SECTOR_11))
    {
        sector = FLASH_SECTOR_11;
    }
    else
    {
        sector = FLASH_SECTOR_11;
    }

    return sector;
}

/**
  * @说明 返回第一存储体当前扇区之后的第一个地址。
  * @参数 address 当前扇区内任意地址。
  * @返回值 下一扇区起始地址；最后一个受支持扇区之后返回存储器末地址。
  */
uint32_t get_next_flash_address(uint32_t address)
{
    uint32_t sector = 0;

    if ((address < ADDR_FLASH_SECTOR_1) && (address >= ADDR_FLASH_SECTOR_0))
    {
        sector = ADDR_FLASH_SECTOR_1;
    }
    else if ((address < ADDR_FLASH_SECTOR_2) && (address >= ADDR_FLASH_SECTOR_1))
    {
        sector = ADDR_FLASH_SECTOR_2;
    }
    else if ((address < ADDR_FLASH_SECTOR_3) && (address >= ADDR_FLASH_SECTOR_2))
    {
        sector = ADDR_FLASH_SECTOR_3;
    }
    else if ((address < ADDR_FLASH_SECTOR_4) && (address >= ADDR_FLASH_SECTOR_3))
    {
        sector = ADDR_FLASH_SECTOR_4;
    }
    else if ((address < ADDR_FLASH_SECTOR_5) && (address >= ADDR_FLASH_SECTOR_4))
    {
        sector = ADDR_FLASH_SECTOR_5;
    }
    else if ((address < ADDR_FLASH_SECTOR_6) && (address >= ADDR_FLASH_SECTOR_5))
    {
        sector = ADDR_FLASH_SECTOR_6;
    }
    else if ((address < ADDR_FLASH_SECTOR_7) && (address >= ADDR_FLASH_SECTOR_6))
    {
        sector = ADDR_FLASH_SECTOR_7;
    }
    else if ((address < ADDR_FLASH_SECTOR_8) && (address >= ADDR_FLASH_SECTOR_7))
    {
        sector = ADDR_FLASH_SECTOR_8;
    }
    else if ((address < ADDR_FLASH_SECTOR_9) && (address >= ADDR_FLASH_SECTOR_8))
    {
        sector = ADDR_FLASH_SECTOR_9;
    }
    else if ((address < ADDR_FLASH_SECTOR_10) && (address >= ADDR_FLASH_SECTOR_9))
    {
        sector = ADDR_FLASH_SECTOR_10;
    }
    else if ((address < ADDR_FLASH_SECTOR_11) && (address >= ADDR_FLASH_SECTOR_10))
    {
        sector = ADDR_FLASH_SECTOR_11;
    }
    else /*(address < FLASH_END_ADDR) && (address >= ADDR_FLASH_SECTOR_23))*/
    {
        sector = FLASH_END_ADDR;
    }
    return sector;
}

void Flash_Erase_Sector(void)
{
    FLASH_EraseInitTypeDef EraseInitStruct = {0};
    uint32_t SectorError = 0;

    uint32_t SectorNumber=FLASH_SECTOR_11;

    EraseInitStruct.TypeErase=FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.VoltageRange=FLASH_VOLTAGE_RANGE_3;
    EraseInitStruct.Sector=SectorNumber;
    EraseInitStruct.NbSectors=1;

    if (HAL_FLASHEx_Erase(&EraseInitStruct,&SectorError) !=HAL_OK)
    {

    }
}

void Flash_Write_Data(float *Data,uint32_t DataLength)
{
    uint32_t DataAddress=FLASH_USER_START_ADDR;
    float_to_uint32 converter[DataLength];
    uint32_t length=DataLength*4;
    for (int i=0;i<DataLength;i++)
    {
        converter[i].floatValue=Data[i];
    }
    HAL_FLASH_Unlock();
    Flash_Erase_Sector();
    for (uint32_t i=0;i<length;i+=4)
    {
        if ((DataAddress + i) >= (FLASH_END_ADDR-1))
            break;

        uint32_t word_data=converter[i/4].uintValue;

        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,DataAddress+i,word_data)!=HAL_OK)
        {

        }
    }
    HAL_FLASH_Lock();
}

void Flash_Read_Data(float *pData,uint32_t DataLength)
{
    uint32_t DataAddress=FLASH_USER_START_ADDR;
    float_to_uint32 converter[DataLength];
    uint32_t length=DataLength*4;
    HAL_FLASH_Unlock();
    for (uint32_t i=0;i<length; i+=4)
    {
        if (DataAddress + i>=FLASH_END_ADDR)
            break;
        converter[i/4].uintValue=*(__IO uint32_t*)(DataAddress+i);
    }
    for (int i=0;i<DataLength;i++)
        pData[i]=converter[i].floatValue;
    HAL_FLASH_Lock();
}
