#include "bsp_ADNS9800.h"
#include "timer.h"

extern void MainClockTask(void);

/***************************************************************************************************
定义:   引脚配置.
***************************************************************************************************/
//片选脚.
#define A9800_CS_PIN_CLK                RCC_AHB1Periph_GPIOG
#define A9800_CS_PIN_PORT               GPIOG
#define A9800_CS_PIN_NO                 GPIO_Pin_15

#define A9800_2_CS_PIN_CLK              RCC_AHB1Periph_GPIOF
#define A9800_2_CS_PIN_PORT             GPIOF
#define A9800_2_CS_PIN_NO               GPIO_Pin_9

#define A9800_3_CS_PIN_CLK              RCC_AHB1Periph_GPIOF
#define A9800_3_CS_PIN_PORT             GPIOF
#define A9800_3_CS_PIN_NO               GPIO_Pin_10
/***************************************************************************************************
定义:   SPI1的配置.
***************************************************************************************************/
//SPI核心.注意:SPI1在APB2上,SPI2在APB1上.
#define SPI1_CLK                RCC_APB2Periph_SPI1     //SPI1是APB2高速外设.
//SCK引脚.
#define SPI1_SCK_PIN_CLK        RCC_AHB1Periph_GPIOA
#define SPI1_SCK_PIN_PORT       GPIOA
#define SPI1_SCK_PIN_NO         GPIO_Pin_5
//MISO引脚.
#define SPI1_MISO_PIN_CLK       RCC_AHB1Periph_GPIOA
#define SPI1_MISO_PIN_PORT      GPIOA
#define SPI1_MISO_PIN_NO        GPIO_Pin_6
//MOSI引脚.
#define SPI1_MOSI_PIN_CLK       RCC_AHB1Periph_GPIOA
#define SPI1_MOSI_PIN_PORT      GPIOA
#define SPI1_MOSI_PIN_NO        GPIO_Pin_7

#define SPI2_CLK                RCC_APB1Periph_SPI2     //SPI1是APB2高速外设.
//SCK引脚.
#define SPI2_SCK_PIN_CLK        RCC_AHB1Periph_GPIOB
#define SPI2_SCK_PIN_PORT       GPIOB
#define SPI2_SCK_PIN_NO         GPIO_Pin_13
//MISO引脚.
#define SPI2_MISO_PIN_CLK       RCC_AHB1Periph_GPIOB
#define SPI2_MISO_PIN_PORT      GPIOB
#define SPI2_MISO_PIN_NO        GPIO_Pin_14
//MOSI引脚.
#define SPI2_MOSI_PIN_CLK       RCC_AHB1Periph_GPIOB
#define SPI2_MOSI_PIN_PORT      GPIOB
#define SPI2_MOSI_PIN_NO        GPIO_Pin_15


/***************************************************************************************************
定义:   全局变量.
***************************************************************************************************/
ADNS9800_VAR_T g_tADNS9800;
uint8_t   g_uA9800Dummy = 0;                  //哑元.用于读寄存查看.



/***************************************************************************************************
函数:   SPI1读/写.
输入:   uWriteData      写入数据.
返回:   读出数据.
说明:   SPI接口在写入一个数据时,总能读出一个数据;读出一个数据时,必须写入一个数据.
***************************************************************************************************/
static uint8_t SpiXReadWrite(uint8_t uWriteData,uint8_t chip)
{
	if(chip == ADNS9800_CHIP1)
	{
		//等待发送寄存器空.
        while ((SPI1->SR & SPI_I2S_FLAG_TXE) == RESET);
        //发送一个字节.
        SPI1->DR = uWriteData;
        //等待接收寄存器有效.
        while ((SPI1->SR & SPI_I2S_FLAG_RXNE) == RESET);
        //接收一个字节.
        return (SPI1->DR);
	}
    else if(chip == ADNS9800_CHIP2)
    {
        //等待发送寄存器空.
        while ((SPI2->SR & SPI_I2S_FLAG_TXE) == RESET);
        //发送一个字节.
        SPI2->DR = uWriteData;
        //等待接收寄存器有效.
        while ((SPI2->SR & SPI_I2S_FLAG_RXNE) == RESET);
        //接收一个字节.
        return (SPI2->DR);
    }
    else
		{
			return 0;
		}
}

/***************************************************************************************************
函数:   SPI1读.
输入:   无.
返回:   读出数据.
说明:   外部调用本函数前必须先保证片选有效.
***************************************************************************************************/
uint8_t SpiXRead(uint8_t chip)
{
    return SpiXReadWrite(0X00,chip);
}

/***************************************************************************************************
函数:   SPI1写.
输入:   uWriteData.
返回:   无.
说明:   外部调用本函数前必须先保证片选有效.
***************************************************************************************************/
void SpiXWrite(uint8_t uWriteData,uint8_t chip)
{
    SpiXReadWrite(uWriteData,chip);
}

/***************************************************************************************************
函数:   SPI1初始化.
输入:   无.
返回:   无.
说明:   
***************************************************************************************************/
void SpiXSInit(void)
{
    SPI_InitTypeDef     SPI_InitStructure;
    GPIO_InitTypeDef    GPIO_InitStructure;

    //SPI1 引脚配置
    {
        //时钟配置.引脚时钟,SPI模块时钟.
        //SPI1在APB2上
        RCC_AHB1PeriphClockCmd(SPI1_SCK_PIN_CLK, ENABLE);//使能PA时钟
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);//使能SPI1 

        //配置引脚.
        //SCK引脚50M推挽输出.
        GPIO_ResetBits(SPI1_SCK_PIN_PORT, SPI1_SCK_PIN_NO);         //预置为空闲电平.
        GPIO_InitStructure.GPIO_Pin = SPI1_SCK_PIN_NO;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_Init(SPI1_SCK_PIN_PORT, &GPIO_InitStructure);
        //MOSI引脚50M推挽输出.
        GPIO_ResetBits(SPI1_MOSI_PIN_PORT, SPI1_MOSI_PIN_NO);       //预置为空闲电平.
        GPIO_InitStructure.GPIO_Pin = SPI1_MOSI_PIN_NO;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_Init(SPI1_MOSI_PIN_PORT, &GPIO_InitStructure);
        //MISO引脚上拉输入.
        GPIO_InitStructure.GPIO_Pin = SPI1_MISO_PIN_NO;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
        GPIO_Init(SPI1_MISO_PIN_PORT, &GPIO_InitStructure);

        GPIO_PinAFConfig(GPIOA,GPIO_PinSource5,GPIO_AF_SPI1); 
        GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_SPI1); 
        GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_SPI1);  

        RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1,ENABLE);
        RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1,DISABLE);
    }

    //SPI1模块配置.
    {
        SPI_Cmd(SPI1, DISABLE);                                                 //必须先禁能,才能改变MODE.

        SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;      //两线全双工.
        SPI_InitStructure.SPI_Mode = SPI_Mode_Master;                           //主.
        SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;                       //8位.
        SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;                             //时间空闲时为低.
        SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;                            //数据在第2个时钟沿捕获.
        SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;                               //软件NSS.
        SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;     // 64分频=1.3125M，APB2=84MHZ
        SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;                      //高位在前.
        SPI_InitStructure.SPI_CRCPolynomial = 7;                                //CRC7.
        
        SPI_Init(SPI1, &SPI_InitStructure);

        //使能SPI1.
        SPI_Cmd(SPI1, ENABLE); 
    }

    //SPI2 引脚配置
    {
        //时钟配置.引脚时钟,SPI模块时钟.
        //SPI1在APB2上
        RCC_AHB1PeriphClockCmd(SPI2_SCK_PIN_CLK, ENABLE);//使能PB时钟
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);//使能SPI2 

        //配置引脚.
        //SCK引脚50M推挽输出.
        GPIO_ResetBits(SPI2_SCK_PIN_PORT, SPI2_SCK_PIN_NO);         //预置为空闲电平.
        GPIO_InitStructure.GPIO_Pin = SPI2_SCK_PIN_NO;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_Init(SPI2_SCK_PIN_PORT, &GPIO_InitStructure);
        //MOSI引脚50M推挽输出.
        GPIO_ResetBits(SPI2_MOSI_PIN_PORT, SPI2_MOSI_PIN_NO);       //预置为空闲电平.
        GPIO_InitStructure.GPIO_Pin = SPI2_MOSI_PIN_NO;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_Init(SPI2_MOSI_PIN_PORT, &GPIO_InitStructure);
        //MISO引脚上拉输入.
        GPIO_InitStructure.GPIO_Pin = SPI2_MISO_PIN_NO;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
        GPIO_Init(SPI2_MISO_PIN_PORT, &GPIO_InitStructure);

        GPIO_PinAFConfig(GPIOB,GPIO_PinSource13,GPIO_AF_SPI2); 
        GPIO_PinAFConfig(GPIOB,GPIO_PinSource14,GPIO_AF_SPI2); 
        GPIO_PinAFConfig(GPIOB,GPIO_PinSource15,GPIO_AF_SPI2);  

        RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI2,ENABLE);
        RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI2,DISABLE);
    }

    //SPI2模块配置.
    {
        SPI_Cmd(SPI2, DISABLE);                                                 //必须先禁能,才能改变MODE.

        SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;      //两线全双工.
        SPI_InitStructure.SPI_Mode = SPI_Mode_Master;                           //主.
        SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;                       //8位.
        SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;                             //时间空闲时为低.
        SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;                            //数据在第2个时钟沿捕获.
        SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;                               //软件NSS.
        SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;     // 32分频=1.3125M，APB1=42MHZ
        SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;                      //高位在前.
        SPI_InitStructure.SPI_CRCPolynomial = 7;                                //CRC7.
        
        SPI_Init(SPI2, &SPI_InitStructure);

        //使能SPI1.
        SPI_Cmd(SPI2, ENABLE); 
    }
}

/***************************************************************************************************
函数:   延时us.
输入:   uNum        微秒数.
返回:   无.
***************************************************************************************************/
void A9800DelayUs(uint16_t uNum)
{
    /*
    uint32_t     uCounter;
    while (uNum--)
    {
        uCounter = SYS_CLK_FREQ / 1000000 / 3;
        while (uCounter--);
    }
    */
   TIMDelay_Nus(uNum);
}

/***************************************************************************************************
函数:   延时ms.
输入:   uNum        毫秒数.
返回:   无.
***************************************************************************************************/
void A9800DelayMs(uint16_t uNum)
{
    /*
    uint32_t     uCounter;

    while (uNum--)
    {
        uCounter = SYS_CLK_FREQ / 1000 / 3;
        while (uCounter--);
    }
    */
   TIMDelay_Nms(uNum);
}

/***************************************************************************************************
函数:   拉低CS.
输入:   无.
返回:   无.
***************************************************************************************************/
void A9800xSetCsLow(uint8_t chip)
{
    if(chip==ADNS9800_CHIP1)
    {
        GPIO_ResetBits(A9800_CS_PIN_PORT, A9800_CS_PIN_NO);
    }
    else if(chip==ADNS9800_CHIP2)
    {
        GPIO_ResetBits(A9800_2_CS_PIN_PORT, A9800_2_CS_PIN_NO);
    }
}
/***************************************************************************************************
函数:   拉高CS.
输入:   无.
返回:   无.
***************************************************************************************************/
void A9800xSetCsHigh(uint8_t chip)
{
    if(chip==ADNS9800_CHIP1)
    {
        GPIO_SetBits(A9800_CS_PIN_PORT, A9800_CS_PIN_NO);
    }
    else if(chip==ADNS9800_CHIP2)
    {
        GPIO_SetBits(A9800_2_CS_PIN_PORT, A9800_2_CS_PIN_NO);
    }
}

/***************************************************************************************************
函数:   读寄存器.
输入:   uAddr       寄存器地址.
返回:   值.
***************************************************************************************************/
uint8_t A9800xReadReg(uint8_t uAddr,uint8_t chip)
{
	uint8_t   uVal;

	A9800xSetCsLow(chip);

	A9800DelayUs(3);
	SpiXWrite(uAddr,chip);
	A9800DelayUs(100);//VENTUS: tSRAD=100us. 
	//在保证了tSRAD之后，不仅CPI每次都读取正确了，而且SROM_check基本都能成功。有时失败：OBSERVATION_FAILED.
	
	uVal = SpiXRead(chip);
	A9800DelayUs(12);

    A9800xSetCsHigh(chip);

    //VENTUS:
    A9800DelayUs(20);//tSRW=tSRR=20us

	return uVal;
}
/***************************************************************************************************
函数:   写寄存器.
输入:   uAddr       寄存器地址.
        uVal        寄存器值.
返回:   值.
***************************************************************************************************/
void A9800xWriteReg(uint8_t uAddr, uint8_t uVal,uint8_t chip)
{
	A9800xSetCsLow(chip);
    
	A9800DelayUs(3);
	SpiXWrite(uAddr | 0x80,chip);
	A9800DelayUs(5); 
	SpiXWrite(uVal,chip);
	A9800DelayUs(3); 
    
	A9800xSetCsHigh(chip);
    
    //VENTUS:
    A9800DelayUs(120);//tSWR=tSWW=120us
		//在保证了tSWR,tSWW,tSRR,tSRW以及tSRAD之后，每一次SROM check均可通过，并且CPI均可读取正确.
		//A9800DelayMs(10);
}

/***************************************************************************************************
函数:   写入CPI.
输入:   uCpi            CPI编码.1-164.每1代表50CPI.寄存器默认68(3400CPI).
返回:   无.
***************************************************************************************************/
void A9800xWriteCpi(uint8_t uCpi,uint8_t chip)
{
    A9800xWriteReg(A9800_CONFIGURATION_I_ADDR, uCpi,chip);
}

/***************************************************************************************************
函数:   写入抬起高度门限.
输入:   uLift           抬起高度门限.1-31.寄存器默认16.
返回:   无.
***************************************************************************************************/
void A9800xWriteLift(uint8_t uLift,uint8_t chip)
{
    A9800xWriteReg(A9800_LIFT_DETECTION_THR_ADDR, uLift,chip);
}

/***************************************************************************************************
函数:   设置CPI.
输入:   uCpi            CPI编码.1-164.每1代表50CPI.寄存器默认68(3400CPI).
返回:   无.
***************************************************************************************************/
void A9800xSSetCpi(uint8_t uCpi)
{
    if (uCpi < A9800_CPI_MIN || uCpi > A9800_CPI_MAX)
    {
        return;

    }
    if (uCpi == g_tADNS9800.g_uA9800Cpi)
    {
        return;
    }

    //DebugLog("设置CPI "); DebugLogAddVal(uCpi, 3); DebugLogAddStr("\r\n");
    g_tADNS9800.g_uA9800Cpi = uCpi;
    A9800xWriteCpi(uCpi,ADNS9800_CHIP1);
    A9800xWriteCpi(uCpi,ADNS9800_CHIP2);
}

/***************************************************************************************************
函数:   设置抬起高度门限.
输入:   uLift           抬起高度门限.1-31.寄存器默认15.
返回:   无.
***************************************************************************************************/
void A9800xSSetLift(uint8_t uLift)
{
    if (uLift < A9800_LIFT_MIN || uLift > A9800_LIFT_MAX)
    {
        return;
    }
    if (uLift == g_tADNS9800.g_uA9800Lift)
    {
        return;
    }

    //DebugLog("设置LIFT "); DebugLogAddVal(uLift, 3); DebugLogAddStr("\r\n");
    g_tADNS9800.g_uA9800Lift = uLift;
    A9800xWriteLift(uLift,ADNS9800_CHIP1);
    A9800xWriteLift(uLift,ADNS9800_CHIP2);
}

/***************************************************************************************************
函数:   突发读移动寄存器.
输入:   puBuf       缓冲区.读出数据由它带回.
        uNum        请求读出数量.
返回:   无.
***************************************************************************************************/
void A9800xBurstReadMoveReg(uint8_t * puBuf, uint8_t uNum, uint8_t chip)
{
    uint8_t   i;

	//__disable_irq();
	
    A9800xSetCsLow(chip);   
    A9800DelayUs(3);

    SpiXWrite(A9800_MOTION_BURST_ADDR,chip);
    A9800DelayUs(100);	// 5);

    for (i = 0; i < uNum; i++)
    {
        puBuf[i] = SpiXRead(chip);
    }
    
    A9800xSetCsHigh(chip);

	//__enable_irq();
}

/***************************************************************************************************
函数:   移动检测工作.
输入:   无.
返回:   无.
说明:   本函数每1ms执行一次.
***************************************************************************************************/
void A9800MoveDetectWork(uint8_t chip)
{
    uint8_t           auBuf[6];
    int16_t          iMoveX;
    int16_t          iMoveY;
    
    A9800xBurstReadMoveReg(auBuf, 6, chip);

    iMoveX = auBuf[3];
    iMoveX <<= 8;
    iMoveX |= auBuf[2];

    iMoveY = auBuf[5];
    iMoveY <<= 8;
    iMoveY |= auBuf[4];

    if(chip == ADNS9800_CHIP1)
    {
        g_tADNS9800.MotionX_1 = iMoveX;
        g_tADNS9800.MotionY_1 = iMoveY;
    }
    else if(chip == ADNS9800_CHIP2)
    {
        g_tADNS9800.MotionX_2 = iMoveX;
        g_tADNS9800.MotionY_2 = iMoveY;
    }

    if (iMoveX == 0 && iMoveY == 0)
    {
        return;
    }

    if(chip == ADNS9800_CHIP1)
    {
        //DebugLog("X:"); DebugLogAddVal(iMoveX, 6 | VAL_FORMAT_SIGN); DebugLogAddStr(", ");
        //DebugLogAddStr("Y:"); DebugLogAddVal(iMoveY, 6 | VAL_FORMAT_SIGN); DebugLogAddStr("\r\n");
        
        g_tADNS9800.PosX_1 += iMoveX;
        g_tADNS9800.PosY_1 += iMoveY;
        
        if (iMoveX >= 0)
        {
            g_tADNS9800.TotalMotionX += iMoveX;
        }
        else
        {
            g_tADNS9800.TotalMotionX += 0 - iMoveX;
        }
        
        if (iMoveY >= 0)
        {
            g_tADNS9800.TotalMotionY += iMoveY;
        }
        else
        {
            g_tADNS9800.TotalMotionY += 0 - iMoveY;
        } 
    }
    else if(chip == ADNS9800_CHIP2)
    {
        //DebugLog("X:"); DebugLogAddVal(iMoveX, 6 | VAL_FORMAT_SIGN); DebugLogAddStr(", ");
        //DebugLogAddStr("Y:"); DebugLogAddVal(iMoveY, 6 | VAL_FORMAT_SIGN); DebugLogAddStr("\r\n");
        
        g_tADNS9800.PosX_2 += iMoveX;
        g_tADNS9800.PosY_2 += iMoveY;
        
        if (iMoveX >= 0)
        {
            g_tADNS9800.TotalMotionX_2 += iMoveX;
        }
        else
        {
            g_tADNS9800.TotalMotionX_2 += 0 - iMoveX;
        }
        
        if (iMoveY >= 0)
        {
            g_tADNS9800.TotalMotionY_2 += iMoveY;
        }
        else
        {
            g_tADNS9800.TotalMotionY_2 += 0 - iMoveY;
        } 
    }

}
/***************************************************************************************************
函数:   定时1ms工作.
输入:   无.
返回:   无.
***************************************************************************************************/
void A9800OnTimerTask(void)
{
    A9800MoveDetectWork(ADNS9800_CHIP1);
    A9800MoveDetectWork(ADNS9800_CHIP2);
}

/***************************************************************************************************
函数:   检查ID.
输入:   无.
返回:   无.
***************************************************************************************************/
void A9800xInitCheckId(uint8_t chip)
{
    g_uA9800Dummy = A9800xReadReg(A9800_PRODUCTID_ADDR,chip);            
    g_uA9800Dummy = A9800xReadReg(A9800_REVISIONID_ADDR,chip);
    g_uA9800Dummy = A9800xReadReg(A9800_INVERSE_PRODUCT_ID_ADDR,chip);
}
/***************************************************************************************************
函数:   上电.
输入:   无.
返回:   无.
***************************************************************************************************/
void A9800xInitPowerUp(uint8_t chip)
{   //上电.
    A9800xWriteReg(A9800_POWER_UP_RESET_ADDR, A9800_POWER_UP_RESET, chip);
    A9800DelayMs(50);                   //延时50ms.

    g_uA9800Dummy = A9800xReadReg(A9800_MOTION_ADDR, chip);
    g_uA9800Dummy = A9800xReadReg(A9800_DELTAX_L_ADDR, chip);
    g_uA9800Dummy = A9800xReadReg(A9800_DELTAX_H_ADDR, chip);
    g_uA9800Dummy = A9800xReadReg(A9800_DELTAY_L_ADDR, chip);
    g_uA9800Dummy = A9800xReadReg(A9800_DELTAY_H_ADDR, chip);
}

/***************************************************************************************************
函数:   加载SROM.
输入:   无.
返回:   无.
***************************************************************************************************/
#define SROM_SIZE           (sizeof(c_auA9800SromData))
void A9800xLoadSrom(uint8_t chip)
{
    uint16_t  uSize;
    uint16_t  i;

    A9800xWriteReg(A9800_CONFIGURATION_IV_ADDR, A9800_SROM_SIZE_3_0K, chip);     // Select 3k SROM size
    A9800DelayMs(1);

    A9800xWriteReg(A9800_SROM_ENABLE_ADDR, 0x1D, chip); 
    A9800DelayMs(10);                                                                // Wait 1 frame period

    A9800xWriteReg(A9800_SROM_ENABLE_ADDR, 0x18, chip);
    A9800DelayUs(120);

    A9800xSetCsLow(chip); 
    
    A9800DelayUs(10);

    SpiXWrite(A9800_SROM_LOAD_BURST_ADDR | 0x80, chip);
    A9800DelayMs(10);

    uSize = sizeof(c_auA9800SromData);
    char tmp[50];
    sprintf(tmp,"Size of SROM: %d",uSize);
    printx(tmp);
    printx("Uploading ADNS9800 firmware...");
    for (i = 0; i < uSize; i++)
    {    
        SpiXWrite(c_auA9800SromData[i], chip);
        A9800DelayUs(100);
    }
    printx("ADNS9800 firmware upload done.");
    A9800xSetCsHigh(chip);
}
/***************************************************************************************************
函数:   检查SROM.
输入:   无.
返回:   操作结果.
***************************************************************************************************/
OP A9800xCheckSrom(uint8_t chip)
{
    uint8_t auBuf[10];

    A9800DelayUs(200);

    auBuf[0] = A9800xReadReg(A9800_SROM_ID_ADDR, chip);                    // Check SROM version
    A9800DelayUs(10);

    auBuf[1] = A9800xReadReg(A9800_MOTION_ADDR, chip);                     // Check Fault
    A9800DelayUs(10);
    if ((auBuf[1] & A9800_MOTION_FAULT) == A9800_MOTION_FAULT)
    {
		printx("FAULT");
        return OP_FAIL;
    }
    
    A9800xWriteReg(A9800_SROM_ENABLE_ADDR, 0x15, chip);                    // Check CRC Test
    A9800DelayMs(50);

    auBuf[2] = A9800xReadReg(A9800_DATA_OUT_LOWER_ADDR, chip);            
    A9800DelayUs(10);
    auBuf[3] = A9800xReadReg(A9800_DATA_OUT_UPPER_ADDR, chip);            
    A9800DelayUs(10);
    if ((auBuf[2] != 0xEF) || (auBuf[3] != 0xBE))
    {
		printx("DATA_OUT_LOWER/UPPER_ADDR");
        return OP_FAIL;
    }

    A9800xWriteReg(A9800_OBSERVATION_ADDR, 0x00, chip);                    // Clear Observation Register
    A9800DelayUs(1);
    auBuf[4] = A9800xReadReg(A9800_OBSERVATION_ADDR, chip);                // Check if SROM is running
    if ((auBuf[4] & 0x40) == 0)
    {
		printx("OBSERVATION_ADDR");
        return OP_FAIL;
    }

    return OP_SUCCESS;
}
/***************************************************************************************************
函数:   使能激光电源.
输入:   无.
返回:   无.
***************************************************************************************************/
void A9800xEnLaserPower(uint8_t chip)
{
    uint8_t uData;    

    uData = A9800xReadReg(A9800_LASER_CTRL0_ADDR, chip);
    A9800xWriteReg(A9800_LASER_CTRL0_ADDR, (uData & 0xFE), chip);                // Enable Laser Power
}
/***************************************************************************************************
函数:   初始化SROM，并使能激光
输入:   无.
返回:   无.
***************************************************************************************************/
void A9800xInitSrom(uint8_t chip)
{
    A9800xLoadSrom(chip);
    A9800DelayMs(1);
    
    g_uA9800Dummy = A9800xReadReg(0x2A, chip);
    OP result = A9800xCheckSrom(chip);
    A9800xEnLaserPower(chip);
	if(result == OP_SUCCESS)
    {
        printx("SROM check success.");
    }
    else if(result == OP_FAIL)
    {
        printx("SROM check failed.");
    }
}

void A9800xdispRegisters(uint8_t chip){

    A9800xSetCsLow(chip);
    uint16_t oreg[7] = {
    0x00,0x3F,0x2A,0x02  };
    uint8_t * oregname[] = {
    "Product_ID","Inverse_Product_ID","SROM_Version","Motion"  };
    uint8_t regres;

    uint8_t rctr=0;
    char tmp[50];
    sprintf(tmp,"-----------ANDS9800 - %d Info----------",chip + 1);
    printx(tmp);
    for(rctr=0; rctr<4; rctr++){
		A9800DelayUs(20);//read和下一步命令之间间隔20us
        regres = A9800xReadReg(oreg[rctr], chip);
        sprintf(tmp,"%s:%x",oregname[rctr],regres);
        printx(tmp);  
    }
    A9800xSetCsHigh(chip);
}

/***************************************************************************************************
函数:   初始化.
输入:   bIap    IAP状态.
返回:   无.
***************************************************************************************************/
void A9800xInit(uint8_t bIap, uint8_t chip)
{
    SpiXSInit();

    GPIO_InitTypeDef    GPIO_InitStructure;
    uint8_t               auBuf[10];
    if(chip == ADNS9800_CHIP1)
    {
        printx("Initiallizing ADNS9800-1...\n----------------------------------");
        //片选脚.
        RCC_AHB1PeriphClockCmd(A9800_CS_PIN_CLK, ENABLE);
        GPIO_SetBits(A9800_CS_PIN_PORT, A9800_CS_PIN_NO);
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_Pin = A9800_CS_PIN_NO;
        GPIO_Init(A9800_CS_PIN_PORT, &GPIO_InitStructure); 
    }
    else if(chip == ADNS9800_CHIP2)
    {
        printx("\nInitiallizing ADNS9800-2...\n----------------------------------");
        //片选脚.
        RCC_AHB1PeriphClockCmd(A9800_2_CS_PIN_CLK, ENABLE);
        GPIO_SetBits(A9800_2_CS_PIN_PORT, A9800_2_CS_PIN_NO);
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_Pin = A9800_2_CS_PIN_NO;
        GPIO_Init(A9800_2_CS_PIN_PORT, &GPIO_InitStructure); 
    }


    if (bIap == 1)          //IAP状态下,不继续初始化,不开启A9800.
    {
        return;
    }
    
    A9800xInitCheckId(chip);
    A9800xInitPowerUp(chip);
    A9800xInitSrom(chip);
    A9800xdispRegisters(chip);

    //A9800xWriteCpi(g_uA9800Cpi, chip);
		A9800xWriteCpi((uint8_t)(_cfg_ADNS9800_CPI/50), chip);
		A9800DelayMs(1);//VENTUS;亲测在写入CPI和再次读取之间需要一定间隔。1ms足以保证再次读取到的CPI是正确的
    char tmp[50];
    uint8_t CPI_Byte = A9800xReadReg(A9800_CONFIGURATION_I_ADDR, chip);
    sprintf(tmp,"ANDS9800 - %d CPI set to:%d.",chip + 1,((uint16_t)CPI_Byte)*50);
    printx(tmp);
    
    A9800xWriteLift(g_tADNS9800.g_uA9800Lift, chip);
    
    auBuf[0] = A9800xReadReg(A9800_MOTION_ADDR, chip);
    auBuf[1] = A9800xReadReg(A9800_LASER_CTRL0_ADDR, chip);
    auBuf[2] = A9800xReadReg(A9800_LASER_CTRL1_ADDR, chip);
    auBuf[3] = A9800xReadReg(A9800_LP_CFG0_ADDR, chip);
    auBuf[4] = A9800xReadReg(A9800_LP_CFG1_ADDR, chip);
    if (auBuf[0] == 0)
    {
        ;
    }
}

void A9800XSInit(uint8_t bIap)
{
    g_tADNS9800.g_uA9800Cpi = A9800_RES6350CPI;     //CPI值.
    g_tADNS9800.g_uA9800Lift = 16;                  //抬起高度门限.
    g_tADNS9800.MotionX_1 = 0;
    g_tADNS9800.MotionY_1 = 0;
    g_tADNS9800.MotionX_2 = 0;
    g_tADNS9800.MotionY_2 = 0;
    g_tADNS9800.PosX_1 = 0;                //X轴位移.
    g_tADNS9800.PosY_1 = 0;                //Y轴位移.
    g_tADNS9800.PosX_2 = 0;              //X轴位移.第2块鼠标芯片
    g_tADNS9800.PosY_2 = 0;              //Y轴位移.第2块鼠标芯片
    g_tADNS9800.TotalMotionX = 0;						//x轴绝对值位移，鼠标芯片-1
    g_tADNS9800.TotalMotionY = 0;						//y轴绝对值位移，鼠标芯片-1
    g_tADNS9800.TotalMotionX_2 = 0;					//x轴绝对值位移，鼠标芯片-2
    g_tADNS9800.TotalMotionY_2 = 0;					//y轴绝对值位移，鼠标芯片-2

    A9800xInit(bIap,ADNS9800_CHIP1);
    A9800xInit(bIap,ADNS9800_CHIP2);
}

void bsp_ADNS9800_Initializes(void)
{
    printx("Initializing ADNS9800s...");
    A9800XSInit(0);
}

/**** Copyright (C)2018 Ventus Guo. All Rights Reserved **** END OF FILE ****/

