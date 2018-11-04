#include "bsp_ADNS9800.h"
#include "timer.h"

extern void MainClockTask(void);

/***************************************************************************************************
����:   ��������.
***************************************************************************************************/
//Ƭѡ��.
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
����:   SPI1������.
***************************************************************************************************/
//SPI����.ע��:SPI1��APB2��,SPI2��APB1��.
#define SPI1_CLK                RCC_APB2Periph_SPI1     //SPI1��APB2��������.
//SCK����.
#define SPI1_SCK_PIN_CLK        RCC_AHB1Periph_GPIOA
#define SPI1_SCK_PIN_PORT       GPIOA
#define SPI1_SCK_PIN_NO         GPIO_Pin_5
//MISO����.
#define SPI1_MISO_PIN_CLK       RCC_AHB1Periph_GPIOA
#define SPI1_MISO_PIN_PORT      GPIOA
#define SPI1_MISO_PIN_NO        GPIO_Pin_6
//MOSI����.
#define SPI1_MOSI_PIN_CLK       RCC_AHB1Periph_GPIOA
#define SPI1_MOSI_PIN_PORT      GPIOA
#define SPI1_MOSI_PIN_NO        GPIO_Pin_7

#define SPI2_CLK                RCC_APB1Periph_SPI2     //SPI1��APB2��������.
//SCK����.
#define SPI2_SCK_PIN_CLK        RCC_AHB1Periph_GPIOB
#define SPI2_SCK_PIN_PORT       GPIOB
#define SPI2_SCK_PIN_NO         GPIO_Pin_13
//MISO����.
#define SPI2_MISO_PIN_CLK       RCC_AHB1Periph_GPIOB
#define SPI2_MISO_PIN_PORT      GPIOB
#define SPI2_MISO_PIN_NO        GPIO_Pin_14
//MOSI����.
#define SPI2_MOSI_PIN_CLK       RCC_AHB1Periph_GPIOB
#define SPI2_MOSI_PIN_PORT      GPIOB
#define SPI2_MOSI_PIN_NO        GPIO_Pin_15


/***************************************************************************************************
����:   ȫ�ֱ���.
***************************************************************************************************/
ADNS9800_VAR_T g_tADNS9800;
uint8_t   g_uA9800Dummy = 0;                  //��Ԫ.���ڶ��Ĵ�鿴.



/***************************************************************************************************
����:   SPI1��/д.
����:   uWriteData      д������.
����:   ��������.
˵��:   SPI�ӿ���д��һ������ʱ,���ܶ���һ������;����һ������ʱ,����д��һ������.
***************************************************************************************************/
static uint8_t SpiXReadWrite(uint8_t uWriteData,uint8_t chip)
{
	if(chip == ADNS9800_CHIP1)
	{
		//�ȴ����ͼĴ�����.
        while ((SPI1->SR & SPI_I2S_FLAG_TXE) == RESET);
        //����һ���ֽ�.
        SPI1->DR = uWriteData;
        //�ȴ����ռĴ�����Ч.
        while ((SPI1->SR & SPI_I2S_FLAG_RXNE) == RESET);
        //����һ���ֽ�.
        return (SPI1->DR);
	}
    else if(chip == ADNS9800_CHIP2)
    {
        //�ȴ����ͼĴ�����.
        while ((SPI2->SR & SPI_I2S_FLAG_TXE) == RESET);
        //����һ���ֽ�.
        SPI2->DR = uWriteData;
        //�ȴ����ռĴ�����Ч.
        while ((SPI2->SR & SPI_I2S_FLAG_RXNE) == RESET);
        //����һ���ֽ�.
        return (SPI2->DR);
    }
    else
		{
			return 0;
		}
}

/***************************************************************************************************
����:   SPI1��.
����:   ��.
����:   ��������.
˵��:   �ⲿ���ñ�����ǰ�����ȱ�֤Ƭѡ��Ч.
***************************************************************************************************/
uint8_t SpiXRead(uint8_t chip)
{
    return SpiXReadWrite(0X00,chip);
}

/***************************************************************************************************
����:   SPI1д.
����:   uWriteData.
����:   ��.
˵��:   �ⲿ���ñ�����ǰ�����ȱ�֤Ƭѡ��Ч.
***************************************************************************************************/
void SpiXWrite(uint8_t uWriteData,uint8_t chip)
{
    SpiXReadWrite(uWriteData,chip);
}

/***************************************************************************************************
����:   SPI1��ʼ��.
����:   ��.
����:   ��.
˵��:   
***************************************************************************************************/
void SpiXSInit(void)
{
    SPI_InitTypeDef     SPI_InitStructure;
    GPIO_InitTypeDef    GPIO_InitStructure;

    //SPI1 ��������
    {
        //ʱ������.����ʱ��,SPIģ��ʱ��.
        //SPI1��APB2��
        RCC_AHB1PeriphClockCmd(SPI1_SCK_PIN_CLK, ENABLE);//ʹ��PAʱ��
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);//ʹ��SPI1 

        //��������.
        //SCK����50M�������.
        GPIO_ResetBits(SPI1_SCK_PIN_PORT, SPI1_SCK_PIN_NO);         //Ԥ��Ϊ���е�ƽ.
        GPIO_InitStructure.GPIO_Pin = SPI1_SCK_PIN_NO;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_Init(SPI1_SCK_PIN_PORT, &GPIO_InitStructure);
        //MOSI����50M�������.
        GPIO_ResetBits(SPI1_MOSI_PIN_PORT, SPI1_MOSI_PIN_NO);       //Ԥ��Ϊ���е�ƽ.
        GPIO_InitStructure.GPIO_Pin = SPI1_MOSI_PIN_NO;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_Init(SPI1_MOSI_PIN_PORT, &GPIO_InitStructure);
        //MISO������������.
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

    //SPI1ģ������.
    {
        SPI_Cmd(SPI1, DISABLE);                                                 //�����Ƚ���,���ܸı�MODE.

        SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;      //����ȫ˫��.
        SPI_InitStructure.SPI_Mode = SPI_Mode_Master;                           //��.
        SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;                       //8λ.
        SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;                             //ʱ�����ʱΪ��.
        SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;                            //�����ڵ�2��ʱ���ز���.
        SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;                               //���NSS.
        SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;     // 64��Ƶ=1.3125M��APB2=84MHZ
        SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;                      //��λ��ǰ.
        SPI_InitStructure.SPI_CRCPolynomial = 7;                                //CRC7.
        
        SPI_Init(SPI1, &SPI_InitStructure);

        //ʹ��SPI1.
        SPI_Cmd(SPI1, ENABLE); 
    }

    //SPI2 ��������
    {
        //ʱ������.����ʱ��,SPIģ��ʱ��.
        //SPI1��APB2��
        RCC_AHB1PeriphClockCmd(SPI2_SCK_PIN_CLK, ENABLE);//ʹ��PBʱ��
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);//ʹ��SPI2 

        //��������.
        //SCK����50M�������.
        GPIO_ResetBits(SPI2_SCK_PIN_PORT, SPI2_SCK_PIN_NO);         //Ԥ��Ϊ���е�ƽ.
        GPIO_InitStructure.GPIO_Pin = SPI2_SCK_PIN_NO;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_Init(SPI2_SCK_PIN_PORT, &GPIO_InitStructure);
        //MOSI����50M�������.
        GPIO_ResetBits(SPI2_MOSI_PIN_PORT, SPI2_MOSI_PIN_NO);       //Ԥ��Ϊ���е�ƽ.
        GPIO_InitStructure.GPIO_Pin = SPI2_MOSI_PIN_NO;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_Init(SPI2_MOSI_PIN_PORT, &GPIO_InitStructure);
        //MISO������������.
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

    //SPI2ģ������.
    {
        SPI_Cmd(SPI2, DISABLE);                                                 //�����Ƚ���,���ܸı�MODE.

        SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;      //����ȫ˫��.
        SPI_InitStructure.SPI_Mode = SPI_Mode_Master;                           //��.
        SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;                       //8λ.
        SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;                             //ʱ�����ʱΪ��.
        SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;                            //�����ڵ�2��ʱ���ز���.
        SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;                               //���NSS.
        SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;     // 32��Ƶ=1.3125M��APB1=42MHZ
        SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;                      //��λ��ǰ.
        SPI_InitStructure.SPI_CRCPolynomial = 7;                                //CRC7.
        
        SPI_Init(SPI2, &SPI_InitStructure);

        //ʹ��SPI1.
        SPI_Cmd(SPI2, ENABLE); 
    }
}

/***************************************************************************************************
����:   ��ʱus.
����:   uNum        ΢����.
����:   ��.
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
����:   ��ʱms.
����:   uNum        ������.
����:   ��.
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
����:   ����CS.
����:   ��.
����:   ��.
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
����:   ����CS.
����:   ��.
����:   ��.
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
����:   ���Ĵ���.
����:   uAddr       �Ĵ�����ַ.
����:   ֵ.
***************************************************************************************************/
uint8_t A9800xReadReg(uint8_t uAddr,uint8_t chip)
{
	uint8_t   uVal;

	A9800xSetCsLow(chip);

	A9800DelayUs(3);
	SpiXWrite(uAddr,chip);
	A9800DelayUs(100);//VENTUS: tSRAD=100us. 
	//�ڱ�֤��tSRAD֮�󣬲���CPIÿ�ζ���ȡ��ȷ�ˣ�����SROM_check�������ܳɹ�����ʱʧ�ܣ�OBSERVATION_FAILED.
	
	uVal = SpiXRead(chip);
	A9800DelayUs(12);

    A9800xSetCsHigh(chip);

    //VENTUS:
    A9800DelayUs(20);//tSRW=tSRR=20us

	return uVal;
}
/***************************************************************************************************
����:   д�Ĵ���.
����:   uAddr       �Ĵ�����ַ.
        uVal        �Ĵ���ֵ.
����:   ֵ.
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
		//�ڱ�֤��tSWR,tSWW,tSRR,tSRW�Լ�tSRAD֮��ÿһ��SROM check����ͨ��������CPI���ɶ�ȡ��ȷ.
		//A9800DelayMs(10);
}

/***************************************************************************************************
����:   д��CPI.
����:   uCpi            CPI����.1-164.ÿ1����50CPI.�Ĵ���Ĭ��68(3400CPI).
����:   ��.
***************************************************************************************************/
void A9800xWriteCpi(uint8_t uCpi,uint8_t chip)
{
    A9800xWriteReg(A9800_CONFIGURATION_I_ADDR, uCpi,chip);
}

/***************************************************************************************************
����:   д��̧��߶�����.
����:   uLift           ̧��߶�����.1-31.�Ĵ���Ĭ��16.
����:   ��.
***************************************************************************************************/
void A9800xWriteLift(uint8_t uLift,uint8_t chip)
{
    A9800xWriteReg(A9800_LIFT_DETECTION_THR_ADDR, uLift,chip);
}

/***************************************************************************************************
����:   ����CPI.
����:   uCpi            CPI����.1-164.ÿ1����50CPI.�Ĵ���Ĭ��68(3400CPI).
����:   ��.
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

    //DebugLog("����CPI "); DebugLogAddVal(uCpi, 3); DebugLogAddStr("\r\n");
    g_tADNS9800.g_uA9800Cpi = uCpi;
    A9800xWriteCpi(uCpi,ADNS9800_CHIP1);
    A9800xWriteCpi(uCpi,ADNS9800_CHIP2);
}

/***************************************************************************************************
����:   ����̧��߶�����.
����:   uLift           ̧��߶�����.1-31.�Ĵ���Ĭ��15.
����:   ��.
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

    //DebugLog("����LIFT "); DebugLogAddVal(uLift, 3); DebugLogAddStr("\r\n");
    g_tADNS9800.g_uA9800Lift = uLift;
    A9800xWriteLift(uLift,ADNS9800_CHIP1);
    A9800xWriteLift(uLift,ADNS9800_CHIP2);
}

/***************************************************************************************************
����:   ͻ�����ƶ��Ĵ���.
����:   puBuf       ������.����������������.
        uNum        �����������.
����:   ��.
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
����:   �ƶ���⹤��.
����:   ��.
����:   ��.
˵��:   ������ÿ1msִ��һ��.
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
����:   ��ʱ1ms����.
����:   ��.
����:   ��.
***************************************************************************************************/
void A9800OnTimerTask(void)
{
    A9800MoveDetectWork(ADNS9800_CHIP1);
    A9800MoveDetectWork(ADNS9800_CHIP2);
}

/***************************************************************************************************
����:   ���ID.
����:   ��.
����:   ��.
***************************************************************************************************/
void A9800xInitCheckId(uint8_t chip)
{
    g_uA9800Dummy = A9800xReadReg(A9800_PRODUCTID_ADDR,chip);            
    g_uA9800Dummy = A9800xReadReg(A9800_REVISIONID_ADDR,chip);
    g_uA9800Dummy = A9800xReadReg(A9800_INVERSE_PRODUCT_ID_ADDR,chip);
}
/***************************************************************************************************
����:   �ϵ�.
����:   ��.
����:   ��.
***************************************************************************************************/
void A9800xInitPowerUp(uint8_t chip)
{   //�ϵ�.
    A9800xWriteReg(A9800_POWER_UP_RESET_ADDR, A9800_POWER_UP_RESET, chip);
    A9800DelayMs(50);                   //��ʱ50ms.

    g_uA9800Dummy = A9800xReadReg(A9800_MOTION_ADDR, chip);
    g_uA9800Dummy = A9800xReadReg(A9800_DELTAX_L_ADDR, chip);
    g_uA9800Dummy = A9800xReadReg(A9800_DELTAX_H_ADDR, chip);
    g_uA9800Dummy = A9800xReadReg(A9800_DELTAY_L_ADDR, chip);
    g_uA9800Dummy = A9800xReadReg(A9800_DELTAY_H_ADDR, chip);
}

/***************************************************************************************************
����:   ����SROM.
����:   ��.
����:   ��.
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
����:   ���SROM.
����:   ��.
����:   �������.
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
����:   ʹ�ܼ����Դ.
����:   ��.
����:   ��.
***************************************************************************************************/
void A9800xEnLaserPower(uint8_t chip)
{
    uint8_t uData;    

    uData = A9800xReadReg(A9800_LASER_CTRL0_ADDR, chip);
    A9800xWriteReg(A9800_LASER_CTRL0_ADDR, (uData & 0xFE), chip);                // Enable Laser Power
}
/***************************************************************************************************
����:   ��ʼ��SROM����ʹ�ܼ���
����:   ��.
����:   ��.
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
		A9800DelayUs(20);//read����һ������֮����20us
        regres = A9800xReadReg(oreg[rctr], chip);
        sprintf(tmp,"%s:%x",oregname[rctr],regres);
        printx(tmp);  
    }
    A9800xSetCsHigh(chip);
}

/***************************************************************************************************
����:   ��ʼ��.
����:   bIap    IAP״̬.
����:   ��.
***************************************************************************************************/
void A9800xInit(uint8_t bIap, uint8_t chip)
{
    SpiXSInit();

    GPIO_InitTypeDef    GPIO_InitStructure;
    uint8_t               auBuf[10];
    if(chip == ADNS9800_CHIP1)
    {
        printx("Initiallizing ADNS9800-1...\n----------------------------------");
        //Ƭѡ��.
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
        //Ƭѡ��.
        RCC_AHB1PeriphClockCmd(A9800_2_CS_PIN_CLK, ENABLE);
        GPIO_SetBits(A9800_2_CS_PIN_PORT, A9800_2_CS_PIN_NO);
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_Pin = A9800_2_CS_PIN_NO;
        GPIO_Init(A9800_2_CS_PIN_PORT, &GPIO_InitStructure); 
    }


    if (bIap == 1)          //IAP״̬��,��������ʼ��,������A9800.
    {
        return;
    }
    
    A9800xInitCheckId(chip);
    A9800xInitPowerUp(chip);
    A9800xInitSrom(chip);
    A9800xdispRegisters(chip);

    //A9800xWriteCpi(g_uA9800Cpi, chip);
		A9800xWriteCpi((uint8_t)(_cfg_ADNS9800_CPI/50), chip);
		A9800DelayMs(1);//VENTUS;�ײ���д��CPI���ٴζ�ȡ֮����Ҫһ�������1ms���Ա�֤�ٴζ�ȡ����CPI����ȷ��
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
    g_tADNS9800.g_uA9800Cpi = A9800_RES6350CPI;     //CPIֵ.
    g_tADNS9800.g_uA9800Lift = 16;                  //̧��߶�����.
    g_tADNS9800.MotionX_1 = 0;
    g_tADNS9800.MotionY_1 = 0;
    g_tADNS9800.MotionX_2 = 0;
    g_tADNS9800.MotionY_2 = 0;
    g_tADNS9800.PosX_1 = 0;                //X��λ��.
    g_tADNS9800.PosY_1 = 0;                //Y��λ��.
    g_tADNS9800.PosX_2 = 0;              //X��λ��.��2�����оƬ
    g_tADNS9800.PosY_2 = 0;              //Y��λ��.��2�����оƬ
    g_tADNS9800.TotalMotionX = 0;						//x�����ֵλ�ƣ����оƬ-1
    g_tADNS9800.TotalMotionY = 0;						//y�����ֵλ�ƣ����оƬ-1
    g_tADNS9800.TotalMotionX_2 = 0;					//x�����ֵλ�ƣ����оƬ-2
    g_tADNS9800.TotalMotionY_2 = 0;					//y�����ֵλ�ƣ����оƬ-2

    A9800xInit(bIap,ADNS9800_CHIP1);
    A9800xInit(bIap,ADNS9800_CHIP2);
}

void bsp_ADNS9800_Initializes(void)
{
    printx("Initializing ADNS9800s...");
    A9800XSInit(0);
}

/**** Copyright (C)2018 Ventus Guo. All Rights Reserved **** END OF FILE ****/

