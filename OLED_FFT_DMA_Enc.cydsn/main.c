/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
#include "project.h"

#include <stdio.h>
#include "arm_math.h"
#include "SSD1306.h"

#define FFT_LEN_SPECTR    128
#define FFT_AVG_NUM       2u

#define GRAPH_128_FALLSPEED (4)
#define GRAPH_128_MAX_FALLDIV (5)

#define FFT_128p_LOG_COEF   (1.01)
#define FFT_128p_LIN_COEF   (0.1)

#define FFT_64p_LOG_COEF   (1.009)
#define FFT_64p_LIN_COEF   (0.2)

#define FFT_32p_LOG_COEF   (1.03)
#define FFT_32p_LIN_COEF   (0.4)

#define LCD_X_RES     128
#define LCD_Y_RES     64

//#define FFT_CONST_CORRECT   (50)

#define TRUE    1
#define FALSE   0

#define FFT_LEN_REAL        (FFT_LEN_SPECTR*2)
#define FFT_BUFFLEN         (FFT_LEN_REAL*2)
#define FFT_MAG_BUFFLEN     FFT_LEN_REAL

#define ADC_SAMPLEBUFF_LEN  FFT_LEN_REAL

#define PARSEBUFF_LEN       (9)
#define PARSE_NO_RESULT     (UINT32_MAX)

#define UART_BUFFLEN        (32)

    /* Defines for DMA */
#define DMA_BYTES_PER_BURST 2
#define DMA_REQUEST_PER_BURST 1
#define DMA_TRANSFER_COUNT (ADC_SAMPLEBUFF_LEN*DMA_BYTES_PER_BURST)
#define DMA_SRC_BASE (CYDEV_PERIPH_BASE)
#define DMA_DST_BASE (CYDEV_SRAM_BASE)
uint8 DMA_Chan = 0;




int16 ADCSampleBuff1[ADC_SAMPLEBUFF_LEN] = {0};
int16 ADCSampleBuff2[ADC_SAMPLEBUFF_LEN] = {0};
int16 *activeSampleBuffPtr = ADCSampleBuff1;
int16 *doneSampleBuffPtr = ADCSampleBuff2;

volatile uint8 sampleBuffDoneFlag = FALSE;
volatile uint8 updateLcdFlag = TRUE;

arm_rfft_instance_q15       RealFFT_Instance;


const uint8 const32Arr[] = {1, 1, 1, 2, 2, 2, 2, 2, 
                            2, 2, 2, 2, 2, 2, 2, 2, 
                            2, 2, 2, 2, 2, 2, 2, 2, 
                            2, 2, 2, 2, 2, 3, 3, 3};

const uint8 const64Arr[] = {1, 1, 1, 1, 1, 1, 2, 2,
                            2, 2, 2, 2, 2, 2, 2, 2,
                            2, 2, 2, 2, 2, 2, 2, 2,
                            2, 2, 2, 2, 2, 2, 2, 2,
                            2, 2, 2, 2, 2, 2, 2, 2,
                            2, 2, 2, 2, 2, 2, 2, 2,
                            2, 2, 2, 2, 2, 2, 2, 2,
                            2, 2, 3, 3, 3, 3, 3, 3};

enum
{
    MODE_FFT_128 = 0,
    MODE_FFT_32,
    MODE_COUNT
};

volatile uint8 mode = MODE_FFT_128;

    int32 displayVal[FFT_LEN_SPECTR];
    int16 maxBuff[FFT_LEN_SPECTR];
    int16 magBuff[FFT_LEN_SPECTR];

void DMA_Setup()
{

    /* Variable declarations for DMA */
    /* Move these variable declarations to the top of the function */
    
    uint8 DMA_TD[2];

    /* DMA Configuration for DMA */
    DMA_Chan = DMA_DmaInitialize(DMA_BYTES_PER_BURST, DMA_REQUEST_PER_BURST, 
        HI16(DMA_SRC_BASE), HI16(DMA_DST_BASE));
    DMA_TD[0] = CyDmaTdAllocate();
    DMA_TD[1] = CyDmaTdAllocate();
    CyDmaTdSetConfiguration(DMA_TD[0], DMA_TRANSFER_COUNT, DMA_TD[1], 
                            DMA__TD_TERMOUT_EN | CY_DMA_TD_INC_DST_ADR);
    CyDmaTdSetConfiguration(DMA_TD[1], DMA_TRANSFER_COUNT, DMA_TD[0], 
                            DMA__TD_TERMOUT_EN | CY_DMA_TD_INC_DST_ADR);
    CyDmaTdSetAddress(DMA_TD[0], LO16((uint32)ADC_DelSig_DEC_SAMP_PTR), LO16((uint32)ADCSampleBuff1));
    CyDmaTdSetAddress(DMA_TD[1], LO16((uint32)ADC_DelSig_DEC_SAMP_PTR), LO16((uint32)ADCSampleBuff2));
    CyDmaChSetInitialTd(DMA_Chan, DMA_TD[0]);
    CyDmaChEnable(DMA_Chan, 1);
}



CY_ISR(ADC_EOS_Handler)
{
    if(activeSampleBuffPtr == ADCSampleBuff1)
    {
        activeSampleBuffPtr = ADCSampleBuff2;
        doneSampleBuffPtr = ADCSampleBuff1;
    }
    else
    {
        activeSampleBuffPtr = ADCSampleBuff1;
        doneSampleBuffPtr = ADCSampleBuff2;
    }
    
    sampleBuffDoneFlag = TRUE;
}




CY_ISR(isr_ButtonPoll_Handler)
{
    static uint8 cnt = 0;
    if(Pin_Button_Read() == 0)
    {
        cnt++;
    }
    else
    {
        if(cnt > 5)
        {
            mode++;
            {
                if(mode >= MODE_COUNT)
                {
                    mode = 0;
                }
            }
        }
        cnt = 0;
    }
}


void Calc_FFTmag(const int16 *dataPtr, q15_t *magBuff)
{
    int16 sigBuff[ADC_SAMPLEBUFF_LEN];
    static q15_t sigFFT[FFT_BUFFLEN];
    uint16 cnt = 0;
    
    memcpy(sigBuff, dataPtr, ADC_SAMPLEBUFF_LEN*sizeof(int16));
            
    arm_rfft_q15( &RealFFT_Instance, 
		        (q15_t *)sigBuff,
		        (q15_t *)sigFFT);
    
    for(cnt=0; cnt<FFT_BUFFLEN; cnt++)
        {
            sigFFT[cnt]<<=6;
        }
    
    arm_cmplx_mag_q15((q15_t *)sigFFT,
                      (q15_t *)magBuff,
					  FFT_MAG_BUFFLEN);
}

uint32 intPow(uint32 val, uint8 pow)
{
    uint8 i = 0;
    uint32 result = 1;
    
    for(i = 0; i < pow; i++)
    {
        result *= val;
    }
    
    return result;
}

uint32 charParse(uint8 inByte)
{
    static int8 buffCnt = 0;
    static uint8 buff[PARSEBUFF_LEN] = {0};
    uint32 result = PARSE_NO_RESULT;
    uint8 i = 0;
    
    switch(inByte)
    {
        case '\r':
        case '\n':
            result = 0;
            for(i = 0; i < buffCnt; i++)
            {
                result += (buff[i] - '0') * intPow(10, buffCnt-1 - i);
            }
            buffCnt = 0;
            break;
        
        default:
            if(inByte >= '0' && inByte <= '9')
            {
                buff[buffCnt] = inByte;
                buffCnt++;
                
                if(buffCnt >= PARSEBUFF_LEN)
                {
                    buffCnt = 0;
                }
            }
            else
            {
                buffCnt = 0;
            }
            break;
    }
    
    return result;
}

void DrawSpectr128 (void)
{
    static uint16 maxDiv = 0;
    
    int8 pCnt = 0;
    int16 cnt = 0;
    int16 tempVal = 0;
    
    maxDiv++;

    OLED_Clear();
    for(cnt = 0; cnt<FFT_LEN_SPECTR; cnt++)
    {
        tempVal = (int16)(log((displayVal[cnt]/(float)FFT_AVG_NUM))/
                          log(FFT_128p_LOG_COEF)*FFT_128p_LIN_COEF);
        
        if(tempVal > LCD_Y_RES)
        {
            tempVal = LCD_Y_RES;
        }
        
        if(tempVal > magBuff[cnt])
        {
            magBuff[cnt] = tempVal;
        }
        
        if(magBuff[cnt] > 0)
        {
            for(pCnt = LCD_Y_RES - magBuff[cnt]; pCnt < LCD_Y_RES; pCnt++ )
            {
                OLED_Pixel((uint8)cnt, (uint8)pCnt, OLED_PIXEL_ON);
            } 
        }
        
        if(tempVal > maxBuff[cnt])
        {
            maxBuff[cnt] = tempVal;
        }
        else if(maxBuff[cnt] > 0)
        {
            OLED_Pixel((uint8)cnt, (uint8)(LCD_Y_RES - maxBuff[cnt]), OLED_PIXEL_ON);
        }
    }
    
    OLED_Update();
    
    maxDiv++;
    
    if(maxDiv >= GRAPH_128_MAX_FALLDIV)
    {
        maxDiv = 0;
        for(cnt = 0; cnt<FFT_LEN_SPECTR; cnt++)
        {
            if(maxBuff[cnt] > 0)
            {
                maxBuff[cnt] -= 1;
            }
        }
    }
    
    for(cnt = 0; cnt<FFT_LEN_SPECTR; cnt++)
    {
        if(magBuff[cnt] > 0)
        {
            magBuff[cnt] -= GRAPH_128_FALLSPEED;
            if(magBuff[cnt] <0)
            {
                magBuff[cnt] = 0;
            }
        }
        
    }
}


void DrawSpectr64(void)
{
    uint8 prevSum = 0;
    float tempFloat = 0;
    int16 cnt = 0;
    int16 pCnt = 0;
    int16 tempVal = 0;
    
    OLED_Clear();
   
    prevSum = 0;
    
    for(cnt = 0; cnt < LCD_Y_RES; cnt++)
    {
        tempFloat = 0;
        for(pCnt = prevSum; pCnt < prevSum+const64Arr[cnt]; pCnt++)
        {
            tempFloat += displayVal[pCnt]/(float)FFT_AVG_NUM;
        }
        
        tempFloat/= const64Arr[cnt];
        prevSum+=const64Arr[cnt];
        
        tempVal = (int16)(log(tempFloat)/log(FFT_64p_LOG_COEF) 
                                            *FFT_64p_LIN_COEF);
        
        if(tempVal > LCD_X_RES)
        {
            tempVal = LCD_X_RES;
        }
        
        if(tempVal > magBuff[cnt])
        {
            magBuff[cnt] = tempVal;
        }
        if(magBuff[cnt] > 0)
        {
            for(pCnt = 0 ; pCnt < magBuff[cnt] ; pCnt++ )
            {
                OLED_Pixel((uint8)pCnt, (uint8)cnt, OLED_PIXEL_ON);
            } 
        }
        
        if(tempVal > maxBuff[cnt])
        {
            maxBuff[cnt] = (int16)tempVal;
        }
        else 
        if(maxBuff[cnt] > 0)
        {
            OLED_Pixel((uint8)maxBuff[cnt], (uint8)cnt, OLED_PIXEL_ON);
        }
    }
    
    OLED_Update();

    for(cnt = 0; cnt<FFT_LEN_SPECTR; cnt++)
    {
        if(maxBuff[cnt] > 0)
        {
            maxBuff[cnt]--;
        }
        
        if(magBuff[cnt] > 0)
        {
            magBuff[cnt] -=8;
            if(magBuff[cnt] <0)
            {
                magBuff[cnt] = 0;
            }
        }
    }
}


int main(void)
{
    int16 cnt = 0;
    q15_t sigFFT_Mag[FFT_MAG_BUFFLEN];
    uint16 avgNum = 0;   
    
    CyGlobalIntEnable; /* Enable global interrupts. */
    
    DMA_Setup();
    
    PGA_Inv_1_Start();
    PGA_1_Start();
    OpampVref_Start();
    isr_ADC_EOS_StartEx(ADC_EOS_Handler);
    
    OLED_Init();
    ADC_DelSig_Start();
    ADC_DelSig_StartConvert();
    isr_ButtonPoll_StartEx(isr_ButtonPoll_Handler);
    
    arm_rfft_init_q15(&RealFFT_Instance,
                      FFT_LEN_REAL,
                      0,
                      1);
    
    memset(maxBuff, 0, LCD_Y_RES);
    
    for(;;)
    {
        OLED_Process();
        
        if(sampleBuffDoneFlag == TRUE)
        {
            /* change pin level to measure calculations time
            * with logic analyser or oscilloscope
            */
            Pin_DBG1_Write(1);
            
            sampleBuffDoneFlag = FALSE;
            
            Calc_FFTmag(doneSampleBuffPtr, sigFFT_Mag);
            
            for(cnt = 0; cnt<FFT_LEN_SPECTR; cnt++)
            {
                displayVal[cnt] += (int32)sigFFT_Mag[cnt]>>6;
            }

            avgNum++;
            
            if(avgNum >= FFT_AVG_NUM)
            {
                avgNum = 0;
                updateLcdFlag = TRUE;
            }
            
            Pin_DBG1_Write(0);
        }
        
        if(updateLcdFlag == TRUE)
        {
            updateLcdFlag = FALSE;
            switch(mode)
            {
            case MODE_FFT_128:
                DrawSpectr128();
                break;
                    
            case MODE_FFT_32:
                DrawSpectr64();
                break;     
            } 
            
            for(cnt = 0; cnt<FFT_LEN_SPECTR; cnt++)
            {
                displayVal[cnt] = 0;
            }
        }
    }
}





/* [] END OF FILE */
