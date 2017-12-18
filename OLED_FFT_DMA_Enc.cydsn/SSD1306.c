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
#include "SSD1306.h"
#include "project.h"

#define WRITECHUNK_LEN      (128u)
#define WRITEBUFF_LEN       (WRITECHUNK_LEN+1u)

uint32 status = 0;
uint16 OLEDBuffIdx = 0;
uint8 updateProcessFlag = 0;

uint8 OLEDBuff[OLED_BUFF_SIZE] = {0};
uint8 OLEDWriteBuff[OLED_BUFF_SIZE] = {0};


void OLED_SendByte(uint8 byte)
{
    status = I2C_MasterSendStart(OLED_I2C_ADDRESS, I2C_WRITE_XFER_MODE);
    if(status == I2C_MSTR_NO_ERROR)
    {
        I2C_MasterWriteByte(byte);
    }
    I2C_MasterSendStop();
}

void OLED_SendCmd(uint8 cmd)
{
     status = I2C_MasterSendStart(OLED_I2C_ADDRESS, I2C_WRITE_XFER_MODE);
    if(status == I2C_MSTR_NO_ERROR)
    {
        I2C_MasterWriteByte(OLED_CONTROL_BYTE_CMD);
        I2C_MasterWriteByte(cmd);
    }
    I2C_MasterSendStop();
}

void OLED_Clear(void)
{
    memset(OLEDBuff, 0, OLED_BUFF_SIZE);
}

void OLED_Init(void)
{
    I2C_Start();
    
    OLED_Clear();
    
    CyDelay(10);
    
    // Init sequence
    OLED_SendCmd(OLED_DISPLAYOFF);            // 0xAE
    OLED_SendCmd(OLED_SETDISPLAYCLOCKDIV);    // 0xD5
    OLED_SendCmd(0xF0);                       // the suggested ratio 0x80

    OLED_SendCmd(OLED_SETMULTIPLEX);          // 0xA8
    OLED_SendCmd(OLED_LCDHEIGHT - 1);

    OLED_SendCmd(OLED_SETDISPLAYOFFSET);      // 0xD3
    OLED_SendCmd(0x00);                       // no offset
    OLED_SendCmd(OLED_SETSTARTLINE | 0x0);    // line #0
    OLED_SendCmd(OLED_CHARGEPUMP);            // 0x8D
    OLED_SendCmd(0x14);
    OLED_SendCmd(OLED_MEMORYMODE);            // 0x20
    OLED_SendCmd(0x00);                       // 0x0 act like ks0108
    OLED_SendCmd(OLED_SEGREMAP | 0x1);
    OLED_SendCmd(OLED_COMSCANDEC);

    OLED_SendCmd(OLED_SETCOMPINS);            // 0xDA
    
    #if OLED_LCDHEIGHT == 64
        OLED_SendCmd(0x12);
    #elif OLED_LCDHEIGHT == 32
        OLED_SendCmd(0x02);
    #endif
    
    OLED_SendCmd(OLED_SETCONTRAST);           // 0x81
    OLED_SendCmd(0x8F);

    OLED_SendCmd(OLED_SETPRECHARGE);          // 0xd9
    OLED_SendCmd(0xF1);
    OLED_SendCmd(OLED_SETVCOMDETECT);         // 0xDB
    OLED_SendCmd(0x20);
    OLED_SendCmd(OLED_DISPLAYALLON_RESUME);   // 0xA4
    OLED_SendCmd(OLED_NORMALDISPLAY);         // 0xA6

    OLED_SendCmd(OLED_DEACTIVATE_SCROLL);

    OLED_SendCmd(OLED_DISPLAYON);             //--turn on oled panel
    
    OLEDBuff[0] = 0xFF;
    OLEDBuff[OLED_BUFF_SIZE-1] = 0xDD;
    
    updateProcessFlag = 1;
    
}

void OLED_SetContrast(uint8 contrast)
{
    OLED_SendCmd(OLED_SETCONTRAST);
    OLED_SendCmd(contrast);
}

void OLED_Update(void)
{
    memcpy(OLEDWriteBuff, OLEDBuff, OLED_BUFF_SIZE);
    updateProcessFlag = 1;
}


void OLED_Process(void)
{
    static uint8 wrBuff[WRITEBUFF_LEN] = {0};
    static uint16 cnt = 0;
    
    if(updateProcessFlag != 0)
    {
        Pin_DBG2_Write(1);
        if((I2C_MasterStatus() & I2C_MSTAT_WR_CMPLT) != 0 ||
            cnt <= 1)
        {
            I2C_MasterClearStatus();
            
            if(cnt == 0)
            {
                OLED_SendCmd(OLED_COLUMNADDR);
                OLED_SendCmd(0);   // Column start address (0 = reset)
                OLED_SendCmd(OLED_LCDWIDTH-1); // Column end address (127 = reset)
                
                OLED_SendCmd(OLED_PAGEADDR);
                OLED_SendCmd(0); // Page start address (0 = reset)
                
                #if OLED_LCDHEIGHT == 64
                    OLED_SendCmd(7); // Page end address
                #elif OLED_LCDHEIGHT == 32
                    OLED_SendCmd(3); // Page end address
                #elif OLED_LCDHEIGHT == 16
                    OLED_SendCmd(1); // Page end address
                #endif
            }
            else
            {
                wrBuff[0] = 0x40;
                memcpy(&wrBuff[1], &OLEDWriteBuff[(cnt-1) * WRITECHUNK_LEN], WRITECHUNK_LEN);
                I2C_MasterWriteBuf(OLED_I2C_ADDRESS, wrBuff, WRITEBUFF_LEN, I2C_MODE_COMPLETE_XFER);
            }

            cnt++;
            if(cnt > (OLED_BUFF_SIZE/WRITECHUNK_LEN))
            {
                cnt = 0;
                updateProcessFlag = 0;
                Pin_DBG2_Write(0);
            }
        }
    }
}


uint8 OLED_Pixel ( uint8 x, uint8 y, OLED_PixelMode_t mode )
{
    int  index;
    uint8  offset;
    uint8  data;

    if ( x >= OLED_X_RES || y >= OLED_Y_RES) return OLED_OUT_OF_BORDER;

    index = ( ( y / 8 ) * OLED_X_RES ) + x;
    offset  = y - ( ( y / 8 ) * 8 );

    data = OLEDBuff[ index ];

    switch(mode)
    {
        case OLED_PIXEL_OFF:
            data &= ( ~( 0x01 << offset ) );
            break;
        case OLED_PIXEL_ON:
            data |= ( 0x01 << offset );
            break;
        case OLED_PIXEL_XOR:
            data ^= ( 0x01 << offset );
            break;
    }

    OLEDBuff[ index ] = data;

    return OLED_OK;
}

uint8 OLED_GotoXYFont(uint8 x, uint8 y)
{
    if( x > OLED_X_TXT1_RES || y > OLED_Y_TXT1_RES ) return OLED_OUT_OF_BORDER;

    OLEDBuffIdx = x* OLED_X_TXT1_SIZE + y * OLED_X_RES;
    return OLED_OK;
}

uint8 OLED_Chr (OLEDFontSize_t size, uint8 ch )
{
    uint16 LcdCacheIdxOld = 0;
    int8 cntX = 0;
    int8 cntY = 0;
    int8 i = 0;
    #if LCD_FONT_MAX_SIZE <= 4
    uint32 fontMap = 0;
    uint32 temp = 0;
    uint32 mask = 0;
    #else
    uint64 fontMap = 0;
    uint64 temp = 0;
    uint64 mask = 0;
    #endif
    
    LcdCacheIdxOld = OLEDBuffIdx;

    if (ch >= ' ')
    {
        //ASCII[0x20-0x7F]
        ch -= ' ';
    }
    
    if(size > OLED_FONT_1X)
    {
        if((int16)(OLEDBuffIdx - (size - 1)*OLED_X_RES) < 0)
        {
            return OLED_OUT_OF_BORDER;
        }
        
        for (cntX = 0; cntX < (int8)(OLED_X_TXT1_SIZE*size); cntX++)
        {
            if(cntX % size == 0)
            {
                fontMap = (FontLookup[ch][cntX/size] <<1);
                temp = 0;
                mask = 0x01;

                for(i = 0; i < (int8)size*8; i++)
                {
                    temp |= fontMap & mask;
                    if((i+1)%size != 0)
                    {
                        fontMap <<= 1;
                    }
                    mask <<= 1;

                }
            }
            
            for(cntY = size-1; cntY >= 0; cntY --)
            {
                OLEDBuff[OLEDBuffIdx - cntY*OLED_X_RES] = 
                    (uint8)(temp >> (size - cntY - 1)*8);
            }
            OLEDBuffIdx++;
        }
        
        for(cntX = 0; cntX < (int8)size; cntX++)
        {
            for(cntY = size-1; cntY >= 0; cntY --)
            {
                OLEDBuff[OLEDBuffIdx - cntY*OLED_X_RES] = 0;
            }
            OLEDBuffIdx++;
        }
    }
    else
    {
        for ( i = 0; i < OLED_X_TXT1_SIZE; i++ )
        {
            OLEDBuff[OLEDBuffIdx] = (FontLookup[ch][i]<<1);
            OLEDBuffIdx++;
        }
        OLEDBuff[OLEDBuffIdx] = 0x00;
        
        OLEDBuffIdx++;
    }
    
    
    
    if(OLEDBuffIdx/OLED_X_RES != LcdCacheIdxOld/OLED_X_RES )
    {
        return OLED_OK_WITH_WRAP;
    }
    
    return OLED_OK;
}

uint8 OLED_Str (OLEDFontSize_t size, const char *dataArray)
{
    uint8 tmpIdx=0;
    uint8 response;
    while(dataArray[tmpIdx] != '\0')
    {
        response = OLED_Chr(size, dataArray[ tmpIdx ]);

        if(response == OLED_OUT_OF_BORDER)
            return OLED_OUT_OF_BORDER;

        tmpIdx++;
    }
    return OLED_OK;
}

uint8 OLED_Chr1X (uint8 ch)
{
    return OLED_Chr (OLED_FONT_1X,  ch );
}

uint8 OLED_Str1X (const char *dataArray)
{
    return OLED_Str(OLED_FONT_1X, dataArray);
}




uint8 OLED_Line ( uint8 x1, uint8 y1, uint8 x2, uint8 y2, OLED_PixelMode_t mode )
{
    int16 dx, dy, stepx, stepy, fraction;
    uint8 response;

    // dy   y2 - y1
    // -- = -------
    // dx   x2 - x1

    dy = y2 - y1;
    dx = x2 - x1;

    if ( dy < 0 )
    {
        dy    = -dy;
        stepy = -1;
    }
    else
    {
        stepy = 1;
    }

    if ( dx < 0 )
    {
        dx    = -dx;
        stepx = -1;
    }
    else
    {
        stepx = 1;
    }

    dx <<= 1;
    dy <<= 1;

    response = OLED_Pixel( x1, y1, mode );
    if(response != OLED_OK)
    {
        return response;
    }

    if ( dx > dy )
    {
        fraction = dy - ( dx >> 1);
        while ( x1 != x2 )
        {
            if ( fraction >= 0 )
            {
                y1 += stepy;
                fraction -= dx;
            }
            x1 += stepx;
            fraction += dy;

            response = OLED_Pixel( x1, y1, mode );
            if(response != OLED_OK)
            {
                return response;
            }

        }
    }
    else
    {
        fraction = dx - ( dy >> 1);
        while ( y1 != y2 )
        {
            if ( fraction >= 0 )
            {
                x1 += stepx;
                fraction -= dy;
            }
            y1 += stepy;
            fraction += dx;

            response = OLED_Pixel( x1, y1, mode );
            if(response != OLED_OK)
            {
                return response;
            }
        }
    }

    return OLED_OK;
}


/* [] END OF FILE */
