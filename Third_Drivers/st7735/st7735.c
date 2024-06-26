/* vim: set ai et ts=4 sw=4: */
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "st7735.h"
// #include "malloc.h"
#include "string.h"

extern void SysTick_DelayTicks(uint32_t n);
#define DELAY 0x80
#define HAL_Delay SysTick_DelayTicks

static lpspi_transfer_t masterXfer;

// based on Adafruit ST7735 library for Arduino
static const uint8_t
    init_cmds1[] = {           // Init for 7735R, part 1 (red or green tab)
        15,                    // 15 commands in list:
        ST7735_SWRESET, DELAY, //  1: Software reset, 0 args, w/delay
        150,                   //     150 ms delay
        ST7735_SLPOUT, DELAY,  //  2: Out of sleep mode, 0 args, w/delay
        255,                   //     500 ms delay
        ST7735_FRMCTR1, 3,     //  3: Frame rate ctrl - normal mode, 3 args:
        0x01, 0x2C, 0x2D,      //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
        ST7735_FRMCTR2, 3,     //  4: Frame rate control - idle mode, 3 args:
        0x01, 0x2C, 0x2D,      //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
        ST7735_FRMCTR3, 6,     //  5: Frame rate ctrl - partial mode, 6 args:
        0x01, 0x2C, 0x2D,      //     Dot inversion mode
        0x01, 0x2C, 0x2D,      //     Line inversion mode
        ST7735_INVCTR, 1,      //  6: Display inversion ctrl, 1 arg, no delay:
        0x07,                  //     No inversion
        ST7735_PWCTR1, 3,      //  7: Power control, 3 args, no delay:
        0xA2,
        0x02,             //     -4.6V
        0x84,             //     AUTO mode
        ST7735_PWCTR2, 1, //  8: Power control, 1 arg, no delay:
        0xC5,             //     VGH25 = 2.4C VGSEL = -10 VGH = 3 * AVDD
        ST7735_PWCTR3, 2, //  9: Power control, 2 args, no delay:
        0x0A,             //     Opamp current small
        0x00,             //     Boost frequency
        ST7735_PWCTR4, 2, // 10: Power control, 2 args, no delay:
        0x8A,             //     BCLK/2, Opamp current small & Medium low
        0x2A,
        ST7735_PWCTR5, 2, // 11: Power control, 2 args, no delay:
        0x8A, 0xEE,
        ST7735_VMCTR1, 1, // 12: Power control, 1 arg, no delay:
        0x0E,
        ST7735_INVOFF, 0, // 13: Don't invert display, no args, no delay
        ST7735_MADCTL, 1, // 14: Memory access control (directions), 1 arg:
        ST7735_ROTATION,  //     row addr/col addr, bottom to top refresh
        ST7735_COLMOD, 1, // 15: set color mode, 1 arg, no delay:
        0x05},            //     16-bit color

#if (defined(ST7735_IS_128X128) || defined(ST7735_IS_160X128))
    init_cmds2[] = {     // Init for 7735R, part 2 (1.44" display)
        2,               //  2 commands in list:
        ST7735_CASET, 4, //  1: Column addr set, 4 args, no delay:
        0x00, 0x00,      //     XSTART = 0
        0x00, 0x7F,      //     XEND = 127
        ST7735_RASET, 4, //  2: Row addr set, 4 args, no delay:
        0x00, 0x00,      //     XSTART = 0
        0x00, 0x7F},     //     XEND = 127
#endif                   // ST7735_IS_128X128

#ifdef ST7735_IS_160X80
    init_cmds2[] = {      // Init for 7735S, part 2 (160x80 display)
        3,                //  3 commands in list:
        ST7735_CASET, 4,  //  1: Column addr set, 4 args, no delay:
        0x00, 0x00,       //     XSTART = 0
        0x00, 0x4F,       //     XEND = 79
        ST7735_RASET, 4,  //  2: Row addr set, 4 args, no delay:
        0x00, 0x00,       //     XSTART = 0
        0x00, 0x9F,       //     XEND = 159
        ST7735_INVON, 0}, //  3: Invert colors
#endif

    init_cmds3[] = {                                                                                                         // Init for 7735R, part 3 (red or green tab)
        4,                                                                                                                   //  4 commands in list:
        ST7735_GMCTRP1, 16,                                                                                                  //  1: Gamma Adjustments (pos. polarity), 16 args, no delay:
        0x02, 0x1c, 0x07, 0x12, 0x37, 0x32, 0x29, 0x2d, 0x29, 0x25, 0x2B, 0x39, 0x00, 0x01, 0x03, 0x10, ST7735_GMCTRN1, 16,  //  2: Gamma Adjustments (neg. polarity), 16 args, no delay:
        0x03, 0x1d, 0x07, 0x06, 0x2E, 0x2C, 0x29, 0x2D, 0x2E, 0x2E, 0x37, 0x3F, 0x00, 0x00, 0x02, 0x10, ST7735_NORON, DELAY, //  3: Normal display on, no args, w/delay
        10,                                                                                                                  //     10 ms delay
        ST7735_DISPON, DELAY,                                                                                                //  4: Main screen turn on, no args w/delay
        100};                                                                                                                //     100 ms delay

static void ST7735_Select()
{
  // HAL_GPIO_WritePin(ST7735_CS_GPIO_Port, ST7735_CS_Pin, GPIO_PIN_RESET);
}

void ST7735_Unselect()
{
  // HAL_GPIO_WritePin(ST7735_CS_GPIO_Port, ST7735_CS_Pin, GPIO_PIN_SET);
}

static void ST7735_Reset()
{
  // HAL_GPIO_WritePin(ST7735_RES_GPIO_Port, ST7735_RES_Pin, GPIO_PIN_RESET);
  // GPIO_PinWrite(BOARD_LCD_RES_PORT, BOARD_LCD_RES_PIN, 0U);
  GPIO_ClearPinsOutput(BOARD_LCD_RES_PORT, BOARD_LCD_RES_PIN_MASK);
  HAL_Delay(5);
  // HAL_GPIO_WritePin(ST7735_RES_GPIO_Port, ST7735_RES_Pin, GPIO_PIN_SET);
  // GPIO_PinWrite(BOARD_LCD_RES_PORT, BOARD_LCD_RES_PIN, 1U);
  GPIO_SetPinsOutput(BOARD_LCD_RES_PORT, BOARD_LCD_RES_PIN_MASK);
}

static void ST7735_WriteCommand(uint8_t cmd)
{
  // HAL_GPIO_WritePin(ST7735_DC_GPIO_Port, ST7735_DC_Pin, GPIO_PIN_RESET);
  // GPIO_PinWrite(BOARD_LCD_DC_PORT, BOARD_LCD_DC_PIN, 0U);
  GPIO_ClearPinsOutput(BOARD_LCD_DC_PORT, BOARD_LCD_DC_PIN_MASK);
  // HAL_SPI_Transmit(&ST7735_SPI_PORT, &cmd, sizeof(cmd), HAL_MAX_DELAY);
  uint8_t masterTxData = cmd;
  masterXfer.txData = &masterTxData;
  masterXfer.rxData = NULL;
  masterXfer.dataSize = 1;
  masterXfer.configFlags =
      LCD_LPSPI_MASTER_PCS_FOR_TRANSFER | kLPSPI_MasterPcsContinuous | kLPSPI_MasterByteSwap;
  LPSPI_MasterTransferBlocking(LCD_LPSPI_MASTER_BASEADDR, &masterXfer);
}

static void ST7735_WriteData(uint8_t *buff, size_t buff_size)
{
  // HAL_GPIO_WritePin(ST7735_DC_GPIO_Port, ST7735_DC_Pin, GPIO_PIN_SET);
  // GPIO_PinWrite(BOARD_LCD_DC_PORT, BOARD_LCD_DC_PIN, 1U);
  GPIO_SetPinsOutput(BOARD_LCD_DC_PORT, BOARD_LCD_DC_PIN_MASK);
  // HAL_SPI_Transmit(&ST7735_SPI_PORT, buff, buff_size, HAL_MAX_DELAY);
  // uint8_t masterTxData;
  masterXfer.txData = buff;
  masterXfer.rxData = NULL;
  masterXfer.dataSize = buff_size;
  masterXfer.configFlags =
      LCD_LPSPI_MASTER_PCS_FOR_TRANSFER | kLPSPI_MasterPcsContinuous | kLPSPI_MasterByteSwap;

  LPSPI_MasterTransferBlocking(LCD_LPSPI_MASTER_BASEADDR, &masterXfer);
  //	SPI2_WriteByte();
  //	User_SPI_Transmit_8Bit(&ST7735_SPI_PORT, buff, buff_size, HAL_MAX_DELAY);
}

static void ST7735_ExecuteCommandList(const uint8_t *addr)
{
  uint8_t numCommands, numArgs;
  uint16_t ms;

  numCommands = *addr++;
  while (numCommands--)
  {
    uint8_t cmd = *addr++;
    ST7735_WriteCommand(cmd);

    numArgs = *addr++;
    // If high bit set, delay follows args
    ms = numArgs & DELAY;
    numArgs &= ~DELAY;
    if (numArgs)
    {
      ST7735_WriteData((uint8_t *)addr, numArgs);
      addr += numArgs;
    }

    if (ms)
    {
      ms = *addr++;
      if (ms == 255)
        ms = 500;
      HAL_Delay(ms);
    }
  }
}

static void ST7735_SetAddressWindow(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1)
{
  // column address set
  ST7735_WriteCommand(ST7735_CASET);
  uint8_t data[] = {0x00, x0 + ST7735_XSTART, 0x00, x1 + ST7735_XSTART};
  ST7735_WriteData(data, sizeof(data));

  // row address set
  ST7735_WriteCommand(ST7735_RASET);
  data[1] = y0 + ST7735_YSTART;
  data[3] = y1 + ST7735_YSTART;
  ST7735_WriteData(data, sizeof(data));

  // write to RAM
  ST7735_WriteCommand(ST7735_RAMWR);
}
static int32_t ST7735_SetCursor(uint32_t Xpos, uint32_t Ypos)
{
  Xpos += ST7735_XSTART;
  Ypos += ST7735_YSTART;
  // column address set
  ST7735_WriteCommand(ST7735_CASET);
  uint8_t data[] = {Xpos >> 8, Xpos & 0xFFU};
  ST7735_WriteData(data, sizeof(data));

  // row address set
  ST7735_WriteCommand(ST7735_RASET);
  data[0] = Ypos >> 8;
  data[1] = Ypos & 0xFFU;
  ST7735_WriteData(data, sizeof(data));

  // write to RAM
  ST7735_WriteCommand(ST7735_RAMWR);

  return 0;
}
void ST7735_Init()
{
  ST7735_Select();
  ST7735_Reset();
  ST7735_ExecuteCommandList(init_cmds1);
  ST7735_ExecuteCommandList(init_cmds2);
  ST7735_ExecuteCommandList(init_cmds3);
  ST7735_Unselect();
}

void ST7735_DrawPixel(uint16_t x, uint16_t y, uint16_t color)
{
  if ((x >= ST7735_WIDTH) || (y >= ST7735_HEIGHT))
    return;

  ST7735_Select();

  ST7735_SetAddressWindow(x, y, x + 1, y + 1);
  uint8_t data[] = {color >> 8, color & 0xFF};
  ST7735_WriteData(data, sizeof(data));

  ST7735_Unselect();
}
#define ABS(x) ((x) > 0 ? (x) : -(x))
void ST7735_DrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color)
{
  uint16_t swap;
  uint16_t steep = ABS(y1 - y0) > ABS(x1 - x0);
  if (steep)
  {
    swap = x0;
    x0 = y0;
    y0 = swap;

    swap = x1;
    x1 = y1;
    y1 = swap;
    //_swap_int16_t(x0, y0);
    //_swap_int16_t(x1, y1);
  }

  if (x0 > x1)
  {
    swap = x0;
    x0 = x1;
    x1 = swap;

    swap = y0;
    y0 = y1;
    y1 = swap;
    //_swap_int16_t(x0, x1);
    //_swap_int16_t(y0, y1);
  }

  int16_t dx, dy;
  dx = x1 - x0;
  dy = ABS(y1 - y0);

  int16_t err = dx / 2;
  int16_t ystep;

  if (y0 < y1)
  {
    ystep = 1;
  }
  else
  {
    ystep = -1;
  }

  for (; x0 <= x1; x0++)
  {
    if (steep)
    {
      ST7735_DrawPixel(y0, x0, color);
    }
    else
    {
      ST7735_DrawPixel(x0, y0, color);
    }
    err -= dy;
    if (err < 0)
    {
      y0 += ystep;
      err += dx;
    }
  }
}
void ST7735_DrawRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
  ST7735_DrawLine(x1, y1, x2, y1, color);
  ST7735_DrawLine(x1, y1, x1, y2, color);
  ST7735_DrawLine(x1, y2, x2, y2, color);
  ST7735_DrawLine(x2, y1, x2, y2, color);
}
void ST7735_DrawFilledCircle(uint16_t x0, uint16_t y0, uint8_t r, uint16_t color)
{
  int16_t f = 1 - r;
  int16_t ddF_x = 1;
  int16_t ddF_y = -2 * r;
  int16_t x = 0;
  int16_t y = r;

  ST7735_DrawPixel(x0, y0 + r, color);
  ST7735_DrawPixel(x0, y0 - r, color);
  ST7735_DrawPixel(x0 + r, y0, color);
  ST7735_DrawPixel(x0 - r, y0, color);
  ST7735_DrawLine(x0 - r, y0, x0 + r, y0, color);

  while (x < y)
  {
    if (f >= 0)
    {
      y--;
      ddF_y += 2;
      f += ddF_y;
    }
    x++;
    ddF_x += 2;
    f += ddF_x;

    ST7735_DrawLine(x0 - x, y0 + y, x0 + x, y0 + y, color);
    ST7735_DrawLine(x0 + x, y0 - y, x0 - x, y0 - y, color);

    ST7735_DrawLine(x0 + y, y0 + x, x0 - y, y0 + x, color);
    ST7735_DrawLine(x0 + y, y0 - x, x0 - y, y0 - x, color);
  }
}

void LCD_WriteData_16bit(uint16_t lcd_data)
{
  //	hspi1.Instance->DR = lcd_data>>8;								// �������ݣ���8λ
  //	while( (hspi1.Instance->SR & 0x0002) == 0);		// �ȴ����ͻ��������
  //	hspi1.Instance->DR = lcd_data;									// �������ݣ���8λ
  //	while( (hspi1.Instance->SR & 0x0002) == 0);		// �ȴ����ͻ��������
}
void ST7735_DrawPixel666(uint16_t x, uint16_t y, uint16_t color1, uint16_t color2, uint16_t color3)
{
  //		if ((x < 0) || (x >= ST7735_WIDTH) ||
  //		 (y < 0) || (y >= ST7735_HEIGHT))	return;
  //	uint16_t color= (((color1&0xff) >>3) <<11) | (((color2&0xff)>>2)<<5) | ((color3&0xff) >>3);
  //	ST7735_Select();
  //	ST7735_SetAddressWindow(x, y, x, y);
  ////	uint8_t data[] = {color >> 8, color & 0xFF};
  ////
  ////	ST7735_WriteData(data, sizeof(data));
  //
  //	 HAL_GPIO_WritePin(ST7735_DC_GPIO_Port, ST7735_DC_Pin, GPIO_PIN_SET);
  //	LCD_WriteData_16bit(color);
  //	while( (hspi1.Instance->SR & 0x0080) != RESET);	//	�ȴ�ͨ�����
  ////    HAL_SPI_Transmit(&ST7735_SPI_PORT, buff, buff_size, HAL_MAX_DELAY);
  ////		SPI2_WriteByte(data[0]);
  ////		SPI2_WriteByte(data[1]);
  ////	User_SPI_Transmit_8Bit(&ST7735_SPI_PORT, buff, buff_size, HAL_MAX_DELAY);
  //
  //	ST7735_Unselect();
}
void LCD_CopyBuffer(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t *DataBuff)
{

  // 	ST7735_Select();
  // 	ST7735_SetAddressWindow(x,y,x+width-1,y+height-1);
  // 	ST7735_Unselect();
  // 	hspi1.Init.DataSize 	= SPI_DATASIZE_16BIT;   //	16λ���ݿ���
  //    HAL_SPI_Init(&hspi1);

  // 	ST7735_Select();
  // 	HAL_GPIO_WritePin(ST7735_DC_GPIO_Port, ST7735_DC_Pin, GPIO_PIN_SET);     // ����ָ��ѡ�� ��������ߵ�ƽ���������δ��� ����

  // //	LCD_SPI_TransmitBuffer(&ST7735_SPI_PORT, DataBuff,width * height) ;

  // 	HAL_SPI_Transmit(&ST7735_SPI_PORT, (uint8_t *)DataBuff, width* height, 1000) ;

  // 	ST7735_Unselect();
  // 	hspi1.Init.DataSize 	= SPI_DATASIZE_8BIT;   //	16λ���ݿ���
  //    HAL_SPI_Init(&hspi1);
}

static void ST7735_WriteChar(uint16_t x, uint16_t y, char ch, FontDef font, uint16_t color, uint16_t bgcolor)
{
  uint32_t i, b, j;

  ST7735_SetAddressWindow(x, y, x + font.width - 1, y + font.height - 1);

  for (i = 0; i < font.height; i++)
  {
    b = font.data[(ch - 32) * font.height + i];
    for (j = 0; j < font.width; j++)
    {
      if ((b << j) & 0x8000)
      {
        uint8_t data[] = {color >> 8, color & 0xFF};
        ST7735_WriteData(data, sizeof(data));
      }
      else
      {
        uint8_t data[] = {bgcolor >> 8, bgcolor & 0xFF};
        ST7735_WriteData(data, sizeof(data));
      }
    }
  }
}

/*
Simpler (and probably slower) implementation:

static void ST7735_WriteChar(uint16_t x, uint16_t y, char ch, FontDef font, uint16_t color) {
    uint32_t i, b, j;

    for(i = 0; i < font.height; i++) {
        b = font.data[(ch - 32) * font.height + i];
        for(j = 0; j < font.width; j++) {
            if((b << j) & 0x8000)  {
                ST7735_DrawPixel(x + j, y + i, color);
            }
        }
    }
}
*/

void ST7735_WriteString(uint16_t x, uint16_t y, const char *str, FontDef font, uint16_t color, uint16_t bgcolor)
{
  ST7735_Select();

  while (*str)
  {
    if (x + font.width >= ST7735_WIDTH)
    {
      x = 0;
      y += font.height;
      if (y + font.height >= ST7735_HEIGHT)
      {
        break;
      }

      if (*str == ' ')
      {
        // skip spaces in the beginning of the new line
        str++;
        continue;
      }
    }

    ST7735_WriteChar(x, y, *str, font, color, bgcolor);
    x += font.width;
    str++;
  }

  ST7735_Unselect();
}
static void ST7735_WriteChar_lucency(uint16_t x, uint16_t y, char ch, FontDef font, uint16_t color)
{
  uint32_t i, b, j;

  ST7735_SetAddressWindow(x, y, x + font.width - 1, y + font.height - 1);

  for (i = 0; i < font.height; i++)
  {
    b = font.data[(ch - 32) * font.height + i];
    for (j = 0; j < font.width; j++)
    {
      if ((b << j) & 0x8000)
      {
        uint8_t data[] = {color >> 8, color & 0xFF};
        ST7735_WriteData(data, sizeof(data));
      }
      // else {
      //     uint8_t data[] = { bgcolor >> 8, bgcolor & 0xFF };
      //     ST7735_WriteData(data, sizeof(data));
      // }
    }
  }
}
void ST7735_WriteString_lucency(uint16_t x, uint16_t y, const char *str, FontDef font, uint16_t color)
{
  ST7735_Select();

  while (*str)
  {
    if (x + font.width >= ST7735_WIDTH)
    {
      x = 0;
      y += font.height;
      if (y + font.height >= ST7735_HEIGHT)
      {
        break;
      }

      if (*str == ' ')
      {
        // skip spaces in the beginning of the new line
        str++;
        continue;
      }
    }

    ST7735_WriteChar_lucency(x, y, *str, font, color);
    x += font.width;
    str++;
  }

  ST7735_Unselect();
}
void ST7735_FillRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
  // clipping
  if ((x >= ST7735_WIDTH) || (y >= ST7735_HEIGHT))
    return;
  if ((x + w - 1) >= ST7735_WIDTH)
    w = ST7735_WIDTH - x;
  if ((y + h - 1) >= ST7735_HEIGHT)
    h = ST7735_HEIGHT - y;

  // ST7735_Select();
  ST7735_SetAddressWindow(x, y, x + w - 1, y + h - 1);

  GPIO_PinWrite(BOARD_LCD_DC_PORT, BOARD_LCD_DC_PIN, 1U);
  uint8_t data[] = {color >> 8, color & 0xFF};
  uint8_t masterTxData;
  // HAL_GPIO_WritePin(ST7735_DC_GPIO_Port, ST7735_DC_Pin, GPIO_PIN_SET);

  masterXfer.txData = data;
  masterXfer.rxData = NULL;
  masterXfer.dataSize = 2;
  masterXfer.configFlags =
      LCD_LPSPI_MASTER_PCS_FOR_TRANSFER | kLPSPI_MasterPcsContinuous | kLPSPI_MasterByteSwap;
  for (y = h; y > 0; y--)
  {
    for (x = w; x > 0; x--)
    {
      // HAL_SPI_Transmit(&ST7735_SPI_PORT, data, sizeof(data), HAL_MAX_DELAY);

      LPSPI_MasterTransferBlocking(LCD_LPSPI_MASTER_BASEADDR, &masterXfer);
    }
  }

  ST7735_Unselect();
}
int32_t st7735_send_data(uint8_t *pdata, uint32_t length)
{
  GPIO_PinWrite(BOARD_LCD_DC_PORT, BOARD_LCD_DC_PIN, 1U);
  uint8_t masterTxData;
  // HAL_GPIO_WritePin(ST7735_DC_GPIO_Port, ST7735_DC_Pin, GPIO_PIN_SET);

  masterXfer.txData = pdata;
  masterXfer.rxData = NULL;
  masterXfer.dataSize = length;
  masterXfer.configFlags =
      LCD_LPSPI_MASTER_PCS_FOR_TRANSFER | kLPSPI_MasterPcsContinuous | kLPSPI_MasterByteSwap;

  LPSPI_MasterTransferBlocking(LCD_LPSPI_MASTER_BASEADDR, &masterXfer);

  return 0;
}
int32_t ST7735_FillRGBRect(uint32_t Xpos, uint32_t Ypos, uint8_t *pData, uint32_t Width, uint32_t Height)
{
  int32_t ret = 0;
  static uint8_t pdata[320];
  uint8_t *rgb_data = pData;
  uint32_t i, j;

  if (((Xpos + Width) > ST7735_WIDTH) || ((Ypos + Height) > ST7735_HEIGHT))
  {
    ret = 1;
  } /* Set Cursor */
  else
  {
    for (j = 0; j < Height; j++)
    {

      //      if(ST7735_SetCursor( Xpos, Ypos+j) != 0)
      //      {
      //        ret = 1;
      //      }
      //      else
      {
        ST7735_SetAddressWindow(Xpos, Ypos + j, Xpos + 160, Ypos + j + 1);
        for (i = 0; i < Width; i++)
        {
          pdata[2U * i + 1U] = (uint8_t)(*(rgb_data));
          pdata[(2U * i)] = (uint8_t)(*(rgb_data + 1));
          rgb_data += 2;
        }
        if (st7735_send_data((uint8_t *)&pdata[0], 2U * Width) != 0)
        {
          ret = 1;
        }
      }
    }
  }

  return ret;
}
void ST7735_FillRectangleFast(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
  // clipping
  //    if((x >= ST7735_WIDTH) || (y >= ST7735_HEIGHT)) return;
  //    if((x + w - 1) >= ST7735_WIDTH) w = ST7735_WIDTH - x;
  //    if((y + h - 1) >= ST7735_HEIGHT) h = ST7735_HEIGHT - y;

  //    ST7735_Select();
  //    ST7735_SetAddressWindow(x, y, x+w-1, y+h-1);

  //    // Prepare whole line in a single buffer
  //    uint8_t pixel[] = { color >> 8, color & 0xFF };
  //    uint8_t *line = malloc(w * sizeof(pixel));
  //    for(x = 0; x < w; ++x)
  //    	memcpy(line + x * sizeof(pixel), pixel, sizeof(pixel));

  //    HAL_GPIO_WritePin(ST7735_DC_GPIO_Port, ST7735_DC_Pin, GPIO_PIN_SET);
  //    for(y = h; y > 0; y--)
  //        HAL_SPI_Transmit(&ST7735_SPI_PORT, line, w * sizeof(pixel), HAL_MAX_DELAY);

  //    free(line);
  //    ST7735_Unselect();
}

void ST7735_FillScreen(uint16_t color)
{
  ST7735_FillRectangle(0, 0, ST7735_WIDTH, ST7735_HEIGHT, color);

}

void ST7735_FillScreenFast(uint16_t color)
{
  ST7735_FillRectangleFast(0, 0, ST7735_WIDTH, ST7735_HEIGHT, color);
}

void ST7735_DrawImage(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t *data)
{
  if ((x >= ST7735_WIDTH) || (y >= ST7735_HEIGHT))
    return;
  if ((x + w - 1) >= ST7735_WIDTH)
    return;
  if ((y + h - 1) >= ST7735_HEIGHT)
    return;

  ST7735_Select();
  ST7735_SetAddressWindow(x, y, x + w - 1, y + h - 1);
  ST7735_WriteData((uint8_t *)data, sizeof(uint16_t) * w * h);
  ST7735_Unselect();
}

void ST7735_InvertColors(bool invert)
{
  ST7735_Select();
  ST7735_WriteCommand(invert ? ST7735_INVON : ST7735_INVOFF);
  ST7735_Unselect();
}

void ST7735_SetGamma(uint8_t gamma)
{
  ST7735_Select();
  ST7735_WriteCommand(ST7735_GAMSET);
  ST7735_WriteData(&gamma, sizeof(gamma));
  ST7735_Unselect();
}
