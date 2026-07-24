/**
 ******************************************************************************
 * @file    OLED.c
 * @brief   0.96寸 OLED 显示驱动 (SSD1306, I2C2)
 * @note    移植自江协科技 V2.0 — 原软件模拟 I2C 替换为 HAL I2C2
 *          地址 0x78, 分辨率 128x64, 显存大小 128x64/8 = 1024 字节
 ******************************************************************************
 */

#include "OLED.h"
#include "i2c.h"
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdarg.h>

/*============================================================================*/
/* I2C 地址 (7-bit 0x3C 左移1位 = 0x78)                                        */
/*============================================================================*/
#define OLED_I2C_ADDR    0x78

/*============================================================================*/
/* 全局显存 — 所有绘图操作在显存中进行，调用 OLED_Update 后刷新到屏幕              */
/*============================================================================*/
uint8_t OLED_DisplayBuf[8][128];

/*============================================================================*/
/* I2C 底层通信（HAL 替代原软件模拟 I2C）                                        */
/*============================================================================*/

/**
 * @brief  写命令到 OLED
 * @param  Command 命令字节
 * @note   格式: Start + 0x78 + 0x00(控制字节=命令) + Command + Stop
 */
static void OLED_WriteCommand(uint8_t Command)
{
    uint8_t buf[2] = {0x00, Command};
    HAL_I2C_Master_Transmit(&hi2c2, OLED_I2C_ADDR, buf, 2, 100);
}

/**
 * @brief  写数据到 OLED
 * @param  Data   数据指针
 * @param  Count  数据字节数 (最大 128)
 * @note   格式: Start + 0x78 + 0x40(控制字节=数据) + Data[0..Count-1] + Stop
 */
static void OLED_WriteData(uint8_t *Data, uint8_t Count)
{
    uint8_t buf[129];  /* 控制字节 + 最多 128 字节数据 */
    buf[0] = 0x40;
    for (uint8_t i = 0; i < Count; i++)
        buf[i + 1] = Data[i];
    HAL_I2C_Master_Transmit(&hi2c2, OLED_I2C_ADDR, buf, Count + 1, 100);
}

/*============================================================================*/
/* 硬件层: 光标定位                                                              */
/*============================================================================*/

/**
 * @brief  设置 OLED 显示位置 (页地址 + 列地址)
 * @param  Page  页号 (0~7), 每页 8 像素高
 * @param  X     列坐标 (0~127)
 */
static void OLED_SetCursor(uint8_t Page, uint8_t X)
{
    OLED_WriteCommand(0xB0 | Page);              /* 页地址 */
    OLED_WriteCommand(0x10 | ((X & 0xF0) >> 4)); /* 列高 4 位 */
    OLED_WriteCommand(0x00 | (X & 0x0F));        /* 列低 4 位 */
}

/*============================================================================*/
/* 初始化                                                                      */
/*============================================================================*/

void OLED_Init(void)
{
    /* I2C2 由 CubeMX 配置，此处只需等待 OLED 上电稳定 */
    HAL_Delay(100);

    /* SSD1306 初始化序列 */
    OLED_WriteCommand(0xAE);  /* 关闭显示 */

    OLED_WriteCommand(0xD5);  /* 显示时钟分频 / 振荡频率 */
    OLED_WriteCommand(0x80);  /* 默认值 */

    OLED_WriteCommand(0xA8);  /* 多路复用比 */
    OLED_WriteCommand(0x3F);  /* 64 行 */

    OLED_WriteCommand(0xD3);  /* 显示偏移 */
    OLED_WriteCommand(0x00);

    OLED_WriteCommand(0x40);  /* 显示起始行 (0x40~0x7F) */

    OLED_WriteCommand(0xA1);  /* 左右方向: 0xA1=正常(左→右), 0xA0=反转 */

    OLED_WriteCommand(0xC8);  /* 上下方向: 0xC8=正常(上→下), 0xC0=反转 */

    OLED_WriteCommand(0xDA);  /* COM 引脚硬件配置 */
    OLED_WriteCommand(0x12);

    OLED_WriteCommand(0x81);  /* 对比度 */
    OLED_WriteCommand(0xCF);  /* 0x00~0xFF, 默认 0x7F */

    OLED_WriteCommand(0xD9);  /* 预充电周期 */
    OLED_WriteCommand(0xF1);

    OLED_WriteCommand(0xDB);  /* VCOMH 电压 */
    OLED_WriteCommand(0x30);

    OLED_WriteCommand(0xA4);  /* 全局显示: 0xA4=正常, 0xA5=全亮 */

    OLED_WriteCommand(0xA6);  /* 显示模式: 0xA6=正常(白底黑字), 0xA7=反转 */

    OLED_WriteCommand(0x8D);  /* 电荷泵 */
    OLED_WriteCommand(0x14);  /* 使能 */

    OLED_WriteCommand(0xAF);  /* 打开显示 */

    OLED_Clear();
    OLED_Update();
}

/*============================================================================*/
/* 更新显存到 OLED                                                               */
/*============================================================================*/

void OLED_Update(void)
{
    for (uint8_t j = 0; j < 8; j++)
    {
        OLED_SetCursor(j, 0);
        OLED_WriteData(OLED_DisplayBuf[j], 128);
    }
}

void OLED_UpdateArea(int16_t X, int16_t Y, uint8_t Width, uint8_t Height)
{
    int16_t Page, Page1;

    Page  = Y / 8;
    Page1 = (Y + Height - 1) / 8 + 1;
    if (Y < 0) { Page -= 1; Page1 -= 1; }

    for (int16_t j = Page; j < Page1; j++)
    {
        if (X >= 0 && X <= 127 && j >= 0 && j <= 7)
        {
            OLED_SetCursor(j, X);
            OLED_WriteData(&OLED_DisplayBuf[j][X], Width);
        }
    }
}

/*============================================================================*/
/* 显存操作                                                                    */
/*============================================================================*/

void OLED_Clear(void)
{
    memset(OLED_DisplayBuf, 0x00, sizeof(OLED_DisplayBuf));
}

void OLED_ClearArea(int16_t X, int16_t Y, uint8_t Width, uint8_t Height)
{
    for (int16_t j = Y; j < Y + Height; j++)
        for (int16_t i = X; i < X + Width; i++)
            if (i >= 0 && i <= 127 && j >= 0 && j <= 63)
                OLED_DisplayBuf[j / 8][i] &= ~(0x01 << (j % 8));
}

void OLED_Reverse(void)
{
    for (uint8_t j = 0; j < 8; j++)
        for (uint8_t i = 0; i < 128; i++)
            OLED_DisplayBuf[j][i] ^= 0xFF;
}

void OLED_ReverseArea(int16_t X, int16_t Y, uint8_t Width, uint8_t Height)
{
    for (int16_t j = Y; j < Y + Height; j++)
        for (int16_t i = X; i < X + Width; i++)
            if (i >= 0 && i <= 127 && j >= 0 && j <= 63)
                OLED_DisplayBuf[j / 8][i] ^= 0x01 << (j % 8);
}

/*============================================================================*/
/* 点 & 线 & 形状绘图                                                           */
/*============================================================================*/

void OLED_DrawPoint(int16_t X, int16_t Y)
{
    if (X >= 0 && X <= 127 && Y >= 0 && Y <= 63)
        OLED_DisplayBuf[Y / 8][X] |= 0x01 << (Y % 8);
}

uint8_t OLED_GetPoint(int16_t X, int16_t Y)
{
    if (X >= 0 && X <= 127 && Y >= 0 && Y <= 63)
        if (OLED_DisplayBuf[Y / 8][X] & (0x01 << (Y % 8)))
            return 1;
    return 0;
}

void OLED_DrawLine(int16_t X0, int16_t Y0, int16_t X1, int16_t Y1)
{
    int16_t x, y, dx, dy, d, incrE, incrNE, temp;
    int16_t x0 = X0, y0 = Y0, x1 = X1, y1 = Y1;
    uint8_t yflag = 0, xyflag = 0;

    if (y0 == y1)
    {
        if (x0 > x1) { temp = x0; x0 = x1; x1 = temp; }
        for (x = x0; x <= x1; x++) OLED_DrawPoint(x, y0);
        return;
    }
    if (x0 == x1)
    {
        if (y0 > y1) { temp = y0; y0 = y1; y1 = temp; }
        for (y = y0; y <= y1; y++) OLED_DrawPoint(x0, y);
        return;
    }

    /* Bresenham 直线算法 */
    if (x0 > x1) { temp = x0; x0 = x1; x1 = temp; temp = y0; y0 = y1; y1 = temp; }
    if (y0 > y1) { y0 = -y0; y1 = -y1; yflag = 1; }
    if (y1 - y0 > x1 - x0) { temp = x0; x0 = y0; y0 = temp; temp = x1; x1 = y1; y1 = temp; xyflag = 1; }

    dx = x1 - x0; dy = y1 - y0;
    incrE = 2 * dy; incrNE = 2 * (dy - dx);
    d = 2 * dy - dx;
    x = x0; y = y0;

    if      (yflag && xyflag) OLED_DrawPoint(y, -x);
    else if (yflag)            OLED_DrawPoint(x, -y);
    else if (xyflag)           OLED_DrawPoint(y, x);
    else                       OLED_DrawPoint(x, y);

    while (x < x1)
    {
        x++;
        if (d < 0) d += incrE;
        else       { y++; d += incrNE; }
        if      (yflag && xyflag) OLED_DrawPoint(y, -x);
        else if (yflag)            OLED_DrawPoint(x, -y);
        else if (xyflag)           OLED_DrawPoint(y, x);
        else                       OLED_DrawPoint(x, y);
    }
}

void OLED_DrawRectangle(int16_t X, int16_t Y, uint8_t Width, uint8_t Height, uint8_t IsFilled)
{
    if (!IsFilled)
    {
        for (int16_t i = X; i < X + Width; i++)
        {
            OLED_DrawPoint(i, Y);
            OLED_DrawPoint(i, Y + Height - 1);
        }
        for (int16_t i = Y; i < Y + Height; i++)
        {
            OLED_DrawPoint(X, i);
            OLED_DrawPoint(X + Width - 1, i);
        }
    }
    else
    {
        for (int16_t i = X; i < X + Width; i++)
            for (int16_t j = Y; j < Y + Height; j++)
                OLED_DrawPoint(i, j);
    }
}

/* ---- 辅助: 求幂 ---- */
static uint32_t OLED_Pow(uint32_t X, uint32_t Y)
{
    uint32_t Result = 1;
    while (Y--) Result *= X;
    return Result;
}

/* ---- 辅助: 判断点是否在多边形内 ---- */
static uint8_t OLED_pnpoly(uint8_t nvert, int16_t *vertx, int16_t *verty, int16_t testx, int16_t testy)
{
    int16_t i, j, c = 0;
    for (i = 0, j = nvert - 1; i < nvert; j = i++)
    {
        if (((verty[i] > testy) != (verty[j] > testy)) &&
            (testx < (vertx[j] - vertx[i]) * (testy - verty[i]) / (verty[j] - verty[i]) + vertx[i]))
            c = !c;
    }
    return c;
}

void OLED_DrawTriangle(int16_t X0, int16_t Y0, int16_t X1, int16_t Y1,
                       int16_t X2, int16_t Y2, uint8_t IsFilled)
{
    if (!IsFilled)
    {
        OLED_DrawLine(X0, Y0, X1, Y1);
        OLED_DrawLine(X0, Y0, X2, Y2);
        OLED_DrawLine(X1, Y1, X2, Y2);
    }
    else
    {
        int16_t vx[] = {X0, X1, X2};
        int16_t vy[] = {Y0, Y1, Y2};
        int16_t minx = X0, miny = Y0, maxx = X0, maxy = Y0;
        if (X1 < minx) minx = X1; if (X2 < minx) minx = X2;
        if (Y1 < miny) miny = Y1; if (Y2 < miny) miny = Y2;
        if (X1 > maxx) maxx = X1; if (X2 > maxx) maxx = X2;
        if (Y1 > maxy) maxy = Y1; if (Y2 > maxy) maxy = Y2;
        for (int16_t i = minx; i <= maxx; i++)
            for (int16_t j = miny; j <= maxy; j++)
                if (OLED_pnpoly(3, vx, vy, i, j)) OLED_DrawPoint(i, j);
    }
}

void OLED_DrawCircle(int16_t X, int16_t Y, uint8_t Radius, uint8_t IsFilled)
{
    int16_t x, y, d, j;
    d = 1 - Radius; x = 0; y = Radius;

    OLED_DrawPoint(X + x, Y + y); OLED_DrawPoint(X - x, Y - y);
    OLED_DrawPoint(X + y, Y + x); OLED_DrawPoint(X - y, Y - x);

    if (IsFilled)
        for (j = -y; j < y; j++)
            OLED_DrawPoint(X, Y + j);

    while (x < y)
    {
        x++;
        if (d < 0) d += 2 * x + 1;
        else       { y--; d += 2 * (x - y) + 1; }

        OLED_DrawPoint(X + x, Y + y); OLED_DrawPoint(X + y, Y + x);
        OLED_DrawPoint(X - x, Y - y); OLED_DrawPoint(X - y, Y - x);
        OLED_DrawPoint(X + x, Y - y); OLED_DrawPoint(X + y, Y - x);
        OLED_DrawPoint(X - x, Y + y); OLED_DrawPoint(X - y, Y + x);

        if (IsFilled)
        {
            for (j = -y; j < y; j++) { OLED_DrawPoint(X + x, Y + j); OLED_DrawPoint(X - x, Y + j); }
            for (j = -x; j < x; j++) { OLED_DrawPoint(X - y, Y + j); OLED_DrawPoint(X + y, Y + j); }
        }
    }
}

void OLED_DrawEllipse(int16_t X, int16_t Y, uint8_t A, uint8_t B, uint8_t IsFilled)
{
    int16_t x, y, j, a = A, b = B;
    float d1, d2;

    x = 0; y = b;
    d1 = b * b + a * a * (-b + 0.5f);

    if (IsFilled)
        for (j = -y; j < y; j++)
            OLED_DrawPoint(X, Y + j);

    OLED_DrawPoint(X + x, Y + y); OLED_DrawPoint(X - x, Y - y);
    OLED_DrawPoint(X - x, Y + y); OLED_DrawPoint(X + x, Y - y);

    while (b * b * (x + 1) < a * a * (y - 0.5f))
    {
        if (d1 <= 0) d1 += b * b * (2 * x + 3);
        else         { d1 += b * b * (2 * x + 3) + a * a * (-2 * y + 2); y--; }
        x++;

        if (IsFilled)
            for (j = -y; j < y; j++)
            {
                OLED_DrawPoint(X + x, Y + j);
                OLED_DrawPoint(X - x, Y + j);
            }

        OLED_DrawPoint(X + x, Y + y); OLED_DrawPoint(X - x, Y - y);
        OLED_DrawPoint(X - x, Y + y); OLED_DrawPoint(X + x, Y - y);
    }

    d2 = b * b * (x + 0.5f) * (x + 0.5f) + a * a * (y - 1) * (y - 1) - a * a * b * b;
    while (y > 0)
    {
        if (d2 <= 0) { d2 += b * b * (2 * x + 2) + a * a * (-2 * y + 3); x++; }
        else          d2 += a * a * (-2 * y + 3);
        y--;

        if (IsFilled)
            for (j = -y; j < y; j++)
            {
                OLED_DrawPoint(X + x, Y + j);
                OLED_DrawPoint(X - x, Y + j);
            }

        OLED_DrawPoint(X + x, Y + y); OLED_DrawPoint(X - x, Y - y);
        OLED_DrawPoint(X - x, Y + y); OLED_DrawPoint(X + x, Y - y);
    }
}

/* ---- 辅助: 判断点是否在指定角度内 ---- */
static uint8_t OLED_IsInAngle(int16_t X, int16_t Y, int16_t StartAngle, int16_t EndAngle)
{
    int16_t PointAngle = (int16_t)(atan2f((float)Y, (float)X) / 3.1415926f * 180.0f);
    if (StartAngle < EndAngle)
    {
        if (PointAngle >= StartAngle && PointAngle <= EndAngle) return 1;
    }
    else
    {
        if (PointAngle >= StartAngle || PointAngle <= EndAngle) return 1;
    }
    return 0;
}

void OLED_DrawArc(int16_t X, int16_t Y, uint8_t Radius,
                  int16_t StartAngle, int16_t EndAngle, uint8_t IsFilled)
{
    int16_t x, y, d, j;
    d = 1 - Radius; x = 0; y = Radius;

    if (OLED_IsInAngle( x,  y, StartAngle, EndAngle)) OLED_DrawPoint(X + x, Y + y);
    if (OLED_IsInAngle(-x, -y, StartAngle, EndAngle)) OLED_DrawPoint(X - x, Y - y);
    if (OLED_IsInAngle( y,  x, StartAngle, EndAngle)) OLED_DrawPoint(X + y, Y + x);
    if (OLED_IsInAngle(-y, -x, StartAngle, EndAngle)) OLED_DrawPoint(X - y, Y - x);

    if (IsFilled)
        for (j = -y; j < y; j++)
            if (OLED_IsInAngle(0, j, StartAngle, EndAngle))
                OLED_DrawPoint(X, Y + j);

    while (x < y)
    {
        x++;
        if (d < 0) d += 2 * x + 1;
        else       { y--; d += 2 * (x - y) + 1; }

        if (OLED_IsInAngle( x,  y, StartAngle, EndAngle)) OLED_DrawPoint(X + x, Y + y);
        if (OLED_IsInAngle( y,  x, StartAngle, EndAngle)) OLED_DrawPoint(X + y, Y + x);
        if (OLED_IsInAngle(-x, -y, StartAngle, EndAngle)) OLED_DrawPoint(X - x, Y - y);
        if (OLED_IsInAngle(-y, -x, StartAngle, EndAngle)) OLED_DrawPoint(X - y, Y - x);
        if (OLED_IsInAngle( x, -y, StartAngle, EndAngle)) OLED_DrawPoint(X + x, Y - y);
        if (OLED_IsInAngle( y, -x, StartAngle, EndAngle)) OLED_DrawPoint(X + y, Y - x);
        if (OLED_IsInAngle(-x,  y, StartAngle, EndAngle)) OLED_DrawPoint(X - x, Y + y);
        if (OLED_IsInAngle(-y,  x, StartAngle, EndAngle)) OLED_DrawPoint(X - y, Y + x);

        if (IsFilled)
        {
            for (j = -y; j < y; j++)
            {
                if (OLED_IsInAngle( x, j, StartAngle, EndAngle)) OLED_DrawPoint(X + x, Y + j);
                if (OLED_IsInAngle(-x, j, StartAngle, EndAngle)) OLED_DrawPoint(X - x, Y + j);
            }
            for (j = -x; j < x; j++)
            {
                if (OLED_IsInAngle(-y, j, StartAngle, EndAngle)) OLED_DrawPoint(X - y, Y + j);
                if (OLED_IsInAngle( y, j, StartAngle, EndAngle)) OLED_DrawPoint(X + y, Y + j);
            }
        }
    }
}

/*============================================================================*/
/* 文字显示                                                                    */
/*============================================================================*/

void OLED_ShowImage(int16_t X, int16_t Y, uint8_t Width, uint8_t Height, const uint8_t *Image)
{
    OLED_ClearArea(X, Y, Width, Height);

    for (uint8_t j = 0; j < (Height - 1) / 8 + 1; j++)
    {
        for (uint8_t i = 0; i < Width; i++)
        {
            if (X + i >= 0 && X + i <= 127)
            {
                int16_t Page = Y / 8;
                int16_t Shift = Y % 8;
                if (Y < 0) { Page -= 1; Shift += 8; }

                if (Page + j >= 0 && Page + j <= 7)
                    OLED_DisplayBuf[Page + j][X + i] |= Image[j * Width + i] << Shift;

                if (Page + j + 1 >= 0 && Page + j + 1 <= 7)
                    OLED_DisplayBuf[Page + j + 1][X + i] |= Image[j * Width + i] >> (8 - Shift);
            }
        }
    }
}

void OLED_ShowChar(int16_t X, int16_t Y, char Char, uint8_t FontSize)
{
    if (FontSize == OLED_8X16)
        OLED_ShowImage(X, Y, 8, 16, OLED_F8x16[Char - ' ']);
    else if (FontSize == OLED_6X8)
        OLED_ShowImage(X, Y, 6, 8, OLED_F6x8[Char - ' ']);
}

void OLED_ShowString(int16_t X, int16_t Y, char *String, uint8_t FontSize)
{
    uint16_t i = 0;
    char SingleChar[5];
    uint8_t CharLength = 0;
    uint16_t XOffset = 0;

    while (String[i] != '\0')
    {
        /* ---- UTF-8 / GB2312 多字节字符解析 ---- */
#ifdef OLED_CHARSET_UTF8
        if ((String[i] & 0x80) == 0x00)
        {
            CharLength = 1; SingleChar[0] = String[i++]; SingleChar[1] = '\0';
        }
        else if ((String[i] & 0xE0) == 0xC0)
        {
            CharLength = 2; SingleChar[0] = String[i++]; if (String[i]=='\0') break;
            SingleChar[1] = String[i++]; SingleChar[2] = '\0';
        }
        else if ((String[i] & 0xF0) == 0xE0)
        {
            CharLength = 3; SingleChar[0] = String[i++]; if (String[i]=='\0') break;
            SingleChar[1] = String[i++]; if (String[i]=='\0') break;
            SingleChar[2] = String[i++]; SingleChar[3] = '\0';
        }
        else if ((String[i] & 0xF8) == 0xF0)
        {
            CharLength = 4; SingleChar[0] = String[i++]; if (String[i]=='\0') break;
            SingleChar[1] = String[i++]; if (String[i]=='\0') break;
            SingleChar[2] = String[i++]; if (String[i]=='\0') break;
            SingleChar[3] = String[i++]; SingleChar[4] = '\0';
        }
        else { i++; continue; }
#endif

#ifdef OLED_CHARSET_GB2312
        if ((String[i] & 0x80) == 0x00)
        {
            CharLength = 1; SingleChar[0] = String[i++]; SingleChar[1] = '\0';
        }
        else
        {
            CharLength = 2; SingleChar[0] = String[i++]; if (String[i]=='\0') break;
            SingleChar[1] = String[i++]; SingleChar[2] = '\0';
        }
#endif

        if (CharLength == 1)
        {
            OLED_ShowChar(X + XOffset, Y, SingleChar[0], FontSize);
            XOffset += FontSize;
        }
        else
        {
            uint16_t pIndex;
            for (pIndex = 0; strcmp(OLED_CF16x16[pIndex].Index, "") != 0; pIndex++)
                if (strcmp(OLED_CF16x16[pIndex].Index, SingleChar) == 0) break;

            if (FontSize == OLED_8X16)
            {
                OLED_ShowImage(X + XOffset, Y, 16, 16, OLED_CF16x16[pIndex].Data);
                XOffset += 16;
            }
            else if (FontSize == OLED_6X8)
            {
                OLED_ShowChar(X + XOffset, Y, '?', OLED_6X8);
                XOffset += OLED_6X8;
            }
        }
    }
}

void OLED_ShowNum(int16_t X, int16_t Y, uint32_t Number, uint8_t Length, uint8_t FontSize)
{
    for (uint8_t i = 0; i < Length; i++)
        OLED_ShowChar(X + i * FontSize, Y,
                      Number / OLED_Pow(10, Length - i - 1) % 10 + '0', FontSize);
}

void OLED_ShowSignedNum(int16_t X, int16_t Y, int32_t Number, uint8_t Length, uint8_t FontSize)
{
    uint32_t Num;
    if (Number >= 0) { OLED_ShowChar(X, Y, '+', FontSize); Num = (uint32_t)Number; }
    else             { OLED_ShowChar(X, Y, '-', FontSize); Num = (uint32_t)(-Number); }

    for (uint8_t i = 0; i < Length; i++)
        OLED_ShowChar(X + (i + 1) * FontSize, Y,
                      Num / OLED_Pow(10, Length - i - 1) % 10 + '0', FontSize);
}

void OLED_ShowHexNum(int16_t X, int16_t Y, uint32_t Number, uint8_t Length, uint8_t FontSize)
{
    for (uint8_t i = 0; i < Length; i++)
    {
        uint8_t d = Number / OLED_Pow(16, Length - i - 1) % 16;
        OLED_ShowChar(X + i * FontSize, Y, d < 10 ? d + '0' : d - 10 + 'A', FontSize);
    }
}

void OLED_ShowBinNum(int16_t X, int16_t Y, uint32_t Number, uint8_t Length, uint8_t FontSize)
{
    for (uint8_t i = 0; i < Length; i++)
        OLED_ShowChar(X + i * FontSize, Y,
                      Number / OLED_Pow(2, Length - i - 1) % 2 + '0', FontSize);
}

void OLED_ShowFloatNum(int16_t X, int16_t Y, double Number,
                       uint8_t IntLength, uint8_t FraLength, uint8_t FontSize)
{
    uint32_t IntNum, FraNum, PowNum;

    if (Number >= 0) OLED_ShowChar(X, Y, '+', FontSize);
    else             { OLED_ShowChar(X, Y, '-', FontSize); Number = -Number; }

    IntNum = (uint32_t)Number;
    Number -= IntNum;
    PowNum = OLED_Pow(10, FraLength);
    FraNum = (uint32_t)(Number * PowNum + 0.5);
    IntNum += FraNum / PowNum;

    OLED_ShowNum(X + FontSize, Y, IntNum, IntLength, FontSize);
    OLED_ShowChar(X + (IntLength + 1) * FontSize, Y, '.', FontSize);
    OLED_ShowNum(X + (IntLength + 2) * FontSize, Y, FraNum, FraLength, FontSize);
}

void OLED_Printf(int16_t X, int16_t Y, uint8_t FontSize, char *format, ...)
{
    char String[256];
    va_list arg;
    va_start(arg, format);
    vsprintf(String, format, arg);
    va_end(arg);
    OLED_ShowString(X, Y, String, FontSize);
}

