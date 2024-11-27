#include "mbed.h"
#include "LCD_DISCO_F429ZI.h"
#include "iostream"

LCD_DISCO_F429ZI lcd;

int main()
{
lcd.Clear(LCD_COLOR_BLUE);
lcd.SetBackColor(LCD_COLOR_BLUE);
lcd.SetTextColor(LCD_COLOR_WHITE);
lcd.DisplayStringAt(0, LINE(5), (uint8_t *)"HELLO", CENTER_MODE);
while(1)
{
}
}

