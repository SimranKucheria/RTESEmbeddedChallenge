#include "mbed.h"
#include "LCD_DISCO_F429ZI.h"
#include <string>
#include "stm32f429i_discovery_ts.h"
#include "rtos/ThisThread.h"
#include "rtos/Mutex.h"

// Global objects and variables
LCD_DISCO_F429ZI lcd;
Mutex lcd_mutex;

// Shared variables
std::string ui_background_color = "BLUE";
std::string header = "Menu";
volatile int user_profiles = 0;
volatile int current_user = 0;

void DisplayLoop() {
    while (true) {
        lcd_mutex.lock();
        
        // Set background color
        if (ui_background_color == "BLUE") {
            lcd.Clear(LCD_COLOR_BLUE);
            lcd.SetBackColor(LCD_COLOR_BLUE);
        } else if (ui_background_color == "GREEN") {
            lcd.Clear(LCD_COLOR_GREEN);
            lcd.SetBackColor(LCD_COLOR_GREEN);
        } else {
            lcd.Clear(LCD_COLOR_RED);
            lcd.SetBackColor(LCD_COLOR_RED);
        }

        lcd.SetTextColor(LCD_COLOR_WHITE);
        lcd.DisplayStringAt(0, LINE(1), (uint8_t *)header.c_str(), CENTER_MODE);

        if (header == "Menu") {
            lcd.DisplayStringAt(0, LINE(5), (uint8_t *)"CREATE USER", CENTER_MODE);
            lcd.DisplayStringAt(0, LINE(10), (uint8_t *)"USER PROFILES", CENTER_MODE);
            lcd.DisplayStringAt(0, LINE(15), (uint8_t *)"DELETE USER", CENTER_MODE);
            
            // Draw horizontal lines
            lcd.DrawHLine(0, 0, lcd.GetXSize());
            lcd.DrawHLine(0, lcd.GetYSize() - 1, lcd.GetXSize());
            lcd.DrawHLine(0, 40, lcd.GetXSize());
            lcd.DrawHLine(0, 290, lcd.GetXSize());
        }
        else if (header == "Create User") {
            if (user_profiles >= 4) {
                lcd.DisplayStringAt(0, LINE(10), (uint8_t *)"CANNOT CREATE MORE", CENTER_MODE);
                lcd.DisplayStringAt(0, LINE(12), (uint8_t *)"PLEASE DELETE", CENTER_MODE);
                lcd.DisplayStringAt(0, LINE(15), (uint8_t *)"BACK TO MENU", CENTER_MODE);
            }
            else {
                std::string message = "CREATED USER " + std::to_string(user_profiles + 1);
                lcd.DisplayStringAt(0, LINE(10), (uint8_t *)message.c_str(), CENTER_MODE);
                user_profiles++;
                lcd.DisplayStringAt(0, LINE(15), (uint8_t *)"BACK TO MENU", CENTER_MODE);
                ThisThread::sleep_for(2000ms);
                header = "Menu";
            }
        }
        else if (header == "Delete User") {
            if (user_profiles <= 0) {
                lcd.DisplayStringAt(0, LINE(10), (uint8_t *)"NO USERS TO DELETE", CENTER_MODE);
                lcd.DisplayStringAt(0, LINE(15), (uint8_t *)"BACK TO MENU", CENTER_MODE);
            }
            else {
                std::string message = "DELETED USER " + std::to_string(user_profiles);
                lcd.DisplayStringAt(0, LINE(10), (uint8_t *)message.c_str(), CENTER_MODE);
                user_profiles--;
                lcd.DisplayStringAt(0, LINE(15), (uint8_t *)"BACK TO MENU", CENTER_MODE);
                ThisThread::sleep_for(2000ms);
                header = "Menu";
            }
        }
        else if (header == "User Profiles") {
            if (user_profiles == 0) {
                lcd.DisplayStringAt(0, LINE(10), (uint8_t *)"PLEASE ADD USER", CENTER_MODE);
                ThisThread::sleep_for(2000ms);
                header = "Menu";
            }
            else {
                for (int8_t i = 1; i <= user_profiles; i++) {
                    std::string profile = "USER PROFILE " + std::to_string(i);
                    lcd.DisplayStringAt(0, LINE((i * 3) + 1), (uint8_t *)profile.c_str(), CENTER_MODE);
                }
                
                // Add Back to Menu option
                lcd.DisplayStringAt(0, LINE(15), (uint8_t *)"BACK TO MENU", CENTER_MODE);
                
                lcd.DrawHLine(0, 0, lcd.GetXSize());
                lcd.DrawHLine(0, lcd.GetYSize() - 1, lcd.GetXSize());
                lcd.DrawHLine(0, 40, lcd.GetXSize());
                lcd.DrawHLine(0, 290, lcd.GetXSize());
                lcd.SetTextColor(LCD_COLOR_ORANGE);
            }
        }
        
        lcd_mutex.unlock();
        ThisThread::sleep_for(50ms);
    }
}

int8_t getUser(int32_t y) {
    if (y < 260 && y >= 210) return 4;
    if (y < 210 && y >= 165) return 3;
    if (y < 165 && y >= 135) return 2;
    if (y < 135 && y >= 80) return 1;
    if (y < 80 && y >= 30) return 0;
    return -1;
}

void DynamicLoop() {
    uint8_t status = BSP_TS_Init(lcd.GetXSize(), lcd.GetYSize());
    
    if (status != TS_OK) {
        lcd_mutex.lock();
        lcd.SetBackColor(LCD_COLOR_WHITE);
        lcd.SetTextColor(LCD_COLOR_RED);
        lcd.DisplayStringAt(0, lcd.GetYSize() - 95, (uint8_t *)"ERROR", CENTER_MODE);
        lcd.DisplayStringAt(0, lcd.GetYSize() - 80, (uint8_t *)"Touchscreen cannot be initialized", CENTER_MODE);
        lcd_mutex.unlock();
        return;
    }

    TS_StateTypeDef ts_state;
    while (true) {
        BSP_TS_GetState(&ts_state);
        if (ts_state.TouchDetected) {
            lcd_mutex.lock();
            
            if (header == "Menu") {
                if (ts_state.Y < LINE(7) && ts_state.Y >= LINE(3)) {
                    header = "Delete User";
                } else if (ts_state.Y < LINE(12) && ts_state.Y >= LINE(8)) {
                    header = "User Profiles";
                } else if (ts_state.Y < LINE(17) && ts_state.Y >= LINE(13)) {
                    header = "Create User";
                }
            }
            else if (header == "User Profiles" || header == "Create User" || header == "Delete User") {
                if (ts_state.Y >= LINE(13) && ts_state.Y < LINE(17)) {
                    header = "Menu";
                }
            }
            
            lcd_mutex.unlock();
        }
        ThisThread::sleep_for(50ms);
    }
}

int main() {
    Thread display_thread;
    Thread dynamic_thread;
    
    display_thread.start(callback(DisplayLoop));
    dynamic_thread.start(callback(DynamicLoop));
    
    while (true) {
        ThisThread::sleep_for(1000ms);
    }
}