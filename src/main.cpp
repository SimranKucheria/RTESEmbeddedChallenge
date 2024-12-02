#include "mbed.h"
#include "LCD_DISCO_F429ZI.h"
#include "iostream"
#include "stm32f429i_discovery_ts.h"

LCD_DISCO_F429ZI lcd;
TS_StateTypeDef ts_state;

std::string ui_background_color = "BLUE";
std::string header = "Menu";
int user_profiles = 0;
int current_user = 0;
int touched = 0;
//Need an array to hold gestures (Lets limit to 4 dynamic users that can be deleted and added)


void DisplayLoop() {
  while (true) {
      
    if (ui_background_color == "BLUE" ) {
        
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

    if (header == "Menu"){
        lcd.DisplayStringAt(0, LINE(5), (uint8_t *) "CREATE USER", CENTER_MODE);
        lcd.DisplayStringAt(0, LINE(10), (uint8_t *) "USER PROFILES", CENTER_MODE);
        lcd.DisplayStringAt(0, LINE(15), (uint8_t *) "DELETE USER", CENTER_MODE);
       
        
        lcd.DrawHLine(0, 0, lcd.GetXSize());
        lcd.DrawHLine(0, lcd.GetYSize() - 1, lcd.GetXSize());
        lcd.DrawHLine(0, 40, lcd.GetXSize());
        lcd.DrawHLine(0, 290, lcd.GetXSize());

        if (touched){
            lcd.SetTextColor(LCD_COLOR_BLACK);
            lcd.DrawRect(100, 50, 30, 30);  // draw the rectangle to indicate the current user profile
            
            thread_sleep_for(10000);
        }
    }

    else if(header == "Create User"){
        if(user_profiles == 4){
            lcd.DisplayStringAt(0, LINE(10), (uint8_t *) "CANNOT CREATE MORE", CENTER_MODE);
            lcd.DisplayStringAt(0, LINE(10), (uint8_t *) "PLEASE DELETE", CENTER_MODE);
            header = "Menu";
        }
        else{
            user_profiles++;
            lcd.DisplayStringAt(0, LINE(10), (uint8_t *) "CREATED USER" +user_profiles, CENTER_MODE);
            //Should go to GYRO recording mode yaha se
            header = "Menu"; //Exiting for now but need to add gyro recording and the return to menu
        }

    }

    else if(header == "Delete User"){
        if(user_profiles == 0){
            lcd.DisplayStringAt(0, LINE(10), (uint8_t *) "NO USERS TO DELETE", CENTER_MODE);
            header = "Menu";
        }
        else{
            lcd.DisplayStringAt(0, LINE(10), (uint8_t *) "DELETED USER" + user_profiles, CENTER_MODE);
            user_profiles--;
            //Manipulate gestures array
            header = "Menu";
        }
    }

    else if(header == "Record New Action"){
        lcd.DisplayStringAt(0, LINE(30), (uint8_t *) "NEED TO CODE", CENTER_MODE);
       
        
        lcd.DrawHLine(0, 0, lcd.GetXSize());
        lcd.DrawHLine(0, lcd.GetYSize() - 1, lcd.GetXSize());
        lcd.DrawHLine(0, 40, lcd.GetXSize());
        lcd.DrawHLine(0, 290, lcd.GetXSize());

    }
    else{
        if (user_profiles == 0){

            lcd.DisplayStringAt(0, LINE(10), (uint8_t *) "PLEASE ADD USER", CENTER_MODE);
        }
        else{

            for (int8_t i = user_profiles; i >= 1; --i) {  // Display numbers for the 5 user profiles available

            lcd.DisplayStringAt(0, LINE((i * 3) + 1), (uint8_t *) user_profiles, CENTER_MODE);
            --user_profiles;

            }
            
            lcd.DrawHLine(0, 0, lcd.GetXSize());
            lcd.DrawHLine(0, lcd.GetYSize() - 1, lcd.GetXSize());
            lcd.DrawHLine(0, 40, lcd.GetXSize());
            lcd.DrawHLine(0, 290, lcd.GetXSize());

            lcd.SetTextColor(LCD_COLOR_ORANGE);
            }
        }
    thread_sleep_for(50);

  }

}

int8_t getUser(int32_t y) {

  if (y < 260 && y >= 210) return 4;
  else if (y < 210 && y >= 165) return 3;
  else if (y < 165 && y >= 135) return 2;
  else if (y < 135 && y >= 80) return 1;
  else if (y < 80 && y >= 30) return 0;
  else return -1;

}

void DynamicLoop() {

  uint8_t status = 0;
  status = BSP_TS_Init(lcd.GetXSize(), lcd.GetYSize());
  if (status != TS_OK){
    BSP_LCD_SetBackColor(LCD_COLOR_WHITE); 
    BSP_LCD_SetTextColor(LCD_COLOR_RED);
    BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()- 95, (uint8_t*)"ERROR", CENTER_MODE);
    BSP_LCD_DisplayStringAt(0, BSP_LCD_GetYSize()- 80, (uint8_t*)"Touchscreen cannot be initialized", CENTER_MODE);
  }
  else{
    TS_StateTypeDef ts_state;
    while (true) {
        BSP_TS_GetState(&ts_state);
        if (ts_state.TouchDetected) {
            if(header == "Menu"){
                touched = 1;
                if (ts_state.Y < LINE(5) && ts_state.Y >= LINE(0)) header = "Create User";
                else if (ts_state.Y < LINE(10)&& ts_state.Y >= LINE(5)) header = "Delete User";
                else if (ts_state.Y < LINE(15) && ts_state.Y >= LINE(10)) header = "User Profiles";
                else header = "Menu";
            }
            else if(header == "User Profiles"){
                current_user = getUser(ts_state.Y) == -1 ? current_user : getUser(ts_state.Y);
            }
        }
        thread_sleep_for(50);
    }
  }
}
int main()
{
Thread display;
display.start(DisplayLoop);

Thread dynamic_ui;
dynamic_ui.start(DynamicLoop);
while(1)
{
}

}

