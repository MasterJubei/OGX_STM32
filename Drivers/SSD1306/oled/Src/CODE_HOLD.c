CODE HOLD FILE _ DO NOT COMPILE


//  SSD1306_ScrollRight(0,7);  // scroll entire screen
//  HAL_Delay(2000);  // 2 sec
//
//  SSD1306_ScrollLeft(0,7);  // scroll entire screen
//  HAL_Delay(2000);  // 2 sec
//
//  SSD1306_Stopscroll();
//  SSD1306_Clear();
//
//  SSD1306_DrawBitmap(0,0,logo, 128, 64, SSD1306_COLOR_WHITE);
//  SSD1306_UpdateScreen();
//
//  HAL_Delay(2000);
//
//  SSD1306_ScrollRight(0x00, 0x0f);    // scroll entire screen right
//
//  HAL_Delay (2000);
//
//  SSD1306_ScrollLeft(0x00, 0x0f);  // scroll entire screen left
//
//  HAL_Delay (2000);
//
//  SSD1306_Scrolldiagright(0x00, 0x0f);  // scroll entire screen diagonal right
//
//  HAL_Delay (2000);
//
//  SSD1306_Scrolldiagleft(0x00, 0x0f);  // scroll entire screen diagonal left
//
//  HAL_Delay (2000);
//
//  SSD1306_Stopscroll();   // stop scrolling. If not done, screen will keep on scrolling
//
//
//  SSD1306_InvertDisplay(1);   // invert the display
//
//  HAL_Delay(2000);
//
//  SSD1306_InvertDisplay(0);  // normalize the display
////
//
//  HAL_Delay(2000);

  /// TESTSSSSSSSSS

//    TestLines(1);
//    HAL_Delay (1000);
//    TestRectangles(1);
//    HAL_Delay (1000);
//    TestFilledRectangles(1);
//    HAL_Delay (1000);
//    TestCircles(6,1);
//    HAL_Delay (1000);
//    TestFilledCircles(6, 1);
//    TestTriangles(1);

//  SSD1306_Clear();
//  SSD1306_DrawTriangle(3, 2, 28, 12, 16, 24, SSD1306_COLOR_WHITE);
//  SSD1306_UpdateScreen();
//  HAL_Delay(2000);
  
//  SSD1306_Clear();




    SSD1306_Clear();
    SSD1306_GotoXY (65, 30);
    sprintf(number, "%-3d", counter);
    SSD1306_Puts(number, &Font_11x18, SSD1306_COLOR_WHITE);
    SSD1306_UpdateScreen(); //display
    counter++;

    HAL_Delay(1000);

    
    
    
    //    //// HORSE ANIMATION START //////
//    SSD1306_Clear();
//    SSD1306_DrawBitmap(0,0,horse1,128,64,SSD1306_COLOR_WHITE);
//    SSD1306_UpdateScreen();
//
//    SSD1306_Clear();
//    SSD1306_DrawBitmap(0,0,horse2,128,64,SSD1306_COLOR_WHITE);
//    SSD1306_UpdateScreen();
//
//    SSD1306_Clear();
//    SSD1306_DrawBitmap(0,0,horse3,128,64,SSD1306_COLOR_WHITE);
//    SSD1306_UpdateScreen();
//
//    SSD1306_Clear();
//    SSD1306_DrawBitmap(0,0,horse4,128,64,SSD1306_COLOR_WHITE);
//    SSD1306_UpdateScreen();
//
//    SSD1306_Clear();
//    SSD1306_DrawBitmap(0,0,horse5,128,64,SSD1306_COLOR_WHITE);
//    SSD1306_UpdateScreen();
//
//    SSD1306_Clear();
//    SSD1306_DrawBitmap(0,0,horse6,128,64,SSD1306_COLOR_WHITE);
//    SSD1306_UpdateScreen();
//
//    SSD1306_Clear();
//    SSD1306_DrawBitmap(0,0,horse7,128,64,SSD1306_COLOR_WHITE);
//    SSD1306_UpdateScreen();
//
//    SSD1306_Clear();
//    SSD1306_DrawBitmap(0,0,horse8,128,64,SSD1306_COLOR_WHITE);
//    SSD1306_UpdateScreen();
//
//    SSD1306_Clear();
//    SSD1306_DrawBitmap(0,0,horse9,128,64,SSD1306_COLOR_WHITE);
//    SSD1306_UpdateScreen();
//
//    SSD1306_Clear();
//    SSD1306_DrawBitmap(0,0,horse10,128,64,SSD1306_COLOR_WHITE);
//    SSD1306_UpdateScreen();
//    //// HORSE ANIMATION ENDS //////    
    
    
    
    