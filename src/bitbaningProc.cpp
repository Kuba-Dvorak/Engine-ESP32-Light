#include <iostream>
#include <vector>
#include <random>
#include <cmath>
#include <math.h>
#include <regex>
#include <algorithm>
#include <array>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_rgb.h"
#include "driver/gpio.h"
#include "esp_heap_caps.h"
#include "Arduino.h"
#include "driver/periph_ctrl.h"
#include "soc/lcd_cam_struct.h"
#include "soc/lcd_cam_reg.h"
#include "driver/gpio.h"      // gpio_matrix_out
#include "rom/lldesc.h"       // lldesc_t
#include "esp_heap_caps.h"    // heap_caps_malloc pro DMA paměť
#include "soc/rtc.h"          // APLL nastavení




struct VGATimings {
    int screenWidth, screenHeight;
    int HFront, HBack, Hsync;
    int VFront, VBack, Vsync;
    int pxClock;
    bool hsyncPos, vsyncPos;
    uint8_t bitHsync, bitVsync;
    float VsyncFreq, HsyncFreq;

    int heightTotal() {
        return screenHeight + VBack + VFront + Vsync;
    }

    int widthTotal() {
        return screenWidth + HBack + HFront + Hsync;
    }

    void setupPins(std::array<int, 8> &pins) {
        for (int i = 0; i < 8; i += 1) {
            gpio_pad_select_gpio(pins[i]);
            gpio_set_direction((gpio_num_t)pins[i], GPIO_MODE_OUTPUT);
            gpio_matrix_out(pins[i], LCD_DATA_OUT0_IDX + i, false,false);
        }
    }

    void prepareLCDMod() {
        periph_module_enable(PERIPH_LCD_CAM_MODULE);
        LCD_CAM.lcd_user.lcd_reset = 1;
        LCD_CAM.lcd_user.lcd_reset = 0;
        LCD_CAM.lcd_clock.val = 0;
        LCD_CAM.lcd_clock.clk_en = 1;
        LCD_CAM.lcd_clock.lcd_clk_sel = 2;
        LCD_CAM.lcd_clock.lcd_clkm_div_num = 2;
        LCD_CAM.lcd_clock.lcd_clkm_div_a = 0;
        LCD_CAM.lcd_clock.lcd_clkm_div_b = 0;
        LCD_CAM.lcd_clock.lcd_clkcnt_n = 0;
        LCD_CAM.lcd_clock.lcd_ck_idle_edge = 0;
        LCD_CAM.lcd_clock.lcd_ck_out_edge = 0;
        LCD_CAM.lcd_ctrl2.val = 0;
        LCD_CAM.lcd_ctrl2.lcd_hsync_width = Hsync;
        LCD_CAM.lcd_ctrl2.lcd_vsync_width = Vsync;
        LCD_CAM.lcd_ctrl2.lcd_hsync_idle_pol = hsyncPos;
        LCD_CAM.lcd_ctrl2.lcd_vsync_idle_pol = vsyncPos;
        LCD_CAM.lcd_ctrl2.lcd_hs_blank_en = 1;
        LCD_CAM.lcd_user.val = 0;
        LCD_CAM.lcd_user.lcd_2byte_en = 0;
        LCD_CAM.lcd_user.lcd_bit_order = 0;
        LCD_CAM.lcd_user.lcd_byte_order = 0;
        LCD_CAM.lcd_user.lcd_8bits_order = 0;
        LCD_CAM.lcd_user.lcd_always_out_en = 1;
        LCD_CAM.lcd_user.lcd_dout = 1;
        LCD_CAM.lcd_user.lcd_dout_cyclelen = 0;
        rtc_clk_apll_enable(true, 0, 128, 2, 0);
    }



    VGATimings(int width = 640, int height = 480, int framerate = 60) {
        if (width == 640 && height == 480 && framerate == 60) {
            this->screenHeight = 480;
            this->screenWidth = 640;
            this->Hsync = 96;
            this->HFront = 16;
            this->HBack = 48;
            this->Vsync = 2;
            this->VBack = 33;
            this->VFront = 10;
            this->pxClock = 25000000;
            this->hsyncPos = false;
            this->vsyncPos = false;
            this->bitHsync = (1 << 6);
            this->bitVsync = (1 << 7);
            this->HsyncFreq = ((float)pxClock / widthTotal());
            this->VsyncFreq = (HsyncFreq / heightTotal());
        }
        else {
            std::cout << "Unsuported resolution" << std::endl;
            Serial.printf("Unsuported resolution");
        }
    }

    uint8_t preparePixel(uint8_t &data, bool &Vsync, bool &Hsync) {
        uint8_t p = 0;
        p |= data;
        if ((Hsync && hsyncPos) || ((!Hsync) && (!hsyncPos))) {
            p |= bitHsync;
        }
        if ((Vsync && vsyncPos) || ((!Vsync) && (!vsyncPos))) {
            p |= bitVsync;
        }
        return p;
    }
};