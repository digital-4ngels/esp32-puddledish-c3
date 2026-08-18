#pragma once

#define LCD_H_RES 466
#define LCD_V_RES 466
#define LCD_COL_OFFSET 6

#define LCD_PCLK GPIO_NUM_38
#define LCD_CS GPIO_NUM_12
#define LCD_RST GPIO_NUM_39
#define LCD_D0 GPIO_NUM_4
#define LCD_D1 GPIO_NUM_5
#define LCD_D2 GPIO_NUM_6
#define LCD_D3 GPIO_NUM_7

#define BAND_ROWS 22
#define BAND_COUNT ((LCD_V_RES + BAND_ROWS - 1) / BAND_ROWS)

#define DISPLAY_BRIGHTNESS 230

#define BOX_W ((float)LCD_H_RES)
#define BOX_H ((float)LCD_V_RES)
#define BOX_D 75.0f
#define BOX_CORNER_R (BOX_W * 0.5f)

#define PX_PER_METER 12677.0f
#define PX_PER_MM (PX_PER_METER / 1000.0f)
#define BOX_BACK_FILLET_MM 2.0f
#define BOX_BACK_FILLET (BOX_BACK_FILLET_MM * PX_PER_MM)
#define BOX_FRONT_FILLET (BOX_BACK_FILLET * 0.25f)

#define PARTICLE_MAX 1000
#define PARTICLE_COUNT 900
#define REST_SPACING 16.0f

#define TIME_SCALE 0.068f
#define SIM_DT_MAX 0.0022f
#define GRAVITY_MPS2 9.81f
#define GRAVITY_GAIN 1.8f
#define SMOOTH_RADIUS 28.0f
#define SUBSTEPS 1
#define K_PRESSURE 400000.0f
#define K_NEAR_PRESSURE 800000.0f
#define MAX_DISPLACEMENT 4.0f
#define WALL_JITTER 0.35f
#define VISC_SIGMA 45.0f
#define VISC_BETA 0.03f
#define WALL_RESTITUTION 0.25f
#define WALL_FRICTION 0.96f

#define GRAVITY_LP_HZ 1.2f
#define SHAKE_GAIN 3.0f
#define ROTATION_GAIN 1.0f
#define IMU_MAP_X(ax, ay, az) (-(ax))
#define IMU_MAP_Y(ax, ay, az) (-(ay))
#define IMU_MAP_Z(ax, ay, az) (az)

#define PROJ_FOCAL 100.0f
#define PARTICLE_RADIUS_PX 7.2f
#define DISC_MAX_R 10
#define HIGHLIGHT_ENABLE 1
#define HIGHLIGHT_LIFT 0.55f
#define SPEED_LEVELS 64
#define DEPTH_LEVELS 16
#define SPEED_COLOR_MAX 5000.0f
#define SPEED_COLOR_GAMMA 0.55f
#define DEPTH_DIM_MIN 0.32f

/* Native +Y is physical right when the board is held USB/PWR-down. */
#define SCREEN_ROTATE_90_CW 1
