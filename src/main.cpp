#include <Arduino.h>
#include <driver/i2s.h>
#include "lvgl.h"
#include "I2SSampler.h"
#include "Processor.h"

#define LGFX_USE_V1
#define LGFX_M5STICK_C
#include "LovyanGFX.hpp"
#include "LGFX_AUTODETECT.hpp"

// LVGL
#define LV_TICK_PERIOD_MS 10
#define LV_HOR_RES_MAX    240
#define LV_VER_RES_MAX    135
#define DISP_BUF_SIZE     LV_HOR_RES_MAX * 80

// I2S
#define WINDOW_SIZE  512
#define I2S_BCK_PIN  -1
#define I2S_WS_PIN   0
#define I2S_DATA_PIN 34

uint32_t color_map[] = {
    0x24c4e6, 0x05a6fb, 0x0571fb, 0x053ffb, 0x0509fb, 0x3305fb, 0x6905fb,
    0x9705fb, 0xcd05fb, 0xfb05f7, 0xfb05c1, 0xfb058f, 0xfb055a, 0xfb0528,
    0xfb1505, 0xfb4a05, 0xfb7c05, 0xfbb205, 0xfbe405, 0xe0fb05, 0xaefb05,
    0x78fb05, 0x46fb05, 0x11fb05, 0x05fb2c, 0x05fb5d, 0x05fb93, 0x05fbc5,
    0x05fbfb, 0x05c9fb, 0x0593fb, 0x0584fb,
};

static void lvgl_init(void);
static void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area,
                          lv_color_t *color_p);
static void lv_tick_task(void *arg);
static void i2s_read_task(void *param);
static void audio_processing_task(void *param);
static void bar_value_update(float *mag);
static void spectrum_draw_event_cb(lv_event_t *e);

static LGFX lcd;
static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf1[DISP_BUF_SIZE];

lv_obj_t *spectrum_obj = NULL;

// i2s config for reading mic
i2s_config_t i2s_config = {
    .mode        = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM),
    .sample_rate = 64000,
    .bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format       = I2S_CHANNEL_FMT_ONLY_RIGHT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count        = 2,
    .dma_buf_len          = 1024,
};

// i2s pins
i2s_pin_config_t i2s_pins = {.bck_io_num   = I2S_PIN_NO_CHANGE,
                             .ws_io_num    = I2S_WS_PIN,
                             .data_out_num = I2S_PIN_NO_CHANGE,
                             .data_in_num  = I2S_DATA_PIN};

I2SSampler i2s_sampler;
Processor audio_processor(WINDOW_SIZE);
TaskHandle_t processing_task_handle;
float bar_chart_peaks[WINDOW_SIZE] = {0};
float bar_chart[WINDOW_SIZE]       = {0};

/* Creates a semaphore to handle concurrent call to lvgl stuff
 * If you wish to call *any* lvgl function from other threads/tasks
 * you should lock on the very same semaphore! */
SemaphoreHandle_t xGuiSemaphore;

void setup() {
    /* Initialize the Serial */
    Serial.begin(115200);

    /* Initialize the LCD */
    lcd.init();
    /* Landscape orientation, flipped */
    lcd.setRotation(3);
    lcd.setBrightness(100);

    /* Initialize the LVGL */
    lvgl_init();

    xGuiSemaphore = xSemaphoreCreateMutex();

    /* Create and start a periodic timer interrupt to call lv_tick_inc */
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &lv_tick_task, .name = "periodic_gui"};
    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    ESP_ERROR_CHECK(
        esp_timer_start_periodic(periodic_timer, LV_TICK_PERIOD_MS * 1000));

    spectrum_obj = lv_obj_create(lv_scr_act());
    lv_obj_remove_style_all(spectrum_obj);
    lv_obj_refresh_ext_draw_size(spectrum_obj);
    lv_obj_set_size(spectrum_obj, LV_HOR_RES_MAX - 16, LV_VER_RES_MAX - 10);
    lv_obj_set_pos(spectrum_obj, 8, 5);
    lv_obj_clear_flag(spectrum_obj,
                      LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(spectrum_obj, spectrum_draw_event_cb, LV_EVENT_ALL,
                        NULL);

    xTaskCreatePinnedToCore(audio_processing_task, "audio processing task",
                            4096, NULL, 2, &processing_task_handle, 0);
    i2s_sampler.start(I2S_NUM_0, i2s_pins, i2s_config, WINDOW_SIZE,
                      processing_task_handle);
}

void loop() {
    /* Delay 1 tick (assumes FreeRTOS tick is 10ms) */
    vTaskDelay(pdMS_TO_TICKS(10));
    /* Try to take the semaphore, call lvgl related function on success */
    if (pdTRUE == xSemaphoreTake(xGuiSemaphore, portMAX_DELAY)) {
        lv_task_handler();
        xSemaphoreGive(xGuiSemaphore);
    }
}

static void lvgl_init(void) {
    /* Initialize the lvgl */
    lv_init();

    lv_disp_draw_buf_init(&draw_buf, buf1, NULL, DISP_BUF_SIZE);

    /* Initialize the display */
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res  = LV_HOR_RES_MAX;
    disp_drv.ver_res  = LV_VER_RES_MAX;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);
}

static void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area,
                          lv_color_t *color_p) {
    int w = (area->x2 - area->x1 + 1);
    int h = (area->y2 - area->y1 + 1);

    /* Start new TFT transaction */
    lcd.startWrite();
    /* set the working window */
    lcd.setAddrWindow(area->x1, area->y1, w, h);

    /* Write the buffer to the display */
    lcd.writePixels((lgfx::rgb565_t *)&color_p->full, w * h);

    /* terminate TFT transaction */
    lcd.endWrite();
    /* tell lvgl that flushing is done */
    lv_disp_flush_ready(disp);
}

static void lv_tick_task(void *arg) {
    (void)arg;

    lv_tick_inc(LV_TICK_PERIOD_MS);
}

static void audio_processing_task(void *param) {
    const TickType_t xMaxBlockTime = pdMS_TO_TICKS(100);
    while (true) {
        // wait for some samples to process
        uint32_t ulNotificationValue = ulTaskNotifyTake(pdTRUE, xMaxBlockTime);
        if (ulNotificationValue > 0) {
            int16_t *samples = i2s_sampler.getCapturedAudioBuffer();
            audio_processor.update(samples);
            bar_value_update(audio_processor.m_energy);
            lv_obj_invalidate(spectrum_obj);
        }
    }
}

static void bar_value_update(float *mag) {
    for (int i = 0; i < WINDOW_SIZE; i++) {
        float m = mag[i];
        if (m > bar_chart[i]) {
            bar_chart[i] = m;
        } else {
            bar_chart[i] = 0.7 * bar_chart[i] + 0.3 * m;
        }
        if (m > bar_chart_peaks[i]) {
            bar_chart_peaks[i] = m;
        } else {
            bar_chart_peaks[i] = 0.95 * bar_chart_peaks[i] + 0.05 * m;
        }
    }
}

static void spectrum_draw_event_cb(lv_event_t *e) {
    lv_event_code_t code = lv_event_get_code(e);

    if (code == LV_EVENT_REFR_EXT_DRAW_SIZE) {
        lv_event_set_ext_draw_size(e, LV_VER_RES);
    } else if (code == LV_EVENT_COVER_CHECK) {
        lv_event_set_cover_res(e, LV_COVER_RES_NOT_COVER);
    } else if (code == LV_EVENT_DRAW_POST) {
        lv_obj_t *obj           = lv_event_get_target(e);
        lv_draw_ctx_t *draw_ctx = lv_event_get_draw_ctx(e);

        lv_opa_t opa = lv_obj_get_style_opa(obj, LV_PART_MAIN);
        if (opa < LV_OPA_MIN) return;

        lv_draw_rect_dsc_t draw_rect_dsc;
        lv_draw_rect_dsc_init(&draw_rect_dsc);
        draw_rect_dsc.bg_opa = LV_OPA_COVER;

        lv_draw_line_dsc_t draw_line_dsc;
        lv_draw_line_dsc_init(&draw_line_dsc);
        draw_line_dsc.width = 1;

        int x_step    = int((LV_HOR_RES_MAX - 16) / (WINDOW_SIZE / 16));
        int bar_count = 1;
        for (int i = 2; i < WINDOW_SIZE / 4; i += 4) {
            float ave = 0;
            for (int j = 0; j < 4; j++) {
                ave += bar_chart[i + j];
            }
            ave /= 4;
            int bar_value = std::min(125.0f, 0.25f * ave);
            ave           = 0;
            for (int j = 0; j < 4; j++) {
                ave += bar_chart_peaks[i + j];
            }
            ave /= 4;
            int peak_value = std::min(125.0f, 0.25f * ave);

            draw_rect_dsc.bg_color = lv_color_hex(color_map[bar_count - 1]);
            /* 5 is the bar width,  bar_value is bar height */
            lv_area_t above_rect;
            above_rect.x1 = bar_count * x_step;
            above_rect.x2 = bar_count * x_step + 5;
            above_rect.y1 = 63 - int(bar_value / 2);
            above_rect.y2 = 63;
            lv_draw_rect(draw_ctx, &draw_rect_dsc, &above_rect);

            lv_area_t blow_rect;
            blow_rect.x1 = bar_count * x_step;
            blow_rect.x2 = bar_count * x_step + 5;
            blow_rect.y1 = 63;
            blow_rect.y2 = 63 + int(bar_value / 2);
            lv_draw_rect(draw_ctx, &draw_rect_dsc, &blow_rect);

            draw_line_dsc.color = lv_color_hex(color_map[bar_count - 1]);

            lv_point_t above_line[2];
            /* upside line always 2 px above the bar */
            above_line[0].x = bar_count * x_step;
            above_line[0].y = 63 - int(peak_value / 2) - 2;
            above_line[1].x = bar_count * x_step + 6;
            above_line[1].y = 63 - int(peak_value / 2) - 2;
            lv_draw_line(draw_ctx, &draw_line_dsc, &above_line[0],
                         &above_line[1]);

            lv_point_t blow_line[2];
            /* under line always 2 px below the bar */
            blow_line[0].x = bar_count * x_step;
            blow_line[0].y = 63 + int(peak_value / 2) + 2;
            blow_line[1].x = bar_count * x_step + 6;
            blow_line[1].y = 63 + int(peak_value / 2) + 2;
            lv_draw_line(draw_ctx, &draw_line_dsc, &blow_line[0],
                         &blow_line[1]);

            bar_count++;
        }
    }
}
