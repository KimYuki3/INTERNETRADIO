#include <WiFi.h>
#include <WiFiManager.h> // ★ WiFiManagerライブラリ
#include <HTTPClient.h>
#include <U8g2lib.h>
#include <AiEsp32RotaryEncoder.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"
#include "freertos/semphr.h"
#include "driver/i2s.h"
#include "mp3_decoder.h"

// -----------------------------------------------------------------------------
// WiFi & Radio Stations
// -----------------------------------------------------------------------------
const char* station_names[] = {
    // Classical
    "Linn Classical", "Radio Swiss Classic", "Venice Classic Radio", "Abacus.fm - Mozart", "Abacus.fm - Beethoven",
    "Abacus.fm - Bach", "KUSC Classical", "WCPE The Classical", "WCRB Classical", "King FM Classical",
    // Jazz
    "Linn Jazz", "Radio Swiss Jazz", "The Jazz Groove", "JAZZ.FM91", "WBGO Jazz88",
    "KJAZZ 88.1", "WJSU - The Jazz Spot", "ABC Jazz", "Jazz24", "TSF Jazz",
    // Healing / Ambient
    "SomaFM - Drone Zone", "SomaFM - Groove Salad", "SomaFM - Lush", "SomaFM - Space Station", "An Ambient Sleeping Pill",
    "Radio Art - Sleep", "Radio Art - Stress Relief", "Ambi Nature Radio", "Chillkyway", "StillStream"
};
const char* station_urls[] = {
    // Classical
    "http://radio.linn.co.uk:8003/autodj", "http://stream.srg-ssr.ch/m/rsc_de/mp3_128", "http://174.36.1.135:8010/stream", "http://streams.abacus.fm/mozart.mp3", "http://streams.abacus.fm/beethoven.mp3",
    "http://streams.abacus.fm/bach.mp3", "https://kusc.streamguys1.com/kusc-128", "http://audio-mp3.wcpe.org:80/wcpe.mp3", "https://streams.wgbh.org/wcrb.mp3", "http://classicalking.streamguys1.com/king-fm-128.mp3",
    // Jazz
    "http://radio.linn.co.uk:8004/autodj", "http://stream.srg-ssr.ch/m/rsj/mp3_128", "https://ice5.securenetsystems.net/JAZZ2", "https://jazzfm91.streamb.live/SB00009", "https://wbgo.streamguys.net/wbgo128",
    "https://kjazz.streamguys1.com/kjazz-128.mp3", "http://192.235.87.133:8000/wjsu-jazz", "https://live-radio01.mediahubaustralia.com/JAZ/mp3/", "https://jazz24.streamguys1.com/jazz24-128.mp3", "http://tsfjazz.ice.infomaniak.ch/tsfjazz-high.mp3",
    // Healing / Ambient
    "http://ice1.somafm.com/dronezone-128-mp3", "http://ice1.somafm.com/groovesalad-128-mp3", "http://ice1.somafm.com/lush-128-mp3", "http://ice1.somafm.com/spacestation-128-mp3", "http://stream.aasp.fm:8000/",
    "https://live.radioart.com/fSleep.mp3", "https://live.radioart.com/fStress_Relief.mp3", "http://stream.ambinatureradio.com:8000/1", "http://192.99.35.215:5052/stream", "http://listen.stillstream.com:8000/"
};
const int station_count = sizeof(station_urls) / sizeof(station_urls[0]);

// -----------------------------------------------------------------------------
// Hardware Pin Settings
// -----------------------------------------------------------------------------
#define I2S_BCLK      22
#define I2S_LRC       21
#define I2S_DOUT      19
#define OLED_SCL      5
#define OLED_SDA      4
#define ROTARY_ENCODER_A_PIN  25
#define ROTARY_ENCODER_B_PIN  26
#define BUTTON_PIN            32
#define LED_PIN               LED_BUILTIN

// -----------------------------------------------------------------------------
// Global Objects & State Variables
// -----------------------------------------------------------------------------
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, OLED_SCL, OLED_SDA);
AiEsp32RotaryEncoder rotaryEncoder = AiEsp32RotaryEncoder(ROTARY_ENCODER_A_PIN, ROTARY_ENCODER_B_PIN, -1, -1, 4);
MP3_DECODER *mp3_decoder;

// --- 状態管理用の変数 ---
enum RadioMode { MODE_VOLUME, MODE_STATION };
volatile RadioMode g_current_mode = MODE_VOLUME;
volatile int g_volume = 15; // 0-21
volatile int g_station_index = 0;
volatile bool g_station_changed = true; // 起動時に最初の局に接続するためのフラグ

// --- FreeRTOS関連 ---
static RingbufHandle_t ringbuf_handle;
static SemaphoreHandle_t g_state_mutex; // ★ 状態変数を保護するミューテックス
static TaskHandle_t http_task_handle = NULL;

// (I2S Audio Setup は変更なし)
void setup_i2s() { /* ... */ }

// =============================================================================
//  Control Task (Encoder and Button)
// =============================================================================
void control_task(void *pvParameters) {
    long last_button_press = 0;

    while(1) {
        // --- Button Check ---
        if (digitalRead(BUTTON_PIN) == LOW && (millis() - last_button_press > 200)) {
            last_button_press = millis();
            xSemaphoreTake(g_state_mutex, portMAX_DELAY);
            g_current_mode = (g_current_mode == MODE_VOLUME) ? MODE_STATION : MODE_VOLUME;
            xSemaphoreGive(g_state_mutex);
        }

        // --- Encoder Check ---
        if (rotaryEncoder.encoderChanged()) {
            int16_t val = rotaryEncoder.readEncoder();
            xSemaphoreTake(g_state_mutex, portMAX_DELAY);
            if (g_current_mode == MODE_VOLUME) {
                g_volume += val;
                if (g_volume < 0) g_volume = 0;
                if (g_volume > 21) g_volume = 21;
            } else { // MODE_STATION
                g_station_index += val;
                if (g_station_index >= station_count) g_station_index = 0;
                if (g_station_index < 0) g_station_index = station_count - 1;
                g_station_changed = true; // ★ 局変更フラグを立てる
            }
            xSemaphoreGive(g_state_mutex);
            rotaryEncoder.reset(0);
        }
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}

// =============================================================================
//  Display Task
// =============================================================================
void display_task(void *pvParameters) {
    while (1) {
        xSemaphoreTake(g_state_mutex, portMAX_DELAY);
        // ローカル変数に状態をコピーして、すぐにミューテックスを解放する
        RadioMode mode = g_current_mode;
        int volume = g_volume;
        int station_idx = g_station_index;
        xSemaphoreGive(g_state_mutex);

        u8g2.clearBuffer();
        
        // --- Mode Display ---
        u8g2.setFont(u8g2_font_6x10_tf);
        if (mode == MODE_VOLUME) {
            u8g2.drawStr(0, 62, "Mode: Volume");
        } else {
            u8g2.drawStr(0, 62, "Mode: Station");
        }

        // --- Station Name ---
        u8g2.setFont(u8g2_font_ncenB10_tr);
        u8g2.drawStr(0, 12, station_names[station_idx]);

        // --- Volume Bar ---
        u8g2.drawFrame(0, 20, 128, 12);
        u8g2.drawBox(2, 22, (124.0f / 21.0f) * volume, 8);

        u8g2.sendBuffer();
        vTaskDelay(250 / portTICK_PERIOD_MS);
    }
}

// =============================================================================
//  MP3 Decode and Playback Task
// =============================================================================
void mp3_decode_task(void *pvParameters) {
    uint8_t *input_buffer = (uint8_t*) malloc(2048);
    int16_t *output_buffer = (int16_t*) malloc(2048);
    float volume_factor = 1.0;

    while (1) {
        size_t bytes_read = 0;
        uint8_t *data = (uint8_t*) xRingbufferReceive(ringbuf_handle, &bytes_read, pdMS_TO_TICKS(200));
        if (bytes_read > 0) {
            int bytes_decoded = mp3_decoder->decode(data, bytes_read, output_buffer);
            xRingbufferReturnItem(ringbuf_handle, (void*)data);

            if (bytes_decoded > 0) {
                xSemaphoreTake(g_state_mutex, portMAX_DELAY);
                // 0-21のボリューム値を0.0-1.0の係数に変換 (簡易的なカーブ)
                volume_factor = pow(10, (float)g_volume / 10.5f - 2.0f);
                xSemaphoreGive(g_state_mutex);

                // ソフトウェアボリューム調整
                for (int i = 0; i < bytes_decoded / 2; i++) {
                    output_buffer[i] = (int16_t)((float)output_buffer[i] * volume_factor);
                }
                
                size_t bytes_written;
                i2s_write(I2S_NUM_0, output_buffer, bytes_decoded, &bytes_written, portMAX_DELAY);
            }
        }
    }
}

// =============================================================================
//  HTTP Streaming Task
// =============================================================================
void http_stream_task(void *pvParameters) {
    while(1) {
        xSemaphoreTake(g_state_mutex, portMAX_DELAY);
        bool changed = g_station_changed;
        int station_idx = g_station_index;
        xSemaphoreGive(g_state_mutex);

        if (changed) {
            HTTPClient http;
            http.begin(station_urls[station_idx]);
            int httpCode = http.GET();
            if (httpCode > 0) {
                WiFiClient *stream = http.getStreamPtr();
                uint8_t buffer[128];
                
                xSemaphoreTake(g_state_mutex, portMAX_DELAY);
                g_station_changed = false; // フラグをリセット
                xSemaphoreGive(g_state_mutex);
                
                Serial.printf("Now playing: %s\n", station_names[station_idx]);

                while (!g_station_changed) { // 局変更フラグが立つまでループ
                    int len = stream->read(buffer, sizeof(buffer));
                    if (len > 0) {
                        xRingbufferSend(ringbuf_handle, buffer, len, pdMS_TO_TICKS(200));
                    } else {
                        break; // ストリームが終了したら抜ける
                    }
                    // 他のタスクにCPUを譲る
                    vTaskDelay(1);
                }
            }
            http.end();
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

// ★ WiFiManagerが設定モードに入った時に呼び出される関数
void configModeCallback (WiFiManager *myWiFiManager) {
    Serial.println("Entered config mode");
    Serial.println(WiFi.softAPIP());
    Serial.println(myWiFiManager->getConfigPortalSSID());

    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.drawStr(0, 10, "WiFi Setup Mode");
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(0, 25, "Connect to AP:");
    u8g2.drawStr(0, 38, myWiFiManager->getConfigPortalSSID().c_str());
    u8g2.drawStr(0, 52, "IP: 192.168.4.1");
    u8g2.sendBuffer();
}


// =============================================================================
//  Setup
// =============================================================================
void setup() {
    Serial.begin(115200);
    pinMode(LED_PIN, OUTPUT);
    pinMode(BUTTON_PIN, INPUT_PULLUP);

    u8g2.begin();
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(0, 10, "Starting up...");
    u8g2.sendBuffer();

    // --- WiFiManager Setup ---
    WiFiManager wm;
    
    // 設定モードに入った時のコールバック関数を登録
    wm.setAPCallback(configModeCallback);
    
    // 3分(180秒)経っても設定されなかったらタイムアウトして再起動
    wm.setConfigPortalTimeout(180);

    // LEDを点滅開始
    Ticker blinker;
    blinker.attach_ms(250, []() { digitalWrite(LED_PIN, !digitalRead(LED_PIN)); });

    // autoConnect: 保存された情報で接続を試み、失敗したら設定ポータルを開始
    if (!wm.autoConnect("InternetRadio_Setup")) {
        Serial.println("Failed to connect and hit timeout");
        u8g2.clearBuffer();
        u8g2.drawStr(0, 10, "Setup Timeout!");
        u8g2.sendBuffer();
        delay(3000);
        ESP.restart();
    }
    
    // --- WiFi Connected ---
    blinker.detach(); // 点滅を停止
    digitalWrite(LED_PIN, HIGH); // 点灯させる
    Serial.println("\nWiFi connected!");
    
    u8g2.clearBuffer();
    u8g2.drawStr(0, 10, "WiFi Connected!");
    u8g2.sendBuffer();
    delay(1000);


    // --- Start All Tasks ---
    setup_i2s();
    mp3_decoder = new MP3_DECODER();
    
    rotaryEncoder.begin();
    rotaryEncoder.setup([] { rotaryEncoder.readEncoder_ISR(); });
    rotaryEncoder.setAcceleration(0);

    g_state_mutex = xSemaphoreCreateMutex();
    ringbuf_handle = xRingbufferCreate(8 * 1024, RINGBUF_TYPE_BYTEBUF);

    xTaskCreatePinnedToCore(mp3_decode_task, "MP3DecodeTask", 4096, NULL, 5, NULL, 1);
    xTaskCreate(http_stream_task, "HTTPStreamTask", 4096, NULL, 4, &http_task_handle);
    xTaskCreate(display_task, "DisplayTask", 4096, NULL, 3, NULL);
    xTaskCreate(control_task, "ControlTask", 2048, NULL, 4, NULL);
}

void loop() {
    vTaskDelete(NULL); // このタスクは不要
}