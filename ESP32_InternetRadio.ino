#include <WiFi.h>
#include "Audio.h"
#include "AiEsp32RotaryEncoder.h"

// -----------------------------------------------------------------------------
// WiFi Settings
// -----------------------------------------------------------------------------
const char* ssid = "YOUR_WIFI_SSID";       // あなたのWiFi SSID
const char* password = "YOUR_WIFI_PASSWORD"; // あなたのWiFi パスワード

// -----------------------------------------------------------------------------
// Internet Radio Station List
// -----------------------------------------------------------------------------
const char* station_names[] = {
    "NHK FM (Tokyo)",
    "Shibuya Radio",
    "J-Pop Powerplay"
};
const char* station_urls[] = {
    "https://nhkradioakfm-i.akamaihd.net/hls/live/512290/1-fm/1-fm-01.m3u8",
    "http://shibuyaradio.net:8000/stream",
    "http://stream.j-pop.im:8000/j-pop_powerplay"
};
const int station_count = sizeof(station_urls) / sizeof(station_urls[0]);
int current_station_index = 0;

// -----------------------------------------------------------------------------
// Hardware Pin Settings
// -----------------------------------------------------------------------------
// I2S
#define I2S_BCLK      22
#define I2S_LRC       21
#define I2S_DOUT      19
// Rotary Encoder
#define ROTARY_ENCODER_A_PIN      25
#define ROTARY_ENCODER_B_PIN      26
#define ROTARY_ENCODER_STEPS      4
// Push Button
#define BUTTON_PIN                32 // 独立したボタン用のピン

// -----------------------------------------------------------------------------
// Global Objects and Variables
// -----------------------------------------------------------------------------
Audio audio;
// ロータリーエンコーダのSWピンを-1に設定して、スイッチ機能を無効化
AiEsp32RotaryEncoder rotaryEncoder = AiEsp32RotaryEncoder(ROTARY_ENCODER_A_PIN, ROTARY_ENCODER_B_PIN, -1, -1, ROTARY_ENCODER_STEPS);

enum Mode { VOLUME_CONTROL, STATION_SELECT };
Mode current_mode = VOLUME_CONTROL;

// ボタンのデバウンス用
unsigned long last_button_press_time = 0;
const long debounce_delay = 50;

// =============================================================================
//  Rotary Encoder Event Handler
// =============================================================================
void rotary_loop() {
    if (rotaryEncoder.encoderChanged()) {
        int16_t val = rotaryEncoder.readEncoder();

        if (current_mode == VOLUME_CONTROL) {
            int current_volume = audio.getVolume();
            if (val > 0) current_volume++; else current_volume--;
            if (current_volume < 0) current_volume = 0;
            if (current_volume > 21) current_volume = 21;
            audio.setVolume(current_volume);
            Serial.printf("Volume: %d\n", current_volume);
        } else {
            if (val > 0) current_station_index++; else current_station_index--;
            if (current_station_index >= station_count) current_station_index = 0;
            if (current_station_index < 0) current_station_index = station_count - 1;
            Serial.printf("\nChanging station to: %s\n", station_names[current_station_index]);
            audio.connecttohost(station_urls[current_station_index]);
        }
        rotaryEncoder.reset(0);
    }
}

// =============================================================================
//  Button Press Handler
// =============================================================================
void check_button_press() {
    // 現在の時刻と最後のボタン押下時刻の差がデバウンス時間より大きいかチェック
    if ((millis() - last_button_press_time) > debounce_delay) {
        // ボタンが押されているかチェック (INPUT_PULLUPなのでLOWが押された状態)
        if (digitalRead(BUTTON_PIN) == LOW) {
            last_button_press_time = millis(); // 押下時刻を更新

            // モードを切り替え
            if (current_mode == VOLUME_CONTROL) {
                current_mode = STATION_SELECT;
                Serial.println("\n--- Mode: Station Select ---");
                Serial.printf("Current Station: %s\n", station_names[current_station_index]);
            } else {
                current_mode = VOLUME_CONTROL;
                Serial.println("\n--- Mode: Volume Control ---");
                Serial.printf("Current Volume: %d\n", audio.getVolume());
            }
        }
    }
}

// =============================================================================
//  Setup
// =============================================================================
void setup() {
    Serial.begin(115200);

    // --- Button Pin Setup ---
    pinMode(BUTTON_PIN, INPUT_PULLUP);

    // --- WiFi Connection ---
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        Serial.print(".");
        delay(500);
    }
    Serial.println("\nWiFi connected!");

    // --- Rotary Encoder Setup ---
    rotaryEncoder.begin();
    rotaryEncoder.setup([] { rotaryEncoder.readEncoder_ISR(); }); // ISR設定のみ
    rotaryEncoder.setBoundaries(0, 100, false);
    rotaryEncoder.setAcceleration(0);

    // --- I2S Audio Setup ---
    audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
    audio.setVolume(5);

    // --- Initial Connection ---
    Serial.println("\n--- Mode: Volume Control ---");
    Serial.printf("Initial station: %s\n", station_names[current_station_index]);
    audio.connecttohost(station_urls[current_station_index]);
}

// =============================================================================
//  Loop
// =============================================================================
void loop() {
    audio.loop();
    rotary_loop();
    check_button_press(); // 毎ループ、ボタンの状態をチェック
}

// =============================================================================
//  Audio library events (unchanged)
// =============================================================================
void audio_info(const char *info){ Serial.print("info        "); Serial.println(info); }
void audio_id3data(const char *info){ Serial.print("id3data     "); Serial.println(info); }
void audio_eof_mp3(const char *info){ Serial.print("eof_mp3     "); Serial.println(info); }
void audio_showstation(const char *info){ Serial.print("station     "); Serial.println(info); }
void audio_showstreamtitle(const char *info){ Serial.print("streamtitle "); Serial.println(info); }
void audio_bitrate(const char *info){ Serial.print("bitrate     "); Serial.println(info); }
void audio_commercial(const char *info){ Serial.print("commercial  "); Serial.println(info); }
void audio_icyurl(const char *info){ Serial.print("icyurl      "); Serial.println(info); }
void audio_lasthost(const char *info){ Serial.print("lasthost    "); Serial.println(info); }
void audio_eof_speech(const char *info){ Serial.print("eof_speech  "); Serial.println(info); }
