/*
  esp32_bridge.ino - Cau noi STM32 <-> may tinh: UART <-> WiFi UDP, do tre thap.

  Luong du lieu:
    STM32 (USART1, 921600) --UART2--> ESP32 --WiFi UDP--> PC (pc_monitor/app.py)

  - ESP32 KHONG giai ma noi dung khung: chi tim dau khung AA 55, kiem tra CRC-8
    roi chuyen nguyen khung sang PC ngay lap tuc. Them truong du lieu moi o STM32
    chi can sua pc_monitor/protocol.py, khong can nap lai ESP32.
  - Tu tim PC: ESP32 phat "beacon" moi 1 s (broadcast, cong 4211). App PC nghe beacon,
    gui "HELLO" ve cong 4210 -> ESP32 ghi nho dia chi PC va gui du lieu thang (unicast).
    Khong can go IP may tinh vao code.
  - PC gui "PING..." -> ESP32 tra "PONG..." ngay de PC do tre WiFi (RTT).
  - Goi UDP tu PC bat dau bang AA 55 duoc chuyen nguyen xuong STM32 (de danh cho
    lenh chinh PID tu PC sau nay; STM32 hien chua doc).

  Giam tre:
  - WiFi.setSleep(false): tat modem-sleep (mac dinh bat, gay tre ~100 ms tung dot).
  - UART RX timeout 1 ky tu: du lieu duoc dua len ngay khi het khung, khong doi day FIFO.
  - Moi lan doc UART xong la gui UDP ngay (neu cung luc co nhieu khung thi gop 1 goi).
  - Dung UDP, khong dung TCP (khong cho bat tay / gui lai).

  Noi day (ESP32 DevKit):
    GPIO16 (RX2) <- PA9  (USART1_TX STM32)
    GPIO17 (TX2) -> PA10 (USART1_RX STM32)
    GND          -- GND STM32 (bat buoc)

  Board trong Arduino IDE: "ESP32 Dev Module" (lop esp32 by Espressif 3.x).
*/

#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp_system.h>

/* ================= CAU HINH - SUA CHO DUNG ================= */
#include "secrets.h"   /* WIFI_SSID, WIFI_PASS - xem secrets.example.h */

#define STM32_UART          Serial2
#define STM32_RX_PIN        16          /* <- PA9  cua STM32 */
#define STM32_TX_PIN        17          /* -> PA10 cua STM32 */
#define STM32_BAUD          921600      /* PHAI trung USART1 trong CubeMX */

#define UDP_PORT_ESP        4210        /* ESP32 nghe: HELLO / PING / lenh tu PC */
#define UDP_PORT_PC         4211        /* PC nghe: beacon + du lieu */
#define BEACON_PERIOD_MS    1000
#define CLIENT_TIMEOUT_MS   3000        /* 3 s khong nhan HELLO -> ngung gui */

#define LED_PIN             2           /* LED tren board; -1 neu khong co */
#define WIFI_TX_POWER_LOW   0           /* 1 = giam cong suat phat WiFi (11 dBm) -> giam dong dinh,
                                           thu khi nguon tren xe yeu (ESP32 tu khoi dong lai) */
/* Y nghia LED:
     3 nhay rat nhanh  = vua khoi dong (lap lai lien tuc = ESP32 bi reset lien tuc -> kiem tra NGUON)
     nhay nhanh        = dang ket noi WiFi
     nhay cham (1 Hz)  = WiFi OK, cho app tren may tinh
     sang lien         = dang gui du lieu cho app                                              */
#define SIMULATE_STM32      0           /* 1 = tu sinh khung gia 500 Hz (thu WiFi + PC khi chua noi STM32) */
#define DEBUG_STATS         1           /* 1 = in thong ke ra Serial (USB) moi 2 s */

/* ================= GIAO THUC (giong STM32 main.c + pc_monitor/protocol.py) ================= */
#define SOF1                0xAA
#define SOF2                0x55
#define MAX_PAYLOAD         64
#define UDP_MAX             1400        /* gioi han 1 goi UDP (duoi MTU) */

WiFiUDP udp;

static IPAddress s_client_ip;
static uint16_t  s_client_port = 0;     /* 0 = chua co PC */
static uint32_t  s_last_hello_ms = 0;

static uint8_t   s_tx[UDP_MAX];         /* cac khung da kiem tra, cho gui sang PC */
static size_t    s_tx_len = 0;

static const char *s_reset_reason = "?";

static const char *resetReasonText(esp_reset_reason_t r)
{
    switch (r)
    {
    case ESP_RST_POWERON:   return "POWERON";
    case ESP_RST_EXT:       return "EXT_PIN";
    case ESP_RST_SW:        return "SOFTWARE";
    case ESP_RST_PANIC:     return "PANIC";
    case ESP_RST_INT_WDT:   return "INT_WDT";
    case ESP_RST_TASK_WDT:  return "TASK_WDT";
    case ESP_RST_WDT:       return "WDT";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    case ESP_RST_BROWNOUT:  return "BROWNOUT";   /* SUT AP NGUON */
    default:                return "OTHER";
    }
}

/* thong ke */
static uint32_t  st_frames_ok = 0, st_frames_bad = 0, st_udp_sent = 0, st_uart_bytes = 0;

/* ------------------------------------------------------------ */
static uint8_t crc8(const uint8_t *p, size_t n)
{
    uint8_t crc = 0;
    for (size_t i = 0; i < n; i++)
    {
        crc ^= p[i];
        for (uint8_t b = 0; b < 8; b++)
        {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

static void flushToPc()
{
    if (s_tx_len == 0) return;
    if (s_client_port != 0 && WiFi.status() == WL_CONNECTED)
    {
        udp.beginPacket(s_client_ip, s_client_port);
        udp.write(s_tx, s_tx_len);
        udp.endPacket();
        st_udp_sent++;
    }
    s_tx_len = 0;
}

static void queueFrame(const uint8_t *f, size_t n)
{
    if (s_tx_len + n > sizeof(s_tx)) flushToPc();
    memcpy(&s_tx[s_tx_len], f, n);
    s_tx_len += n;
}

/* ------------------ Tach khung tu dong byte UART ------------------ */
enum ParseState { P_SOF1, P_SOF2, P_TYPE, P_LEN, P_PAYLOAD, P_CRC };
static ParseState s_ps = P_SOF1;
static uint8_t    s_frame[4 + MAX_PAYLOAD + 1];
static uint8_t    s_pidx = 0;

static void parseByte(uint8_t b)
{
    switch (s_ps)
    {
    case P_SOF1:
        if (b == SOF1) { s_frame[0] = b; s_ps = P_SOF2; }
        break;
    case P_SOF2:
        if (b == SOF2)      { s_frame[1] = b; s_ps = P_TYPE; }
        else if (b != SOF1) { s_ps = P_SOF1; }
        break;
    case P_TYPE:
        s_frame[2] = b; s_ps = P_LEN;
        break;
    case P_LEN:
        if (b > MAX_PAYLOAD) { st_frames_bad++; s_ps = P_SOF1; break; }
        s_frame[3] = b; s_pidx = 0;
        s_ps = (b == 0) ? P_CRC : P_PAYLOAD;
        break;
    case P_PAYLOAD:
        s_frame[4 + s_pidx++] = b;
        if (s_pidx >= s_frame[3]) s_ps = P_CRC;
        break;
    case P_CRC:
    {
        size_t n = 4 + s_frame[3];                 /* sof1..het payload */
        if (crc8(&s_frame[2], n - 2) == b)
        {
            s_frame[n] = b;
            queueFrame(s_frame, n + 1);
            st_frames_ok++;
        }
        else
        {
            st_frames_bad++;
        }
        s_ps = P_SOF1;
        break;
    }
    }
}

/* ------------------ Gia lap STM32 (SIMULATE_STM32 = 1) ------------------ */
#if SIMULATE_STM32
static void simulateStm32()
{
    static uint32_t next_us = 0, tick = 0;
    uint32_t now = micros();
    if ((int32_t)(now - next_us) < 0) return;
    next_us = now + 2000;                          /* 500 khung/s nhu STM32 */
    tick++;

    float t     = tick * 0.002f;
    float angle = 3.0f * sinf(2.0f * PI * 0.7f * t);
    float ref   = 40.0f * sinf(2.0f * PI * 0.5f * t);
    float rpm1  = ref + 4.0f * sinf(2.0f * PI * 3.0f * t);
    float rpm2  = ref - 3.0f * sinf(2.0f * PI * 2.0f * t);
    int16_t d1  = (int16_t)(rpm1 * 80.0f), d2 = (int16_t)(rpm2 * 80.0f);
    uint8_t f[31];
    f[0] = SOF1; f[1] = SOF2; f[2] = 0x01; f[3] = 26;
    memcpy(&f[4], &tick, 4);   memcpy(&f[8], &angle, 4); memcpy(&f[12], &ref, 4);
    memcpy(&f[16], &rpm1, 4);  memcpy(&f[20], &rpm2, 4);
    memcpy(&f[24], &d1, 2);    memcpy(&f[26], &d2, 2);
    f[28] = 0x01 | (3 << 4);   f[29] = 0;
    f[30] = crc8(&f[2], 28);
    for (int i = 0; i < 31; i++) parseByte(f[i]);
}
#endif

/* ------------------ Goi UDP tu PC ------------------ */
static void handleUdpIn(uint32_t now_ms)
{
    int size;
    while ((size = udp.parsePacket()) > 0)
    {
        uint8_t in[256];
        int n = udp.read(in, sizeof(in));
        if (n <= 0) continue;

        if (n >= 5 && memcmp(in, "HELLO", 5) == 0)
        {
            s_client_ip     = udp.remoteIP();
            s_client_port   = udp.remotePort();
            s_last_hello_ms = now_ms;
        }
        else if (n >= 4 && memcmp(in, "PING", 4) == 0)
        {
            in[1] = 'O';                            /* PING... -> PONG... giu nguyen phan con lai */
            udp.beginPacket(udp.remoteIP(), udp.remotePort());
            udp.write(in, n);
            udp.endPacket();
        }
        else if (n >= 2 && in[0] == SOF1 && in[1] == SOF2)
        {
            STM32_UART.write(in, n);                /* lenh PC -> STM32 */
        }
    }
}

static void housekeeping(uint32_t now_ms)
{
    static uint32_t last_beacon = 0, last_stats = 0;

    if (s_client_port != 0 && (now_ms - s_last_hello_ms) > CLIENT_TIMEOUT_MS)
    {
        s_client_port = 0;                          /* PC da tat app */
    }

    if (WiFi.status() == WL_CONNECTED && (now_ms - last_beacon) >= BEACON_PERIOD_MS)
    {
        last_beacon = now_ms;
        udp.beginPacket(WiFi.broadcastIP(), UDP_PORT_PC);
        udp.printf("ROBOT_ESP32;rst=%s;up=%lu", s_reset_reason, (unsigned long)(now_ms / 1000));
        udp.endPacket();
    }

#if LED_PIN >= 0
    /* LED: nhay nhanh = chua co WiFi, nhay cham = cho PC, sang = dang gui cho PC */
    bool on;
    if (WiFi.status() != WL_CONNECTED) on = (now_ms / 100) & 1;
    else if (s_client_port == 0)       on = (now_ms / 500) & 1;
    else                               on = true;
    digitalWrite(LED_PIN, on ? HIGH : LOW);
#endif

#if DEBUG_STATS
    if ((now_ms - last_stats) >= 2000)
    {
        last_stats = now_ms;
        Serial.printf("[%lus] WiFi=%s IP=%s | UART %lu B, khung OK %lu, loi %lu | UDP %lu goi | PC: %s:%u\n",
                      (unsigned long)(now_ms / 1000),
                      WiFi.status() == WL_CONNECTED ? "OK" : "--",
                      WiFi.localIP().toString().c_str(),
                      (unsigned long)st_uart_bytes, (unsigned long)st_frames_ok,
                      (unsigned long)st_frames_bad, (unsigned long)st_udp_sent,
                      s_client_port ? s_client_ip.toString().c_str() : "chua co",
                      s_client_port);
    }
#endif
}

void setup()
{
    Serial.begin(115200);
    s_reset_reason = resetReasonText(esp_reset_reason());
#if LED_PIN >= 0
    pinMode(LED_PIN, OUTPUT);
    for (int i = 0; i < 3; i++)                     /* bao "vua khoi dong" */
    {
        digitalWrite(LED_PIN, HIGH); delay(60);
        digitalWrite(LED_PIN, LOW);  delay(60);
    }
#endif
    Serial.printf("\nESP32 khoi dong, ly do reset lan truoc: %s\n", s_reset_reason);
    if (esp_reset_reason() == ESP_RST_BROWNOUT)
    {
        Serial.println("!!! SUT AP NGUON (brownout): nguon cap cho ESP32 khong du dong khi bat WiFi");
    }

    STM32_UART.setRxBufferSize(2048);
    STM32_UART.begin(STM32_BAUD, SERIAL_8N1, STM32_RX_PIN, STM32_TX_PIN);
    STM32_UART.setRxTimeout(1);                     /* dua du lieu len ngay sau 1 ky tu im lang */

    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);                           /* QUAN TRONG cho do tre thap */
#if WIFI_TX_POWER_LOW
    WiFi.setTxPower(WIFI_POWER_11dBm);
#endif
    WiFi.setHostname("robot-esp32");
    WiFi.setAutoReconnect(true);
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    Serial.print("Dang ket noi WiFi");
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - t0) < 20000)
    {
        delay(100);
#if LED_PIN >= 0
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));   /* nhay nhanh trong luc ket noi */
#endif
        if (((millis() - t0) % 1000) < 100) Serial.print(".");
    }
    Serial.println();
    if (WiFi.status() == WL_CONNECTED)
    {
        WiFi.setSleep(false);
        Serial.print("WiFi OK, IP ESP32: ");
        Serial.println(WiFi.localIP());
    }
    else
    {
        Serial.println("Chua ket noi duoc WiFi, se tu thu lai...");
    }
    udp.begin(UDP_PORT_ESP);
#if SIMULATE_STM32
    Serial.println("CHE DO GIA LAP: tu sinh du lieu STM32");
#endif
}

void loop()
{
    uint32_t now_ms = millis();

#if SIMULATE_STM32
    simulateStm32();
#else
    uint8_t buf[256];
    int avail;
    while ((avail = STM32_UART.available()) > 0)
    {
        size_t n = STM32_UART.read(buf, (avail > (int)sizeof(buf)) ? sizeof(buf) : (size_t)avail);
        st_uart_bytes += n;
        for (size_t i = 0; i < n; i++) parseByte(buf[i]);
    }
#endif

    flushToPc();            /* gui ngay, khong gom theo thoi gian */
    handleUdpIn(now_ms);
    housekeeping(now_ms);
}
