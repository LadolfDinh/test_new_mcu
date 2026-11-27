/*
  ESP32: nhan goc + toc do (rad/s) tu STM32 qua UART2, sau do gui qua WiFi (UDP)
  cho may tinh de ve bieu do thoi gian thuc bang Python.

  ***** CHE DO GIA LAP (khi STM32 dang hong / chua noi) *****
  Dat SIMULATE_DATA = 1 de tu sinh du lieu goc/toc do dang sin/cos gia,
  khong can doc UART that. Khi STM32 sua xong, dat lai SIMULATE_DATA = 0
  de dung du lieu that nhan tu UART.

  QUAN TRONG - can sua truoc khi nap:
  - WIFI_SSID, WIFI_PASS   : wifi ma ca ESP32 va PC dang chay Python cung ket noi
  - PC_UDP_IP              : dia chi IP cua may tinh chay script Python (vd 192.168.1.10)
  - PC_UDP_PORT            : phai trung voi UDP_PORT trong script Python

  Dinh dang goi tin gui qua WiFi: chuoi text "goc,toc_do\n" (vd "3.245,1.802\n")
  De don gian, khong dung nhi phan nhu ben UART STM32<->ESP32.
*/

#include <WiFi.h>
#include <WiFiUdp.h>
#include <string.h>

/* ================= CAU HINH WIFI - SUA LAI CHO DUNG ================= */
#include "secrets.h"   /* WIFI_SSID, WIFI_PASS - xem secrets.example.h */

const char *PC_UDP_IP   = "192.168.1.136";   /* IP cua may tinh chay Python */
const uint16_t PC_UDP_PORT = 4210;

/* ================= CHE DO GIA LAP DU LIEU ================= */
#define SIMULATE_DATA   0    /* 1 = gia lap (STM32 dang hong), 0 = dung UART that */

/* ---- Chan UART2 nhan tu STM32 ---- */
#define UART_RX_PIN     16
#define UART_TX_PIN     17
#define UART_BAUDRATE   115200

/* Khung STM32 -> ESP32: SOH1 SOH2 | goc(4, do) toc_do(4, rad/s) | checksum(1) */
#define RX_SOH1            0xAA
#define RX_SOH2            0x55
#define RX_PAYLOAD_LEN      8

/* ---- Du lieu hien tai (that hoac gia lap) ---- */
volatile float g_angle_deg   = 0.0f;
volatile float g_speed_radps = 0.0f;

volatile bool g_rx_flag = false;

enum RxState { WAIT_SOH1, WAIT_SOH2, PAYLOAD };
RxState  rx_state = WAIT_SOH1;
uint8_t  rx_payload[RX_PAYLOAD_LEN + 1];
uint8_t  rx_index = 0;

WiFiUDP udp;

/* ============================================================
   Tinh checksum kieu XOR
   ============================================================ */
uint8_t calcChecksum(const uint8_t *data, uint16_t len)
{
    uint8_t cs = 0;
    for (uint16_t i = 0; i < len; i++)
    {
        cs ^= data[i];
    }
    return cs;
}

/* ============================================================
   Kiem tra checksum, cap nhat bien tu khung UART that nhan tu STM32
   ============================================================ */
void processRxFrame()
{
    if (calcChecksum(rx_payload, RX_PAYLOAD_LEN) != rx_payload[RX_PAYLOAD_LEN])
    {
        return;   /* sai checksum -> bo qua khung loi */
    }

    float angle, speed;
    memcpy(&angle, &rx_payload[0], 4);
    memcpy(&speed, &rx_payload[4], 4);

    g_angle_deg   = angle;
    g_speed_radps = speed;
}

/* ============================================================
   "Ngat" UART2 - goi khi co du lieu moi den (ESP32 core >= 2.0.1)
   ============================================================ */
void onUartReceive()
{
    while (Serial2.available())
    {
        uint8_t b = (uint8_t)Serial2.read();

        switch (rx_state)
        {
            case WAIT_SOH1:
                if (b == RX_SOH1) rx_state = WAIT_SOH2;
                break;

            case WAIT_SOH2:
                if (b == RX_SOH2) { rx_state = PAYLOAD; rx_index = 0; }
                else               rx_state = WAIT_SOH1;
                break;

            case PAYLOAD:
                rx_payload[rx_index++] = b;
                if (rx_index >= (RX_PAYLOAD_LEN + 1))
                {
                    g_rx_flag = true;
                    rx_state  = WAIT_SOH1;
                }
                break;
        }
    }
}

/* ============================================================
   Gui goc + toc do hien tai cho PC qua UDP, dinh dang text CSV
   ============================================================ */
void sendToPC()
{
    char buf[64];
    /* sao chep sang bien khong volatile truoc khi dung snprintf cho an toan */
    float angle = g_angle_deg;
    float speed = g_speed_radps;

    int len = snprintf(buf, sizeof(buf), "%.3f,%.3f\n", angle, speed);

    udp.beginPacket(PC_UDP_IP, PC_UDP_PORT);
    udp.write((const uint8_t *)buf, len);
    udp.endPacket();
}

#if SIMULATE_DATA
/* ============================================================
   Gia lap du lieu goc/toc do khi STM32 chua san sang.
   Goc dao dong dang sin +-15 do, toc do dao dong dang cos +-5 rad/s.
   ============================================================ */
void simulateData()
{
    float t = millis() / 1000.0f;   /* thoi gian tinh bang giay */

    g_angle_deg   = 15.0f * sinf(t * 1.0f);        /* dao dong cham, +-15 do */
    g_speed_radps = 5.0f  * cosf(t * 1.0f + 0.5f); /* lech pha 1 chut cho giong that */
}
#endif

void setup()
{
    Serial.begin(115200);

#if SIMULATE_DATA
    Serial.println("CHE DO GIA LAP: dang sinh du lieu goc/toc do gia (STM32 chua ket noi)");
#else
    Serial2.begin(UART_BAUDRATE, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
    Serial2.onReceive(onUartReceive, false);
    Serial.println("Dang doi du lieu that tu STM32 qua UART...");
#endif

    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.print("Dang ket noi WiFi");
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(300);
        Serial.print(".");
    }
    Serial.println();
    Serial.print("Da ket noi WiFi, IP cua ESP32: ");
    Serial.println(WiFi.localIP());
    Serial.print("Se gui du lieu UDP toi ");
    Serial.print(PC_UDP_IP);
    Serial.print(":");
    Serial.println(PC_UDP_PORT);

    udp.begin(PC_UDP_PORT);   /* mo socket UDP local (dung chung port de don gian) */
}

void loop()
{
#if SIMULATE_DATA
    static uint32_t last_sim_ms = 0;
    if (millis() - last_sim_ms >= 100)   /* cap nhat du lieu gia moi 100ms */
    {
        last_sim_ms = millis();
        simulateData();
        sendToPC();
    }
#else
    if (g_rx_flag)
    {
        g_rx_flag = false;
        processRxFrame();
        sendToPC();   /* co du lieu that moi -> gui ngay qua WiFi */
    }
#endif
}
