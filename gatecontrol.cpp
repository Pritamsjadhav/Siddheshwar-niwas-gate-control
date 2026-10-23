#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "esp_gap_ble_api.h"
#include <esp_task_wdt.h>

// --- Watchdog Timer Timeout (Seconds) ---
#define WDT_TIMEOUT_SEC 5

// --- Native Android Popup Passkey (6 digits) ---
const uint32_t STATIC_PASSKEY = 997525; 

// --- 433 MHz Transmitter Config ---
const int TX_PIN = 13; 
const int PULSE_LENGTH = 320;

// 24-bit EV1527 Remote Codes
const uint32_t CH1_CODE = 0x123452; // Gate CLOSE
const uint32_t CH2_CODE = 0x123451; // Gate OPEN
const uint32_t CH3_CODE = 0x123454; // Motor Stop
const uint32_t CH4_CODE = 0x123458; // Lamps

uint32_t activeCode = 0;
unsigned long runStartTime = 0;
const unsigned long AUTO_TIMEOUT_MS = 30000;
bool triggerLampPulse = false;

// Standard Nordic UART Service UUIDs
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

BLECharacteristic *pTxCharacteristic;
int connectedClientsCount = 0;
BLEServer *pServerInstance = nullptr;

// Hardware-verified EV1527 frame timing
void sendEV1527(uint32_t code) {
  digitalWrite(TX_PIN, HIGH);
  delayMicroseconds(PULSE_LENGTH * 1);
  digitalWrite(TX_PIN, LOW);
  delayMicroseconds(PULSE_LENGTH * 31);

  for (int i = 23; i >= 0; i--) {
    if ((code >> i) & 1) {
      digitalWrite(TX_PIN, HIGH);
      delayMicroseconds(PULSE_LENGTH * 3);
      digitalWrite(TX_PIN, LOW);
      delayMicroseconds(PULSE_LENGTH * 1);
    } else {
      digitalWrite(TX_PIN, HIGH);
      delayMicroseconds(PULSE_LENGTH * 1);
      digitalWrite(TX_PIN, LOW);
      delayMicroseconds(PULSE_LENGTH * 3);
    }
  }
  digitalWrite(TX_PIN, LOW);
}

void sendClockPulse(uint32_t code, unsigned long durationMs) {
  unsigned long start = millis();
  while (millis() - start < durationMs) {
    sendEV1527(code);
    esp_task_wdt_reset(); // Feed WDT during long pulses
  }
  digitalWrite(TX_PIN, LOW);
}

void sendBleMessage(const char* msg) {
  if (connectedClientsCount > 0 && pTxCharacteristic != nullptr) {
    pTxCharacteristic->setValue((uint8_t*)msg, strlen(msg));
    pTxCharacteristic->notify();
  }
  Serial.print(msg);
}

// Connection Callbacks for Multi-Device Support & Continuous Advertising
class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer, esp_ble_gatts_cb_param_t *param) override {
    connectedClientsCount++;
    Serial.printf("[BLE] Client connected. Total active: %d. Enforcing security...\n", connectedClientsCount);
    
    pServer->updateConnParams(param->connect.remote_bda, 0x06, 0x12, 0, 500);
    esp_ble_set_encryption(param->connect.remote_bda, ESP_BLE_SEC_ENCRYPT_MITM);

    // Keep broadcasting advertising so other phones can still discover "Siddheshwar Niwas"
    pServer->getAdvertising()->start();
  }

  void onDisconnect(BLEServer* pServer) override {
    connectedClientsCount = max(0, connectedClientsCount - 1);
    activeCode = 0;
    digitalWrite(TX_PIN, LOW);
    Serial.printf("[BLE] Client disconnected. Total active: %d. Ensuring advertising is active...\n", connectedClientsCount);
    
    delay(100);
    pServer->getAdvertising()->start();
  }
};

// Command Receiver Callbacks
class MyRxCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) override {
    String rxValue = pCharacteristic->getValue(); // Fixed for ESP32 Core v3.x String type
    if (rxValue.length() > 0) {
      for (int i = 0; i < rxValue.length(); i++) {
        char cmd = rxValue[i];
        if (cmd == '\r' || cmd == '\n' || cmd == ' ') continue;

        switch (cmd) {
          case '1': // Open
            if (activeCode == CH1_CODE) {
              activeCode = 0;
              digitalWrite(TX_PIN, LOW);
              sendBleMessage("OK: OPEN Stopped\r\n");
            } else {
              activeCode = CH1_CODE;
              runStartTime = millis();
              sendBleMessage("RUN: Gate Opening\r\n");
            }
            break;

          case '2': // Close
            if (activeCode == CH2_CODE) {
              activeCode = 0;
              digitalWrite(TX_PIN, LOW);
              sendBleMessage("OK: CLOSE Stopped\r\n");
            } else {
              activeCode = CH2_CODE;
              runStartTime = millis();
              sendBleMessage("RUN: Gate Closing\r\n");
            }
            break;

          case '3': // Stop
            activeCode = 0;
            digitalWrite(TX_PIN, LOW);
            sendBleMessage("OK: Motor Stopped\r\n");
            break;

          case '4': // Lamps
            triggerLampPulse = true;
            sendBleMessage("OK: Lamp Toggled\r\n");
            break;

          case '0':
          case 's':
          case 'S':
            activeCode = 0;
            digitalWrite(TX_PIN, LOW);
            sendBleMessage("OK: All Stopped\r\n");
            break;

          default:
            break;
        }
      }
    }
  }
};

void setup() {
  Serial.begin(115200);
  pinMode(TX_PIN, OUTPUT);
  digitalWrite(TX_PIN, LOW);

  // 1. Initialize Watchdog Timer
  Serial.println("[WDT] Initializing Watchdog Timer...");
  esp_task_wdt_deinit();
  esp_task_wdt_config_t twdt_config = {
    .timeout_ms = WDT_TIMEOUT_SEC * 1000,
    .idle_core_mask = (1 << 0),
    .trigger_panic = true
  };
  esp_task_wdt_init(&twdt_config);
  esp_task_wdt_add(NULL); 

  // 2. Initialize BLE Device with Max Hardware RF Power (+9 dBm)
  BLEDevice::init("Siddheshwar Niwas");
  
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_P9);
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_SCAN, ESP_PWR_LVL_P9);
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_P9);

  // 3. Configure Low-Level GAP Security for Static Passkey Prompt
  uint32_t passkey = STATIC_PASSKEY; 
  esp_ble_auth_req_t auth_req = ESP_LE_AUTH_REQ_SC_MITM_BOND;
  esp_ble_io_cap_t iocap = ESP_IO_CAP_OUT; 
  uint8_t key_size = 16;
  uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
  uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
  uint8_t auth_option = ESP_BLE_ONLY_ACCEPT_SPECIFIED_AUTH_ENABLE;

  esp_ble_gap_set_security_param(ESP_BLE_SM_SET_STATIC_PASSKEY, &passkey, sizeof(uint32_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_ONLY_ACCEPT_SPECIFIED_SEC_AUTH, &auth_option, sizeof(uint8_t));

  // 4. Create BLE Server & UART Service
  pServerInstance = BLEDevice::createServer();
  pServerInstance->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServerInstance->createService(SERVICE_UUID);

  pTxCharacteristic = pService->createCharacteristic(
                        CHARACTERISTIC_UUID_TX,
                        BLECharacteristic::PROPERTY_NOTIFY
                      );
  pTxCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(
                                           CHARACTERISTIC_UUID_RX,
                                           BLECharacteristic::PROPERTY_WRITE |
                                           BLECharacteristic::PROPERTY_WRITE_NR
                                         );
  pRxCharacteristic->setAccessPermissions(ESP_GATT_PERM_WRITE_ENC_MITM);
  pRxCharacteristic->setCallbacks(new MyRxCallbacks());

  pService->start();

  // 5. Configure High-Performance Advertising for Maximum Distance & Continuous Visibility
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinInterval(0x20); // Fast interval (~20ms)
  pAdvertising->setMaxInterval(0x40); // Fast interval (~40ms)
  
  BLEDevice::startAdvertising();

  Serial.println("=========================================");
  Serial.println("Siddheshwar Niwas BLE Controller Online");
  Serial.printf("Transmitter Pin: GPIO %d\n", TX_PIN);
  Serial.println("Continuous Adv Mode: +9dBm, Passkey 997525, Multi-Client Ready");
  Serial.println("=========================================");
}

void loop() {
  esp_task_wdt_reset();

  if (triggerLampPulse) {
    triggerLampPulse = false;
    sendClockPulse(CH4_CODE, 350);
  }

  // 30-Second Continuous Safety Shutoff
  if (activeCode != 0) {
    if (millis() - runStartTime < AUTO_TIMEOUT_MS) {
      sendEV1527(activeCode);
    } else {
      activeCode = 0;
      digitalWrite(TX_PIN, LOW);
      sendBleMessage("ALERT: 30s Auto-Shutoff Engaged\r\n");
    }
  }
}
