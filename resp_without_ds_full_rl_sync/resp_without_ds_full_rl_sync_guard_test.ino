// BY NU IOT LAB //
// Authors: Damir Assylbek, Aizhuldyz Nadirkhanova, Dimitrios Zormpas //
// GPL-3.0 license //  
// REFERENCES:
// rssi: https://github.com/TenoTrash/ESP32_ESPNOW_RSSI/blob/main/Modulo_Receptor_OLED_SPI_RSSI.ino

#include <esp_wifi.h>
#include <esp_now.h>
#include <WiFi.h>
#include "sarsa.h"

#include <OneWire.h>
#include <DallasTemperature.h>
#define ONE_WIRE_BUS 4
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
float temperature = 0.0;

#define TIMEOUT_THRESHOLD 35
#define TOTAL_PACKETS 500
uint8_t masterAddress[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
int packetReceived = 0;
int packetMissed = 0;
bool SYNCED = false;
//flag
wifi_country_t country = {
  .cc = "JP",
  .schan = 1,
  .nchan = 14,
  .max_tx_power = 20,
  .policy = WIFI_COUNTRY_POLICY_AUTO,
};

esp_now_peer_info_t masterInfo;

typedef struct struct_message { 
  unsigned long time;
  unsigned int packetNumber;
} struct_message;

typedef struct {
  String timestamp;
  int packetNumber;
  bool received;
  float successRate;
  int radioOnTime;
  int data; 
  int length; 
  String madeAdjustment; 
  int newDelay; 
  int rssi_display;
  float temperature;
  String madeGuardAdjustment;
  int new_guard_time;
} PacketLog;

/////////////////////////////////////   RSSI  //////////////////////////////////////

int rssi_display;
int totalRssi = 0;
// Estructuras para calcular los paquetes, el RSSI, etc
typedef struct {
  unsigned frame_ctrl: 16;
  unsigned duration_id: 16;
  uint8_t addr1[6]; /* receiver address */
  uint8_t addr2[6]; /* sender address */
  uint8_t addr3[6]; /* filtering address */
  unsigned sequence_ctrl: 16;
  uint8_t addr4[6]; /* optional */
} wifi_ieee80211_mac_hdr_t;

typedef struct {
  wifi_ieee80211_mac_hdr_t hdr;
  uint8_t payload[0]; /* network data ended with 4 bytes csum (CRC32) */
} wifi_ieee80211_packet_t;

//La callback que hace la magia
void promiscuous_rx_cb(void *buf, wifi_promiscuous_pkt_type_t type) {
  // All espnow traffic uses action frames which are a subtype of the mgmnt frames so filter out everything else.
  if (type != WIFI_PKT_MGMT)
    return;

  const wifi_promiscuous_pkt_t *ppkt = (wifi_promiscuous_pkt_t *)buf;
  const wifi_ieee80211_packet_t *ipkt = (wifi_ieee80211_packet_t *)ppkt->payload;
  const wifi_ieee80211_mac_hdr_t *hdr = &ipkt->hdr;

  int rssi = ppkt->rx_ctrl.rssi;
  rssi_display = rssi;
}

//////////////////////////////////// END RSSI /////////////////////////////////

String formatTimestamp(unsigned long totalMillis) {
  unsigned long totalSeconds = totalMillis / 1000;
  unsigned long totalMinutes = totalSeconds / 60;
  unsigned long totalHours = totalMinutes / 60;

  unsigned long ms = totalMillis % 1000;
  unsigned long seconds = totalSeconds % 60;
  unsigned long minutes = totalMinutes % 60;
  unsigned long hours = totalHours % 24;  // Uncomment if you want the hour to reset every 24 hours

  char timestamp[20];
  sprintf(timestamp, "%02lu:%02lu:%02lu.%03lu", hours, minutes, seconds, ms);

  return String(timestamp);
}

PacketLog packetLog;
struct_message myData;
bool first_packet = true;
unsigned long l_delay = 0;
unsigned long lastPacketTime = 0;
// Create an instance of the SARSA class
int numStates = 5;           // Set the number of states
int numActions = 11;           // Set the number of actions
int actions[] = {-10, -5, -3, -2, -1, 2, 1, 3, 5, 10, 0};
float learningRate = 0.1;     // Set the learning rate (alpha)
float explorationRate = 1.0;  // Set the exploration rate (epsilon)
float min_epsilon = 0.05; // min possible exploration rate  
float decay_rate = 0.001; // decay rate of the epsilon
bool update = false; // to update the Q-table
bool timer = false;
bool ON = false;
bool miss_p = false;
unsigned long start_time = 0;
unsigned long end_time = 0;
unsigned long calib_start = 0;
bool calib = true;
bool recalib = false;
int prev_pkt = 0;
int prev_pkt_recab = 0;
int lst_missed_packet = 0;
int diff_count = 0;
float success_rate = 0;
int delta_min = 10;
int lst_pack_time = 0;
int cons_pack = 0; 
int hist_of_miss = 0;
// int last_pkt_guard = 0; 
unsigned long wifiOnTime = 0; 
SARSA sarsa(numStates, numActions, learningRate, explorationRate);

void printPacketLog() {
  Serial.print(packetLog.timestamp);
  Serial.print(" | ");
  Serial.print(packetLog.packetNumber);
  Serial.print("  | ");
  Serial.print(packetLog.received ? "Yes" : "TIMEOUT");
  Serial.print(" | ");
  Serial.print(packetLog.successRate, 2);  
  Serial.print(" | ");
  Serial.print(packetLog.radioOnTime);
  Serial.print(" ms  | ");
  Serial.print(packetLog.data);  
  Serial.print(" | ");
  Serial.print(packetLog.length); 
  Serial.print(" | ");
  Serial.print(packetLog.rssi_display);
  Serial.print(" | ");
  Serial.print(packetLog.temperature);
  Serial.print(" | ");
  if(packetLog.madeAdjustment != "") {
    Serial.print(packetLog.madeAdjustment); 
    Serial.print("  | ");
    Serial.print(packetLog.newDelay);
    Serial.print("  | ");
  } else {
    Serial.print("N/A | ");  // If no adjustment, print N/A and leave the new delay column blank
  }
  if (packetLog.madeGuardAdjustment != "") {
    Serial.print(packetLog.madeGuardAdjustment);
    Serial.print(" |");
    Serial.print(packetLog.new_guard_time);
  }
  else {
    Serial.print("N/A | ");  // If no adjustment, print N/A and leave the new delay column blank
  }
  Serial.println();
}

void MissedPacket() {
  packetMissed++; 
  lst_missed_packet = packetReceived + packetMissed;
  prev_pkt_recab = lst_missed_packet;
  int pkt_dif = (packetMissed + packetReceived) - prev_pkt_recab;
  // Serial.printf("Difference in missed\n", pkt_dif); 
  if (prev_pkt_recab != 0 && pkt_dif > 1 ) {
    calib_start = 0;
    calib = true;
    lst_missed_packet = 0;
    lst_pack_time = 0;
    currentState = 0;
    nextState = 0;
    start_time = 0;
    ON = false;
    miss_p = false;
    l_delay = 0;
    packetLog.radioOnTime = 0;
    packetLog.packetNumber = packetMissed + packetReceived; 
    packetLog.received = false; 
    printPacketLog();

    Serial.println("Recalibrating... in missed"); 
    packetReceived = 0;
    packetMissed = 0;
    cons_pack = 0;
    prev_pkt_recab = 0;
    pkt_dif = 0;
    recalib = true;
  }
  else {  
    int currentState = 4; //missed the packet and we need to change the guard time
    int nextState = 2;
    int currentAction = sarsa.selectAction(currentState);
    float reward = 0.0;     // Set the reward
    if(actions[currentAction] < 0) {
      currentAction += 2;
    }
    int nextAction = sarsa.selectAction(nextState);     // Set the next action
    if((delta_min+actions[currentAction]) < 30){
      delta_min += actions[currentAction];
      sarsa.updateQValue(currentState, currentAction, reward, nextState, nextAction);
      packetLog.madeGuardAdjustment = String("+") + String(actions[currentAction]);
    }
    miss_p = true;
    success_rate = float(packetReceived) / float(packetReceived + packetMissed);
    currentState = 2; 
    currentAction = sarsa.selectAction(currentState);  // Set the current action 
    l_delay += actions[currentAction];
    reward = 0.0;     // Set the reward
    long delta2 = millis() - start_time;
    if(delta2 - actions[currentAction] > 100) {
      reward = 1.0;
    } 
    nextState = 1;      // Set the next state 
    nextAction = sarsa.selectAction(nextState);     // Set the next action
    sarsa.updateQValue(currentState, currentAction, reward, nextState, nextAction);
    explorationRate = std::max(min_epsilon, (1 - min_epsilon) * std::exp(-decay_rate * (packetReceived + packetMissed)));
    packetLog.madeAdjustment = String(actions[currentAction]);
    packetLog.newDelay = l_delay;
    packetLog.timestamp = formatTimestamp(millis()); 
    packetLog.packetNumber = packetMissed + packetReceived; 
    packetLog.received = false; 
    packetLog.successRate = success_rate * 100;
    packetLog.radioOnTime = delta2; 
    packetLog.temperature = temperature;
    packetLog.new_guard_time = delta_min;
    printPacketLog();
    lst_pack_time = millis();
  }
}

void Calibration() {
  l_delay = (millis() - calib_start - 25) / (myData.packetNumber - prev_pkt);
  calib = false;
  timer = true;
  start_time = 0;
  recalib = false;
  }

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  lst_pack_time = 0;
  cons_pack = 0;
  bool upd = false;
  packetLog.temperature = temperature;
  packetLog.rssi_display = rssi_display; 
  totalRssi += rssi_display;
  // Turn off Wi-Fi module
  if (esp_wifi_stop() != ESP_OK) {
    Serial.println("Troubles with stop function");
  }
  end_time = millis(); // for metrics
  int currentState = 1;
  packetReceived++;
  
  memcpy(&myData, incomingData, sizeof(myData));
  int pkt_dif = myData.packetNumber - prev_pkt_recab; 
  if ((prev_pkt_recab != 0 && pkt_dif > 1) || (myData.packetNumber < lst_missed_packet)) {
    lst_missed_packet = 0;
    calib_start = 0;
    calib = true;
    lst_pack_time = 0;
    currentState = 0;
    nextState = 0;
    start_time = 0;
    ON = false;
    miss_p = false;
    l_delay = 0;
    packetLog.radioOnTime = 0;
    Serial.println("Recalibrating... in ondata"); 
    cons_pack = 0;
    prev_pkt_recab = 0;
    pkt_dif = 0;
    recalib = true;
    
  } 
  success_rate = float(packetReceived) / float(packetReceived + packetMissed);
  if(calib) {
    packetReceived = myData.packetNumber; 
    currentState = 0; 
    if (calib_start > 0) {
      Calibration();
    } 
    prev_pkt = myData.packetNumber; 
    calib_start = millis();
  }  
  if(timer) {
    ON = false;
  }
  unsigned long delta = end_time - start_time;
  wifiOnTime += delta;
  prev_pkt_recab = myData.packetNumber; // 4
  if(start_time != 0 && !calib) {
    packetLog.radioOnTime = end_time - start_time;
  }
  // Update Q-values using SARSA
  float timer_sarsa_start = micros();
  float reward = 0.0;   
  if(long(delta) > delta_min && !calib && start_time != 0 && !miss_p) { 
    int currentAction = sarsa.selectAction(currentState);  // Set the current action 
    if(long(delta - actions[currentAction]) > delta_min / 2) {
      l_delay += actions[currentAction];
      reward = 1.0;
      int nextState = 1;      // Set the next state 
      int nextAction = sarsa.selectAction(nextState);     // Set the next action
      sarsa.updateQValue(currentState, currentAction, reward, nextState, nextAction);
      packetLog.madeAdjustment = String("+") + String(actions[currentAction]);
      packetLog.newDelay = l_delay;
      upd = true;
      }
  }
  else if(long(delta) <= delta_min && long(delta) > delta_min / 2 && !calib && start_time != 0 && !miss_p) {
    currentState = 3;
    int currentAction = sarsa.selectAction(currentState);
     
    if(actions[currentAction] < 0) {
      currentAction += 2;
    }
    if(long(delta - actions[currentAction]) > delta_min / 2 - 2 ) { //FOR POSITIVE VALUES
      l_delay += actions[currentAction];
      reward = 1.0;
    }
    int nextState = 3;      // Set the next state 
    int nextAction = sarsa.selectAction(nextState);     // Set the next action
    sarsa.updateQValue(currentState, currentAction, reward, nextState, nextAction);
    packetLog.madeAdjustment = String("+") + String(actions[currentAction]);
    packetLog.newDelay = l_delay;
    upd = true;
  }
  else if(long(delta) < delta_min / 2 - 1  && !calib && start_time != 0 && !miss_p) {
    currentState = 3;
    int currentAction = sarsa.selectAction(currentState); 
    // FOR NEGATIVE VALUES 
    if (actions[currentAction] > 0) {
        currentAction -= 2;
    }
    if(long(delta) + actions[currentAction] < delta_min / 2 + 2) {
      l_delay += actions[currentAction];
      reward = 1.0;
    }
    int nextState = 3;      // Set the next state 
    int nextAction = sarsa.selectAction(nextState);     // Set the next action
    sarsa.updateQValue(currentState, currentAction, reward, nextState, nextAction);
    packetLog.madeAdjustment = String(actions[currentAction]);
    packetLog.newDelay = l_delay;
    upd = true;
  }

  explorationRate = std::max(min_epsilon, (1 - min_epsilon) * std::exp(-decay_rate * (packetReceived + packetMissed)));
  miss_p = false;
  delay(l_delay);  // Adjust the delay time as needed 
  // Reinitialize Wi-Fi module
  WiFi.mode(WIFI_STA);
  esp_wifi_start();
  esp_wifi_set_channel(14, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_country(&country);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_promiscuous_rx_cb(&promiscuous_rx_cb);
  packetLog.timestamp = formatTimestamp(millis()); 
  packetLog.packetNumber = myData.packetNumber;
  packetLog.received = true; 
  packetLog.data = myData.time; 
  packetLog.length = len; 
  packetLog.successRate = success_rate * 100;
  if (!upd) {
    packetLog.madeAdjustment = "";
    packetLog.madeGuardAdjustment = "";
  }
  printPacketLog();
  start_time = millis();
  if(timer) {
    ON = true;
  }
}

void printFinalStatistics() {
    float finalSuccessRate = 0;
    if (packetReceived + packetMissed != 0) {
        finalSuccessRate = (float)packetReceived / (packetReceived + packetMissed) * 100;
    }
    Serial.println("\n------ Final Statistics ------");
    Serial.print("Total Packets Received: ");
    Serial.println(packetReceived);
    Serial.print("Total Packets Missed: ");
    Serial.println(packetMissed);
    Serial.print("Final Success Rate: ");
    Serial.print(finalSuccessRate);
    Serial.println("%");
    if (packetReceived > 0) {  // Check to avoid division by zero
        float averageRssi = float(totalRssi) / packetReceived;
        Serial.print("Average RSS: ");
        Serial.println(averageRssi);
    }
    if (packetReceived > 0) {  // Check to avoid division by zero
        float avgtemperature = float(temperature) / packetReceived;
        Serial.print("Average Temperature: ");
        Serial.println(avgtemperature);
    }
    Serial.println("--------------------------------");
}

void setup() {
  sensors.begin();
  Serial.begin(115200);
  delay(1000);
  packetReceived = 0;

  WiFi.mode(WIFI_STA);
  

  Serial.println("  Timestamp  |Pct#| RCV |RAD(ms)|SCS rt|Data|Len|RSSI|Adj|New delay|");
  Serial.println("--------------------------------------------------------------------");

  if (esp_now_init() != ESP_OK) {
    Serial.println("There was an error initializing ESP-NOW");
    return;
  }
  memcpy(masterInfo.peer_addr, masterAddress, 6);
  masterInfo.channel = 14;
  esp_wifi_set_channel(14, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_country(&country);
  
  if (esp_now_add_peer(&masterInfo) != ESP_OK) {
    Serial.println("There was an error registering the master");
    return;
  }
  int currentState = 0;
  esp_now_register_recv_cb(OnDataRecv);
  start_time = millis();

  esp_wifi_set_promiscuous(true);
  esp_wifi_set_promiscuous_rx_cb(&promiscuous_rx_cb);
}

void loop() {
  if (packetReceived + packetMissed >= TOTAL_PACKETS) {
      printFinalStatistics();
      while (1);  // Infinite loop to halt the program, or you can add your own logic to end it
  }
  long timeSinceLastPacket = millis() - start_time;
  if (cons_pack >= 3) {
      calib_start = 0;
      calib = true;
      lst_pack_time = 0;
      currentState = 0;
      nextState = 0;
      start_time = 0;
      ON = false;
      miss_p = false;
      packetLog.radioOnTime = 0;
      Serial.println("Recalibrating... in loop"); 
      cons_pack = 0;
      recalib = true;
  }
  if (timeSinceLastPacket > TIMEOUT_THRESHOLD && ON && !recalib) {
    MissedPacket();
    ON = false;
    cons_pack++;
  } 
  else if(start_time != 0 && lst_pack_time > 0 && millis() - lst_pack_time > l_delay && !recalib) {
    cons_pack++;
    MissedPacket();
    Serial.printf("Consecutive lost packets: %i\n", cons_pack);
  }
}