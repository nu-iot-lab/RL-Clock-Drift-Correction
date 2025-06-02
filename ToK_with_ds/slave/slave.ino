// BY NU IOT LAB //
// Authors: Damir Assylbek, Zhansaya Talaptan, Ardak Alipova, Nurzhan Kozhamuratov, Dimitrios Zormpas //
// GPL-3.0 license //
// REFERENCES:
// rssi: https://github.com/TenoTrash/ESP32_ESPNOW_RSSI/blob/main/Modulo_Receptor_OLED_SPI_RSSI.ino

#include <esp_wifi.h>
#include <esp_now.h>
#include <WiFi.h>
#include "sarsa.h"
#include "huffman.h"
#include <stdlib.h> 
#include <time.h> 

#define TIMEOUT_THRESHOLD 150
// for 5 sec: 150
// for 30 sec: 450

#define TOTAL_PACKETS 500
#define ESP_PPM 0.005
#define RSSI_BUFFER_SIZE 20 
#define my_id 3              // should be unique for each end-device!!!


uint8_t broadcastAddress[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

RTC_DATA_ATTR int RECEIVE_COUNTER = 0;
RTC_DATA_ATTR int MISS_COUNTER = 0;
RTC_DATA_ATTR int packetReceived = 0;
RTC_DATA_ATTR bool SYNCED = false;
RTC_DATA_ATTR unsigned long totalMillis = 0;


wifi_country_t country = {
  .cc = "JP",
  .schan = 1,
  .nchan = 14,
  .max_tx_power = 20,
  .policy = WIFI_COUNTRY_POLICY_AUTO,
};

esp_now_peer_info_t broadcastInfo;

typedef struct struct_message {
  unsigned int sender_id;  
  unsigned int packet_type;    
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
  String madeGuardAdjustment;
  int new_guard_time = 30;
  int sarsa_time; 
} PacketLog;

typedef struct knowledge_struct {
  uint8_t sender_id;      
  uint8_t knowledge_type;        
  float rss;
  //knowledge itself:
  unsigned long sleep_delay;      
  unsigned long guard_time;       
  Encoded_data encoded_table;     
} knowledge_struct;

typedef struct request_struct {
  uint8_t requester_id;
  uint8_t knowledge_type;
  float rss;
} request_struct;


typedef struct response_struct {
  uint8_t recipient_id;
  uint8_t packet_type;
  uint8_t knowledge_type;       
  unsigned long sleep_delay;    
  unsigned long guard_time;     
  Encoded_data encoded_table;   
} response_struct;

typedef struct acknowledgement_struct {
  uint8_t recipient_id;
  uint8_t packet_type; 
} acknowledgement_struct;
/////////////////////////////////////   RSSI  //////////////////////////////////////

RTC_DATA_ATTR int rssi_display;
RTC_DATA_ATTR int totalRssi = 0;
// Estructuras para calcular los paquetes, el RSSI, etc
typedef struct {
  unsigned frame_ctrl : 16;
  unsigned duration_id : 16;
  uint8_t addr1[6]; /* receiver address */
  uint8_t addr2[6]; /* sender address */
  uint8_t addr3[6]; /* filtering address */
  unsigned sequence_ctrl : 16;
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
  unsigned long hours = totalHours % 24; 

  char timestamp[20];
  sprintf(timestamp, "%02lu:%02lu:%02lu.%03lu", hours, minutes, seconds, ms);

  return String(timestamp);
}


PacketLog packetLog;
struct_message myData;
RTC_DATA_ATTR unsigned long long l_delay = 0;
RTC_DATA_ATTR unsigned long lastPacketTime = 0;
// Create an instance of the SARSA class
int numStates = 5;  // Set the number of states
int numActions = 11;  // Set the number of actions
int actions[] = { -10, -5, -3, -2, -1, 2, 1, 3, 5, 10, 0};
float learningRate = 0.1;     // Set the learning rate (alpha)
RTC_DATA_ATTR float explorationRate = 1.0;  // Set the exploration rate (epsilon)
float min_epsilon = 0.05;     // min possible exploration rate
float decay_rate = 0.001;     // decay rate of the epsilon
RTC_DATA_ATTR bool update = false;          // to update the Q-table
RTC_DATA_ATTR bool timer = false;
RTC_DATA_ATTR bool ON = false;
RTC_DATA_ATTR bool inOnData = false;
RTC_DATA_ATTR bool miss_p = false;
RTC_DATA_ATTR bool upd = false;
unsigned long start_time = 0;
RTC_DATA_ATTR unsigned long setup_time = 0;
unsigned long end_time = 0;

unsigned long sarsa_start_time = 0;
unsigned long sarsa_end_time = 0;
RTC_DATA_ATTR unsigned long calib_start = 0;
RTC_DATA_ATTR bool calib = true;
RTC_DATA_ATTR bool recalib = false;
RTC_DATA_ATTR int prev_pkt = 0;
RTC_DATA_ATTR int prev_pkt_recab = 0;
RTC_DATA_ATTR int lst_missed_packet = 0;
RTC_DATA_ATTR int diff_count = 0;
RTC_DATA_ATTR float success_rate = 0;
RTC_DATA_ATTR int lst_pack_time = 0;
RTC_DATA_ATTR int cons_pack = 0;
RTC_DATA_ATTR int hist_of_miss = 0;
RTC_DATA_ATTR bool recalib_lp = true;
RTC_DATA_ATTR int delay_ms = 0;
RTC_DATA_ATTR unsigned long last_delta_rcv = 0;
RTC_DATA_ATTR unsigned long wifiOnTime = 0;
RTC_DATA_ATTR bool first_packet_received = false;

RTC_DATA_ATTR int received_packet_counter = 0;
RTC_DATA_ATTR int guard_decrement_init = 5;
RTC_DATA_ATTR int guard_decrement_current = guard_decrement_init; 
RTC_DATA_ATTR int guard_decrement_max = 10;
const int guard_decrement_scale = 1;
RTC_DATA_ATTR bool upd_delta = false;
const int init_guard_time = 30; 
const int max_guard_time = 60;  
const int min_guard_time = 18;    
RTC_DATA_ATTR int current_guard_time = init_guard_time;   

RTC_DATA_ATTR float averageRssi;
//to keep track of avg rss of last 20 packets:
RTC_DATA_ATTR float rssiBuffer[20] = {0};
RTC_DATA_ATTR int currentIndex = 0;   
RTC_DATA_ATTR float sumRssi = 0;
RTC_DATA_ATTR float avg_rss_of_last_n_pkts = 0;

SARSA sarsa(numStates, numActions, learningRate, explorationRate);
RTC_DATA_ATTR float serializedQTable[55];

//for ToK:
const bool send_sleep_delay = true;  //set to false if don't wish to transfer
const bool send_guard_time = true;   //set to false if don't wish to transfer
const bool send_q_table = true;      //set to false if don't wish to transfer
request_struct req_pack;
response_struct resp_pack;
knowledge_struct knowledge_pack;
acknowledgement_struct ack_pack;
RTC_DATA_ATTR bool knowledge_request_phase = true;
RTC_DATA_ATTR int conseq_recieved = 0;
RTC_DATA_ATTR bool converged = false;
RTC_DATA_ATTR bool knowledge_delivery_success = false;
RTC_DATA_ATTR bool round_chosen = false;
RTC_DATA_ATTR unsigned long received_master_beacon_at;
RTC_DATA_ATTR bool send_knowledge = false;
RTC_DATA_ATTR bool send_request = false;
RTC_DATA_ATTR int round_to_send_knowledge;
RTC_DATA_ATTR int round_since_convergence = 0;
const uint8_t sleep_delay_bit = 0b0001; 
const uint8_t guard_time_bit = 0b0010; 
const uint8_t q_table_bit = 0b0100; 
RTC_DATA_ATTR uint8_t knowledge_type = 0;
RTC_DATA_ATTR unsigned long chosen_time_knowledge;
RTC_DATA_ATTR unsigned long chosen_time_request;


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
  if (packetLog.madeAdjustment != "") {
    Serial.print(packetLog.madeAdjustment);
    Serial.print("  | ");
    Serial.print(packetLog.newDelay);
    Serial.print("  | ");
  } else {
    Serial.print("N/A | ");  // If no adjustment, print N/A and leave the new delay column blank
    Serial.print(packetLog.newDelay);
    Serial.print("  | ");
  }
  if (packetLog.madeGuardAdjustment != "") {
    Serial.print(packetLog.madeGuardAdjustment);
    Serial.print(" | ");
    Serial.print(packetLog.new_guard_time);
    Serial.print("  | ");
  } else {
    Serial.print("N/A | ");  // If no adjustment, print N/A and leave the new delay column blank
    Serial.print(packetLog.new_guard_time);
    Serial.print("  | ");
  }
  Serial.print(packetLog.sarsa_time);
  Serial.print(" |");
  Serial.println();
}

void MissedPacket() {
  hist_of_miss++;
  MISS_COUNTER++;
  received_packet_counter = 0;
  prev_pkt_recab = 0;                              
  lst_missed_packet = packetReceived + cons_pack; 
  sarsa_start_time = micros();
  conseq_recieved = 0; 
  int currentState = 4;  //missed the packet and we need to change the guard time
  int nextState = 2;
  int currentAction = sarsa.selectAction(currentState);
  float reward = 0.0;  // Set the reward
  if (actions[currentAction] < 0) {
    currentAction += 2;
  }
  int nextAction = sarsa.selectAction(nextState);  // Set the next action
  if (hist_of_miss >= 2 && ((current_guard_time + actions[currentAction]) < max_guard_time)) {
    hist_of_miss = 0;
    current_guard_time += actions[currentAction];
    if(current_guard_time >= 18) {
      guard_decrement_max = 10;
      if(guard_decrement_current > guard_decrement_max) { // for cases when current was 15 while max is 10 
        guard_decrement_current = guard_decrement_max;
      }
    }
    upd_delta = true;
    sarsa.updateQValue(currentState, currentAction, reward, nextState, nextAction);
    packetLog.madeGuardAdjustment = String("+") + String(actions[currentAction]);

  } else {
    upd = false;
    upd_delta = false;
  }
  miss_p = true;
  success_rate = float(RECEIVE_COUNTER) / float(RECEIVE_COUNTER + MISS_COUNTER);
  currentState = 2;
  currentAction = sarsa.selectAction(currentState);  

  l_delay += actions[currentAction];
  reward = 0.0;  // Set the reward
  long delta2 = millis() - start_time;
  if (delta2 - actions[currentAction] > 100) {
    reward = 1.0;
  }
  nextState = 1;                               // Set the next state
  nextAction = sarsa.selectAction(nextState);  // Set the next action
  sarsa.updateQValue(currentState, currentAction, reward, nextState, nextAction);
  explorationRate = std::max(min_epsilon, (1 - min_epsilon) * std::exp(-decay_rate * (RECEIVE_COUNTER + MISS_COUNTER)));
  sarsa.setExpRate(explorationRate);
  sarsa_end_time = micros();
  packetLog.madeAdjustment = String(actions[currentAction]);
  packetLog.newDelay = l_delay;
  packetLog.timestamp = formatTimestamp(totalMillis + millis());
  packetLog.packetNumber = cons_pack + packetReceived;
  packetLog.received = false;
  packetLog.successRate = success_rate * 100;
  packetLog.radioOnTime = delta2;
  packetLog.new_guard_time = current_guard_time;
  if (!upd_delta) {
    packetLog.madeGuardAdjustment = "";
  }
  packetLog.sarsa_time = sarsa_end_time - sarsa_start_time;
  printPacketLog();
  lst_pack_time = millis();
}

void Calibration() {
  l_delay = (millis() - calib_start - setup_time) / (myData.packetNumber - prev_pkt);
  unsigned long long const_guard_time = ceil(l_delay * ESP_PPM);
  l_delay += const_guard_time;
  delay_ms = l_delay;
  calib = false;
  timer = true;
  start_time = 0;
  recalib = false;
  recalib_lp = true;
}

void Recalibration() {
  lst_missed_packet = 0; 
  calib_start = 0; 
  calib = true; 
  lst_pack_time = 0; 
  start_time = 0; 
  ON = false; 
  miss_p = false; 
  l_delay = 0; 
  packetLog.radioOnTime = 0;
  cons_pack = 0; 
  recalib = true;
  prev_pkt_recab = 0;
}


void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  received_master_beacon_at = millis();
  inOnData = true;
  if (len < sizeof(uint8_t) * 2) return; // ensure the packet is large enough
  
  uint8_t packet_type = *((uint8_t*)(incomingData + sizeof(uint8_t))); // extracting packet type
  if (packet_type == 1) {  // response packet with knowledge received
    memcpy(&resp_pack, incomingData, sizeof(resp_pack));
    if (resp_pack.recipient_id == my_id) { //if the response is for me
      if (resp_pack.knowledge_type == 0) {
        Serial.println("check 3");
        Serial.println("Master responded with empty pack. I will start learning by myself.");
        knowledge_request_phase = false;
      } else {
        explorationRate = 0.5; //can be changed, if needed
        if (resp_pack.knowledge_type & sleep_delay_bit) {
          Serial.print("Sleep delay received: ");
          Serial.println(resp_pack.sleep_delay);
          l_delay = resp_pack.sleep_delay;
        }
        if (resp_pack.knowledge_type & guard_time_bit) {
          Serial.print("Guard time received: ");
          Serial.println(resp_pack.guard_time);
          current_guard_time = resp_pack.guard_time;
        }
        if (resp_pack.knowledge_type & q_table_bit) {
          Serial.println("Encoded table received: ");
          
          /*unpacking the q table*/
          int index = 0;
          std::vector<std::vector<float>> decodedTable(numStates, std::vector<float>(numActions));
          sarsa.decodeEncodedData(resp_pack.encoded_table, decodedTable);
          sarsa.copyTable(decodedTable);
          sarsa.printQTable();
        }
        knowledge_request_phase = false;
      }
    } 
  } else if (packet_type == 2) { //knowledge acknowledgement received
    memcpy(&ack_pack, incomingData, sizeof(ack_pack));
    if (ack_pack.recipient_id == my_id) {
      knowledge_delivery_success = true;
      Serial.println("Knowledge Acknowledgement received.");
    }
  } else if (packet_type == 0) { //master beacon received
    conseq_recieved++;
    if (converged && !knowledge_delivery_success) {
      round_since_convergence++;
      Serial.print("round_since_convergence = ");
      Serial.println(round_since_convergence);
    }

    lst_pack_time = 0;
    cons_pack = 0;
    packetLog.rssi_display = rssi_display;
    totalRssi += rssi_display;
    
    ++received_packet_counter;
    // Turn off Wi-Fi module 
    esp_wifi_disconnect();
    if (esp_wifi_stop() != ESP_OK) {
      Serial.println("Troubles with stop function");
    }
    end_time = millis();  
    int currentState = 1;
    RECEIVE_COUNTER++;

    sumRssi -= rssiBuffer[currentIndex];  // remove the oldest value from sum
    rssiBuffer[currentIndex] = rssi_display; // insert the new RSS value
    sumRssi += rssi_display;                 // add the new RSS value to sum
    currentIndex = (currentIndex + 1) % RSSI_BUFFER_SIZE;
    int valid_samples = (RECEIVE_COUNTER < RSSI_BUFFER_SIZE) ? RECEIVE_COUNTER : RSSI_BUFFER_SIZE;
    avg_rss_of_last_n_pkts = sumRssi / valid_samples;


    if ((!converged && conseq_recieved >= 15) || (converged && !knowledge_delivery_success && !round_chosen)) {
      converged = true;
      Serial.println("I've converged. Knowledge not sent yet.");
      srand(millis()); //seed to randomize rand()
      round_to_send_knowledge = rand() % 11;  //random round between 0 and 10
      Serial.print("I've chosen a random round to send knowledge. Round = ");
      Serial.println(round_to_send_knowledge);
      round_chosen = true;
      round_since_convergence = 0;
    }
    
    memcpy(&myData, incomingData, sizeof(myData));
    
    if (!first_packet_received) { 
      first_packet_received = true;
    }

    packetReceived = myData.packetNumber;
    if (packetLog.packetNumber > packetReceived) {
      RECEIVE_COUNTER = 1;
      MISS_COUNTER = 0;
    }

    int pkt_dif = myData.packetNumber - prev_pkt_recab;  
    upd = false;
    upd_delta = false;

    if ((prev_pkt_recab != 0 && pkt_dif > 1) || (myData.packetNumber - lst_missed_packet > 1 && lst_missed_packet != 0) && recalib_lp) {
      Serial.println("Recalibrating... in ondata");
      Recalibration();
    }

    lst_missed_packet = 0;

    success_rate = float(RECEIVE_COUNTER) / float(RECEIVE_COUNTER + MISS_COUNTER);
    if (calib) {
      currentState = 0;
      if (calib_start > 0) {
        Calibration();
      }
      prev_pkt = myData.packetNumber;
      calib_start = millis();
    }
    if (timer) {
      ON = false;
    }
    unsigned long delta = end_time - start_time;
    wifiOnTime += delta;
    prev_pkt_recab = myData.packetNumber;  // 38
    if (start_time != 0 && !calib) {
      packetLog.radioOnTime = end_time - start_time;
      if (end_time - start_time < TIMEOUT_THRESHOLD) {
        last_delta_rcv = end_time - start_time;
      }
    }
    
    float timer_sarsa_start = micros();
    sarsa_start_time = micros();
    float reward = 0.0;
    long half_guard_time = current_guard_time / 2;

    if (long(delta) > current_guard_time && !calib && start_time != 0 && !miss_p) {  
      int currentAction = sarsa.selectAction(currentState); 
      if (long(delta - actions[currentAction]) > half_guard_time) {
        l_delay += actions[currentAction];
        reward = 1.0;
        int nextState = 1;                               
        int nextAction = sarsa.selectAction(nextState);  
        sarsa.updateQValue(currentState, currentAction, reward, nextState, nextAction);
        packetLog.madeAdjustment = String(actions[currentAction]);
        packetLog.newDelay = l_delay;
        upd = true;
      }
    } else if (long(delta) <= current_guard_time && long(delta) > half_guard_time && !calib && start_time != 0 && !miss_p) {
      currentState = 3;
      int currentAction = sarsa.selectAction(currentState);

      if (actions[currentAction] < 0) {
        currentAction += 2;
      }
      if (long(delta - actions[currentAction]) > half_guard_time - 2) {  //FOR POSITIVE VALUES
        l_delay += actions[currentAction];
        reward = 1.0;
      }
      int nextState = 3;                               
      int nextAction = sarsa.selectAction(nextState);  
      sarsa.updateQValue(currentState, currentAction, reward, nextState, nextAction);
      packetLog.madeAdjustment = String(actions[currentAction]);
      packetLog.newDelay = l_delay;
      upd = true;

    } else if (long(delta) < half_guard_time - 1 && !calib && start_time != 0 && !miss_p) {
      currentState = 3;
      int currentAction = sarsa.selectAction(currentState);
      // FOR NEGATIVE VALUES
      if (actions[currentAction] > 0) {
        currentAction -= 2;
      }
      if (long(delta) + actions[currentAction] < half_guard_time + 2) {
        l_delay += actions[currentAction];
        reward = 1.0;
      }
      int nextState = 3;                               
      int nextAction = sarsa.selectAction(nextState);  
      sarsa.updateQValue(currentState, currentAction, reward, nextState, nextAction);
      packetLog.madeAdjustment = String(actions[currentAction]);
      packetLog.newDelay = l_delay;
      upd = true;
    }

    if (currentState == 3 && received_packet_counter >= guard_decrement_current && delta > 0) {
      currentState = 4;
      int currentAction = sarsa.selectAction(currentState);
      if (actions[currentAction] > 0) {
        currentAction -= 2;
      }
      if (current_guard_time + actions[currentAction] >= min_guard_time) {
        reward = 1.0;
        int nextState = 3;
        int nextAction = sarsa.selectAction(nextState);
        sarsa.updateQValue(currentState, currentAction, reward, nextState, nextAction);
        current_guard_time += actions[currentAction];
        packetLog.madeGuardAdjustment = String(actions[currentAction]);
        packetLog.new_guard_time = current_guard_time;

        if(current_guard_time < 18) {
          guard_decrement_max = 15;
        }

        received_packet_counter = 0;
        if(guard_decrement_current+guard_decrement_scale <= guard_decrement_max) {
          guard_decrement_current += guard_decrement_scale;
        }
        
        upd_delta = true;
      }
    }

    explorationRate = std::max(min_epsilon, (1 - min_epsilon) * std::exp(-decay_rate * (RECEIVE_COUNTER + MISS_COUNTER)));
    sarsa.setExpRate(explorationRate);
    sarsa_end_time = micros();
    miss_p = false;

    packetLog.timestamp = formatTimestamp(totalMillis + millis());
    packetLog.packetNumber = myData.packetNumber;
    packetLog.received = true;
    packetLog.data = myData.time;
    packetLog.length = len;
    packetLog.successRate = success_rate * 100;
    if (!upd) {
      packetLog.madeAdjustment = "";
    }
    if (!upd_delta) {
      packetLog.madeGuardAdjustment = "";
    }
    packetLog.sarsa_time = sarsa_end_time - sarsa_start_time;
    printPacketLog();
    

    //to send a request pack to the master:
    if (knowledge_request_phase && !calib) {
      Serial.println("Ready to send request...");
      send_request = true;
      srand(millis()); //seed to randomize rand()
      chosen_time_request = rand() % (l_delay - 500 + 1); //random time t: 0 ms <= t <=  (l_delay - 500 ms)
      chosen_time_request = (chosen_time_request / 5) * 5; //rounding down to nearest multiple of 5 to enforce 5 ms time slots
      return;
    }

    //to send knowledge to the master:
    if (converged && round_since_convergence == round_to_send_knowledge && !knowledge_delivery_success) {
      Serial.println("Ready to send knowledge...");
      send_knowledge = true;
      srand(millis()); //seed to randomize rand()
      chosen_time_knowledge = rand() % (l_delay - 500 + 1); //random time t: 0 <= t <=  (l_delay - 500)
      chosen_time_knowledge = (chosen_time_knowledge / 5) * 5; //rounding down to nearest multiple of 5 to enforce 5 ms time slots
      return;
    }

    if (!send_request && !send_knowledge) {
      WiFi.mode(WIFI_STA);
      esp_wifi_start();
      esp_wifi_set_channel(14, WIFI_SECOND_CHAN_NONE);
      esp_wifi_set_country(&country);
      esp_wifi_set_promiscuous(true);
      esp_wifi_set_promiscuous_rx_cb(&promiscuous_rx_cb);
      esp_log_level_set("wifi", ESP_LOG_NONE);
      
      if (!calib) {
        sarsa.serializeQTable(serializedQTable);
        esp_sleep_enable_timer_wakeup(l_delay * 1000ULL);
        esp_deep_sleep_start();
      }
    } 
  //else {
  //   Serial.println("Pkt not for me. Ignored.");
  }   
}

void printFinalStatistics() {
  float finalSuccessRate = 0;
  if (RECEIVE_COUNTER + MISS_COUNTER != 0) {
    finalSuccessRate = (float)RECEIVE_COUNTER / (RECEIVE_COUNTER + MISS_COUNTER) * 100;
  }
  Serial.println("\n------ Final Statistics ------");
  Serial.print("Total Packets Received: ");
  Serial.println(RECEIVE_COUNTER);
  Serial.print("Total Packets Missed: ");
  Serial.println(MISS_COUNTER);
  Serial.print("Final Success Rate: ");
  Serial.print(finalSuccessRate);
  Serial.println("%");
  if (RECEIVE_COUNTER > 0) {  
    float averageRssi = float(totalRssi) / RECEIVE_COUNTER;
    Serial.print("Average RSS: ");
    Serial.println(averageRssi);
  }
  Serial.println("--------------------------------");
}


void packKnowledge() {
  knowledge_pack.sender_id = my_id;
  knowledge_pack.rss = avg_rss_of_last_n_pkts;

  if (send_sleep_delay) {
    knowledge_type |= sleep_delay_bit;
    knowledge_pack.sleep_delay = l_delay;
    Serial.print("Packing the sleep delay. Sleep delay = " );
    Serial.println(l_delay);
  }
  
  if (send_guard_time) {
    knowledge_type |= guard_time_bit;
    knowledge_pack.guard_time = current_guard_time;
    Serial.print("Packing the guard time. Guard time = " );
    Serial.println(current_guard_time);
  } 

  if (send_q_table) {
    knowledge_type |= q_table_bit;
    Serial.println("Packing the Q-Table: " );
    knowledge_pack.encoded_table = sarsa.getEncodedTableStruct();
  }
  
  knowledge_pack.knowledge_type = knowledge_type;
}

void packRequest() {
  req_pack.requester_id = my_id;
  req_pack.rss = averageRssi;
  knowledge_type = 0;

  if (send_sleep_delay) {
    knowledge_type |= sleep_delay_bit;
  }
  if (send_guard_time) {
    knowledge_type |= guard_time_bit;
  } 
  if (send_q_table) {
    knowledge_type |= q_table_bit;
  }
  req_pack.knowledge_type = knowledge_type;
}

void setup() {
  Serial.begin(115200);
  int start_setup_time = millis();

  WiFi.useStaticBuffers(true);
  WiFi.mode(WIFI_STA);
  esp_wifi_set_protocol(WIFI_IF_AP, WIFI_PROTOCOL_LR);
  esp_wifi_config_espnow_rate(WIFI_IF_AP, WIFI_PHY_RATE_LORA_250K);

  if (esp_now_init() != ESP_OK) {
    Serial.println("There was an error initializing ESP-NOW");
    return;
  }
  
  esp_wifi_set_channel(14, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_country(&country);

  memcpy(broadcastInfo.peer_addr, broadcastAddress, 6);
  broadcastInfo.channel = 14;
  if (esp_now_add_peer(&broadcastInfo) != ESP_OK) {
    Serial.println("There was an error registering the broadcast address");
    return;
  }

  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_promiscuous_rx_cb(&promiscuous_rx_cb);
  esp_log_level_set("wifi", ESP_LOG_NONE);


  if (!first_packet_received) {
    packetReceived = 0;
    int currentState = 0;
    Serial.println("  Timestamp  |Pct#| RCV | PRR | delta |Data|Len|RSSI|sleep Adj|New sleep|guard Adj|New guard|sarsa time");
    Serial.println("-------------------------------------------------------------------------------------------------------");
    setup_time = millis() - start_setup_time;
  } else {
    sarsa.deserializeQTable(serializedQTable);
  } 

  start_time = millis();
  if (!calib && timer) {
    ON = true;
  }
}


void loop() {

  //to request knowledge from master upon joining the system:
  if (send_request && (millis() - received_master_beacon_at) >= chosen_time_request) {
    packRequest();
    WiFi.mode(WIFI_STA);
    esp_wifi_start();
    esp_wifi_set_channel(14, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_country(&country);
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_promiscuous_rx_cb(&promiscuous_rx_cb);
    esp_log_level_set("wifi", ESP_LOG_NONE);
    esp_err_t result1 = esp_now_send(broadcastInfo.peer_addr, (uint8_t *)&req_pack, sizeof(req_pack));
    if (result1 == ESP_OK) {
      Serial.println("Request sent successfully.");
    } else {
      // Print the error code if the send fails:
      Serial.print("Error sending request: ");
      Serial.println(esp_err_to_name(result1));
    }
    send_request = false;
  }

  //to send knowledge to master after converging: 
  if (send_knowledge && (millis() - received_master_beacon_at) >= chosen_time_knowledge) {
    //packing the knowledge:
    packKnowledge();

    //initializing wifi:
    WiFi.mode(WIFI_STA);
    esp_wifi_start();
    esp_wifi_set_channel(14, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_country(&country);
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_promiscuous_rx_cb(&promiscuous_rx_cb);
    esp_log_level_set("wifi", ESP_LOG_NONE);

    //sending the knowledge pack:
    esp_err_t result1 = esp_now_send(broadcastInfo.peer_addr, (uint8_t *)&knowledge_pack, sizeof(knowledge_pack));
    if (result1 == ESP_OK) {
      Serial.println("Knowledge sent successfully.");
    } else {
      // Print the error code if the send fails:
      Serial.print("Error sending knowledge: ");
      Serial.println(esp_err_to_name(result1));
    }
    send_knowledge = false;
    round_chosen = false;
  }

  if (RECEIVE_COUNTER + MISS_COUNTER >= TOTAL_PACKETS) {
    printFinalStatistics();
    Serial.println("Turning off...");
    delay(1000);
    Serial.flush();
    esp_deep_sleep_start(); 
  }
    
  long timeSinceLastPacket = millis() - start_time;
  if (cons_pack >= 3) { 
    Serial.println("Recalibrating... in loop");
    Recalibration();
  }
  if (timeSinceLastPacket > TIMEOUT_THRESHOLD && ON && !recalib) {
    cons_pack++;
    MissedPacket();
    ON = false;
    inOnData = false;
  } else if (start_time != 0 && lst_pack_time > 0 && (millis() - lst_pack_time > l_delay + last_delta_rcv) && !recalib && !inOnData) {
    cons_pack++;
    MissedPacket();
  }
}