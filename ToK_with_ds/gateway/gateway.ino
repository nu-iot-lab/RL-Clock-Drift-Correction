// BY NU IOT LAB //
// GPL-3.0 license //

#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

#define TIME_PERIOD 5
#define S_TO_MS 1000
#define MAX_TX_COUNT 500


//for qtable:
#define MAX_TREE_SIZE 52
#define MAX_STRING_SIZE 22

uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
unsigned int packetNumber = 1;
wifi_country_t country = {
  .cc = "JP",
  .schan = 1,
  .nchan = 14,
  .max_tx_power = 20,
  .policy = WIFI_COUNTRY_POLICY_AUTO,
};
esp_now_peer_info_t slaveInfo;

// knowledge type bit flags
const uint8_t sleep_delay_bit = 0b0001; 
const uint8_t guard_time_bit = 0b0010; 
const uint8_t q_table_bit = 0b0100;

// broadcast pack struct
typedef struct struct_message {
  unsigned int sender_id = 0;  
  unsigned int packet_type = 0;    
  unsigned long time;         
  unsigned int packetNumber;
} struct_message;

struct Encoded_data {
  uint8_t encoded_tree[MAX_TREE_SIZE];
  uint8_t encoded_string[MAX_STRING_SIZE];
  short encoded_tree_length;
  short encoded_string_length;
  short dataBits;
};

// struct to store received knowledge
typedef struct knowledge_struct {
  uint8_t sender_id;      
  uint8_t knowledge_type;        
  float rss;
  unsigned long sleep_delay;      
  unsigned long guard_time;       
  Encoded_data encoded_table;     
} knowledge_struct;

// request pack struct
typedef struct request_struct {
  uint8_t requester_id;
  uint8_t knowledge_type;
  float rss;
} request_struct;

// response pack struct
typedef struct response_struct {
  uint8_t recipient_id;
  uint8_t packet_type = 1;
  uint8_t knowledge_type;       
  unsigned long sleep_delay;    
  unsigned long guard_time;     
  Encoded_data encoded_table;   
} response_struct;

typedef struct acknowledgement_struct {
  uint8_t recipient_id;
  uint8_t packet_type = 2; 
} acknowledgement_struct;

struct_message myData;
knowledge_struct knowledge_pack;  // to send knowledge
knowledge_struct knowledge_excellent_rss; // to store knowledge for excellent RSS
knowledge_struct knowledge_medium_rss;    // to store knowledge for medium RSS
knowledge_struct knowledge_poor_rss;      // to store knowledge for poor RSS
request_struct req_pack;   // to unpack received req pack
acknowledgement_struct ack_pack; //to send knowledge acknowledgements

// unpacking and storing received knowledge:
void ProcessKnowledge(knowledge_struct& struct_with_knowledge) {
  if (knowledge_pack.knowledge_type & sleep_delay_bit) {
    Serial.print("Unpacking sleep delay: ");
    Serial.println(knowledge_pack.sleep_delay);
    struct_with_knowledge.sleep_delay = knowledge_pack.sleep_delay;
  }

  if (knowledge_pack.knowledge_type & guard_time_bit) {
    Serial.print("Unpacking guard time: ");
    Serial.println(knowledge_pack.guard_time);
    struct_with_knowledge.guard_time = knowledge_pack.guard_time;
  }

  if (knowledge_pack.knowledge_type & q_table_bit) {
    Serial.println("Unpacking Q-Table.");
    struct_with_knowledge.encoded_table = knowledge_pack.encoded_table;
  }
}

// preparing a knowledge response pack to send:
void CreateKnowledgeResponse(const knowledge_struct& category, uint8_t request_bits, response_struct& response) {
  response.knowledge_type = 0; // reset response knowledge type
  if ((request_bits & sleep_delay_bit) && category.sleep_delay != 0) {
    response.sleep_delay = category.sleep_delay;
    response.knowledge_type |= sleep_delay_bit;
  } else {
    Serial.println("Requested sleep delay is not available.");
  }

  if ((request_bits & guard_time_bit) && category.guard_time != 0) {
    response.guard_time = category.guard_time;
    response.knowledge_type |= guard_time_bit;
  } else {
    Serial.println("Requested guard time is not available.");
  }

  if ((request_bits & q_table_bit) && category.encoded_table.encoded_string_length > 0) {
    response.encoded_table = category.encoded_table;
    response.knowledge_type |= q_table_bit;
  } else {
    Serial.println("Requested encoded table is not available.");
  }
}

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  if (len == 96) { //handling knowledge pkts:
    Serial.println("Knowledge packet received.");
    memcpy(&knowledge_pack, incomingData, sizeof(knowledge_pack));
    
    //sending knowledge acknowledgement:
    ack_pack.recipient_id = knowledge_pack.sender_id;
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t*)&ack_pack, sizeof(ack_pack));
    if (result == ESP_OK) {
      Serial.println("Knowledge Acknowledgement sent successfully.");
    } else {
      Serial.printf("Error broadcasting Knowledge Acknowledgement. Code: %d\n", result);
    }

    Serial.printf("Sender ID: %u, RSS: %.2f\n", knowledge_pack.sender_id, knowledge_pack.rss);
    if (knowledge_pack.rss > -70) {
      Serial.println("Category: Excellent.");
      ProcessKnowledge(knowledge_excellent_rss);
    } else if (knowledge_pack.rss <= -70 && knowledge_pack.rss > -85) {
      Serial.println("Category: Medium.");
      ProcessKnowledge(knowledge_medium_rss);
    } else {
      Serial.println("Category: Poor.");
      ProcessKnowledge(knowledge_poor_rss);
    }
  } else if (len == 8) { //handling request pkts:
    Serial.println("Request packet received.");
    memcpy(&req_pack, incomingData, sizeof(req_pack));

    knowledge_struct* selected_category;
    if (req_pack.rss > -70) {
      selected_category = &knowledge_excellent_rss;
    } else if (req_pack.rss <= -70 && req_pack.rss > -85) {
      selected_category = &knowledge_medium_rss;
    } else {
      selected_category = &knowledge_poor_rss;
    }

    response_struct response;
    CreateKnowledgeResponse(*selected_category, req_pack.knowledge_type, response);
    response.recipient_id = req_pack.requester_id;

    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t*)&response, sizeof(response));
    if (result == ESP_OK) {
      Serial.println("Broadcast response sent successfully.");
    } else {
      Serial.printf("Error broadcasting response. Code: %d\n", result);
    }
  } else {
    Serial.println("Unknown packet type received. Packet ignored.");
    Serial.println(len);
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  Serial.println("ESP-NOW Broadcast Initiator");

  WiFi.useStaticBuffers(true);
  WiFi.mode(WIFI_STA);

  esp_wifi_set_protocol(WIFI_IF_AP, WIFI_PROTOCOL_LR);
  esp_wifi_config_espnow_rate(WIFI_IF_AP, WIFI_PHY_RATE_LORA_250K);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  Serial.println("ESP-NOW initialized.");
  esp_wifi_set_country(&country);
  esp_wifi_set_channel(14, WIFI_SECOND_CHAN_NONE);

  memcpy(slaveInfo.peer_addr, broadcastAddress, 6);
  if (esp_now_add_peer(&slaveInfo) != ESP_OK) {
    Serial.println("Failed to add peer.");
    return;
  }

  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
  Serial.println("Setup complete.");
}

void loop() {
  for (int tx = 0; tx < MAX_TX_COUNT; ++tx) {
    myData.time = 24;
    myData.packetNumber = packetNumber++;

    Serial.printf("Sending packet %d\n", myData.packetNumber);
    esp_err_t result = esp_now_send(slaveInfo.peer_addr, (uint8_t*)&myData, sizeof(myData));
    if (result != ESP_OK) {
      Serial.println("Error sending packet.");
    }
    
    delay(TIME_PERIOD * S_TO_MS);
  }

  Serial.println("Sent out 500 beacons. Entering deep sleep.");
  delay(1000);
  Serial.flush();
  esp_deep_sleep_start();
}
