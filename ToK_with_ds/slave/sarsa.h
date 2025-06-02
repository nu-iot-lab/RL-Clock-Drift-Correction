#include <iostream>
#include <vector>
#include <random>
#include <map>
#include <iomanip>
#include <cmath>
#include <climits>
#include "huffman.h"
#include "bitstream.h"

#define MAX_TREE_SIZE 52
#define MAX_STRING_SIZE 22

struct Encoded_data {
  uint8_t encoded_tree[MAX_TREE_SIZE];     //52 bytes
  uint8_t encoded_string[MAX_STRING_SIZE]; //22 bytes 
  short encoded_tree_length;               //2 bytes
  short encoded_string_length;             //2 bytes
  short dataBits;                          //2 bytes   
};

class SARSA {
private:
  std::vector<std::vector<float>> qTable;
  float learningRate;
  float explorationRate;
  std::default_random_engine generator;

public:
  SARSA(int numStates, int numActions, float alpha, float epsilon)
        : learningRate(alpha), explorationRate(epsilon) {
    qTable.resize(numStates, std::vector<float>(numActions, 0.0f));

    for (int state = 0; state < numStates; ++state) {
      for (int action = 0; action < numActions; ++action) {
        if (state == 0 && action != numActions - 1) {
          qTable[state][action] = -1.0f;
        } else if (state == 1 && (action < 5 || action == numActions - 1)) {
          qTable[state][action] = -1.0f;
        } else if (state == 2 && (action < 0 || action > 5)) {
          qTable[state][action] = -1.0f;
        } else if (state == 3 && (action < 3 || action > numActions - 4)) {
          qTable[state][action] = -1.0f;
        } else if (state == 4 && (action < 3 || action > numActions - 4)) {
          qTable[state][action] = -1.0f;
        }
      }
    }
  }

  int selectAction(int state) {
    if (state == 0){
      return qTable[state].size()-1;
    }
    if (random(0, 100) < explorationRate * 100) {
      if(state == 1) {
          return random(5, qTable[state].size()-1);
      }
      else if(state == 2) {
          return random(0, 5);
      }
      else if (state == 3) {
          return random(3, qTable[state].size()-4);
      }
      else if (state == 4) {
          return random(3, qTable[state].size()-4);
      }
    } else {
      float maxQ = qTable[state][0];
      int maxIndex = 0;
      for (int i = 1; i < qTable[state].size(); ++i) {
        if (qTable[state][i] > maxQ) {
          maxQ = qTable[state][i];
          maxIndex = i;
        }
      }
      return maxIndex;
    }
  }

  void updateQValue(int state, int action, float reward, int nextState, int nextAction) {
  // 𝑄(𝑆𝑡,𝐴𝑡)=𝑄(𝑆𝑡,𝐴𝑡)+𝛼[𝑅𝑡+1+𝛾𝑄(𝑆𝑡+1,𝐴𝑡+1)−𝑄(𝑆𝑡,𝐴𝑡)]
    float currentQ = qTable[state][action];
    float nextQ = qTable[nextState][nextAction];
    float newQ = currentQ + learningRate * (reward + nextQ - currentQ);
    qTable[state][action] = newQ;
  }

  void setExpRate(float newExpRate) {
    explorationRate = newExpRate;
  }

  void printQTable() {
    std::cout << std::fixed << std::setprecision(3);
    for (const auto& row : qTable) {
        for (const auto& value : row) {
            std::cout << value << " ";
        }
        std::cout << "\n";
    }
    std::cout << std::endl;
  }

  Encoded_data getEncodedTableStruct() {
    std::map<int, std::string> huffmanCode;
    HuffmanCoding huffman;
    std::string encodedString;
    Node* root = huffman.encode(qTable, huffmanCode, encodedString);

    int maxDataValue = HuffmanCoding::getMaxDataValue(root);
    int dataBits = static_cast<int>(std::ceil(std::log2(maxDataValue + 1)));

    Encoded_data encodedData;
    encodedData.dataBits = dataBits;

    BitWriter treeWriter(encodedData.encoded_tree, MAX_TREE_SIZE);
    if (!serializeBitWriter(root, treeWriter, dataBits) || !treeWriter.flush()) {
      std::cerr << "Error: Huffman tree serialization failed." << std::endl;
      encodedData.encoded_tree_length = 0;
    } else {
      encodedData.encoded_tree_length = static_cast<int>(treeWriter.getSize());
    }

    BitWriter dataWriter(encodedData.encoded_string, MAX_STRING_SIZE);
    bool dataWriteSuccess = true;
    for (char bitChar : encodedString) {
      bool bit = (bitChar == '1');
      if (!dataWriter.writeBit(bit)) {
        std::cerr << "Error: Encoded string serialization failed." << std::endl;
        dataWriteSuccess = false;
        break;
      }
    }
    if (!dataWriter.flush()) {
      std::cerr << "Error: Encoded string flush failed." << std::endl;
      dataWriteSuccess = false;
    }

    if (dataWriteSuccess) {
      encodedData.encoded_string_length = static_cast<int>(dataWriter.getSize());
    } else {
      encodedData.encoded_string_length = 0;
    }

    printQTable();
    return encodedData;
  }


  bool serializeBitWriter(Node* node, BitWriter& bw, int dataBits) {
    if (!node) return true;

    if (!node->left && !node->right) {
      if (!bw.writeBit(1)) return false;
      if (!bw.writeBits(node->data, dataBits)) return false;
    } else {
      if (!bw.writeBit(0)) return false;
      if (!serializeBitWriter(node->left, bw, dataBits)) return false;
      if (!serializeBitWriter(node->right, bw, dataBits)) return false;
    }
    return true;
  }

  bool decodeEncodedData(const Encoded_data& encodedData, std::vector<std::vector<float>>& decodedTable) {
    HuffmanCoding huffman;

    BitReader treeReader(encodedData.encoded_tree, encodedData.encoded_tree_length);
    Node* root = huffman.deserialize(treeReader, encodedData.dataBits);
    if (!root) {
      std::cerr << "Error: Failed to deserialize Huffman tree." << std::endl;
      return false;
    }

    BitReader dataReader(encodedData.encoded_string, encodedData.encoded_string_length);
    int totalValues = qTable.size() * qTable[0].size();
    decodedTable.resize(qTable.size(), std::vector<float>(qTable[0].size(), 0.0f));
    if (!huffman.decode(root, dataReader, decodedTable, totalValues)) {
      std::cerr << "Error: Failed to decode encoded data." << std::endl;
      huffman.clearTree(root);
      return false;
    }
    huffman.clearTree(root);
    return true;
  }

  void copyTable(std::vector<std::vector<float>> decodedTable) {
    for (int state = 0; state < 5; ++state) {
      for (int action = 0; action < 11; ++action) {
        qTable[state][action] = decodedTable[state][action];
      }
    }
  }

  void serializeQTable(float serialized[55]) {
    int index = 0;
    for (int i = 0; i < 5; ++i) {
        for (int j = 0; j < 11; ++j) {
            if (index < 55) { // Ensure we do not exceed the array size
              serialized[index] = qTable[i][j];
              index++;
            }
        }
    }
  }

  void deserializeQTable(float serialized[55]) {
    int index = 0;
    for (int i = 0; i < 5; ++i) {
      for (int j = 0; j < 11; ++j) {
        if (index < 55) { 
          qTable[i][j] = serialized[index];
          index++;
        }
      }
    }
  }
};