#include <iostream>
#include <vector>
#include <random>
#include <map>
#include <iomanip> 

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
        // Simulate out-of-range conditions as needed
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
      // Explore: Select a random action
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

  void setExpRate(float newExpRate) {
    explorationRate = newExpRate;
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