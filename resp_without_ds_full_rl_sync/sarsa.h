// BY NU IOT LAB //
// Authors: Damir Assylbek, Aizhuldyz Nadirkhanova, Dimitrios Zormpas //
// GPL-3.0 license //

#include <iostream>
#include <vector>
#include <random>

// SARSA class
class SARSA {
private:
  std::vector<std::vector<float>> qTable;  // Q-table
  float learningRate;                      // Learning rate (alpha)
  float explorationRate;                   // Exploration rate (epsilon)

public:
  SARSA(int numStates, int numActions, float alpha, float epsilon)
    : learningRate(alpha), explorationRate(epsilon) {
    // Initialize the Q-table with specific values for out-of-range actions
    qTable.resize(numStates, std::vector<float>(numActions, 0.0));

    for (int state = 0; state < numStates; ++state) {
        for (int action = 0; action < numActions; ++action) {
            if (state == 0 && action != qTable[state].size() - 1) {
                qTable[state][action] = -1.0;  // Set a suitable negative value
            } else if (state == 1 && (action < 5 || action == qTable[state].size() - 1)) {
                qTable[state][action] = -1.0;  // Set a suitable negative value
            } else if (state == 2 && (action < 0 || action > 5)) {
                qTable[state][action] = -1.0;  // Set a suitable negative value
            } else if (state == 3 && (action < 3 || action > qTable[state].size() - 4)) {
                qTable[state][action] = -1.0;  // Set a suitable negative value
            } else if (state == 4 && (action < 3 || action > qTable[state].size() - 4)) {
                qTable[state][action] = -1.0;  // Set a suitable negative value
            }
        }
    }
}



  int selectAction(int state) {
    // Epsilon-greedy action selection
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
      // Exploit: Select the action with the highest Q-value
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

};
