#include <iostream>
#include <vector>
#include <random>

// QLearning class
class QLearning {
private:
    std::vector<std::vector<float>> qTable;  // Q-table
    float learningRate;                      // Learning rate (alpha)
    float discountFactor;                    // Discount factor (gamma)
    float explorationRate;                   // Exploration rate (epsilon)

public:
    QLearning(int numStates, int numActions, float alpha, float gamma, float epsilon)
        : learningRate(alpha), discountFactor(gamma), explorationRate(epsilon) {
        // Initialize the Q-table with zeros
        qTable.resize(numStates, std::vector<float>(numActions, 0.0));
    }

    int selectAction(int state) {
        // Epsilon-greedy action selection
        if (random(0, 100) < explorationRate * 100) {
            // Explore: Select a random action
            return random(0, qTable[state].size());
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

    void updateQValue(int state, int action, float reward, int nextState) {
        // Update the Q-value using the Q-learning formula
        float currentQ = qTable[state][action];
        float maxNextQ = *std::max_element(qTable[nextState].begin(), qTable[nextState].end());
        float newQ = currentQ + learningRate * (reward + discountFactor * maxNextQ - currentQ);
        qTable[state][action] = newQ;
    }
};
