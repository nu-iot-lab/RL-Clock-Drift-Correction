#include "huffman.h"
#include <iostream>
#include <bitset>
#include <iomanip>
#include <cmath>

Node::Node(int data, unsigned freq, Node* l, Node* r)
    : data(data), freq(freq), left(l), right(r) {}

bool Compare::operator()(Node* l, Node* r) {
    return l->freq > r->freq;
}

HuffmanCoding::HuffmanCoding() : root(nullptr) {}

HuffmanCoding::~HuffmanCoding() {
    clearTree(root);
}

void HuffmanCoding::clearTree(Node* node) {
    if (node) {
        clearTree(node->left);
        clearTree(node->right);
        delete node;
    }
}

Node* HuffmanCoding::encode(const std::vector<std::vector<float>>& qTable, std::map<int, std::string>& huffmanCode, std::string& encodedString) {
    std::map<int, int> freq;
    for (auto &row : qTable) {
        for (float val : row) {
            int quantized = static_cast<int>(round((val + 1) * 1000));
            freq[quantized]++;
        }
    }

    std::priority_queue<Node*, std::vector<Node*>, Compare> pq;
    for (auto& pair : freq) {
        pq.push(new Node(pair.first, pair.second));
    }

    while (pq.size() > 1) {
        Node *left = pq.top(); pq.pop();
        Node *right = pq.top(); pq.pop();
        pq.push(new Node(-1, left->freq + right->freq, left, right));
    }

    root = pq.top();
    printCodes(root, "", huffmanCode);

    for (auto &row : qTable) {
        for (float val : row) {
            int quantized = static_cast<int>(round((val + 1) * 1000));
            encodedString += huffmanCode[quantized];
        }
    }
    return root;
}

void HuffmanCoding::printCodes(Node* root, std::string str, std::map<int, std::string>& huffmanCode) {
    if (!root) return;
    if (root->data != -1) 
        huffmanCode[root->data] = str;
    printCodes(root->left, str + "0", huffmanCode);
    printCodes(root->right, str + "1", huffmanCode);
}

Node* HuffmanCoding::deserialize(BitReader& br, int dataBits) {
    bool bit;
    if (!br.readBit(bit)) {
        return nullptr;
    }

    if (bit) {
        uint32_t dataValue;
        if (!br.readBits(dataValue, dataBits)) {
            return nullptr;
        }
        return new Node(dataValue, 0);
    } else {
        Node* left = deserialize(br, dataBits);
        Node* right = deserialize(br, dataBits);
        if (!left || !right) {
            return nullptr;
        }
        return new Node(-1, 0, left, right);
    }
}

bool HuffmanCoding::decode(Node* root, BitReader& br, std::vector<std::vector<float>>& decodedTable, int totalValues) {
    if (!root) {
        return false;
    }

    if (!root->left && !root->right) {
        float value = (root->data / 1000.0f) - 1.0f;
        for (auto& row : decodedTable) {
            std::fill(row.begin(), row.end(), value);
        }
        return true;
    }

    int currentRow = 0;
    int currentCol = 0;
    Node* currentNode = root;
    while (totalValues > 0) {
        bool bit;
        if (!br.readBit(bit)) {
            return false;
        }
        currentNode = bit ? currentNode->right : currentNode->left;
        if (!currentNode) {
            return false;
        }
        if (!currentNode->left && !currentNode->right) {
            float value = (currentNode->data / 1000.0f) - 1.0f;
            decodedTable[currentRow][currentCol++] = value;
            if (currentCol == decodedTable[currentRow].size()) {
                currentCol = 0;
                currentRow++;
            }
            currentNode = root;
            --totalValues;
        }
    }
    return true;
}

int HuffmanCoding::getMaxDataValue(Node* node) {
    if (!node) return INT_MIN;
    if (!node->left && !node->right) {
        return node->data;
    }
    return std::max(getMaxDataValue(node->left), getMaxDataValue(node->right));
}