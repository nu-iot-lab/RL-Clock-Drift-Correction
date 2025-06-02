#ifndef HUFFMAN_H
#define HUFFMAN_H

#include <vector>
#include <map>
#include <string>
#include <queue>
#include "bitstream.h"
#include <climits>

struct Node {
    int data;
    unsigned freq;
    Node* left;
    Node* right;

    Node(int data, unsigned freq, Node* l = nullptr, Node* r = nullptr);
};

struct Compare {
    bool operator()(Node* l, Node* r);
};

class HuffmanCoding {
public:
    HuffmanCoding();
    ~HuffmanCoding();

    Node* encode(const std::vector<std::vector<float>>& qTable, std::map<int, std::string>& huffmanCode, std::string& encodedString);
    bool decode(Node* root, BitReader& br, std::vector<std::vector<float>>& decodedTable, int totalValues);
    void printCodes(Node* root, std::string str, std::map<int, std::string>& huffmanCode);
    Node* deserialize(BitReader& br, int dataBits);
    void clearTree(Node* node);
    static int getMaxDataValue(Node* node);

private:
    Node* root;
};

#endif // HUFFMAN_H