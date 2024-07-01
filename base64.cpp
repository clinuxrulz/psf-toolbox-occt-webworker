#include <iostream>
#include <string>
#include <vector>
#include <cstdint>

using namespace std;

// Base64 encoding table
const string base64_chars =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

// Function to encode a string to base64 (no padding)
string base64_encode(const string& str) {
  vector<uint8_t> bytes(str.begin(), str.end());
  string encoded;
  for (size_t i = 0; i < bytes.size(); i += 3) {
    // Pad the last block with zeros if necessary
    uint32_t block = (bytes[i] << 16) | (i + 1 < bytes.size() ? bytes[i + 1] << 8 : 0) |
                     (i + 2 < bytes.size() ? bytes[i + 2] : 0);

    // Encode the block into 4 base64 characters
    for (int j = 3; j >= 0; --j) {
      encoded += base64_chars[(block >> (6 * j)) & 0x3f];
    }
  }

  // No padding added
  return encoded;
}

// Function to decode a base64 encoded string (no padding)
string base64_decode(const string& encoded) {
  string decoded;
  vector<uint8_t> bytes;
  for (size_t i = 0; i < encoded.size(); i += 4) {
    // Decode the 4 base64 characters into a 3 byte block
    uint32_t block = 0;
    for (int j = 0; j < 4; ++j) {
      // Check for valid index before accessing encoded string
      if (i + j < encoded.size()) { 
        // Find the index of the character in the base64 table
        int index = base64_chars.find(encoded[i + j]);
        block |= (index << (6 * (3 - j)));
      }
    }

    // Extract the bytes from the block (only if they exist)
    bytes.push_back((block >> 16) & 0xff);
    if (i + 1 < encoded.size()) {
      bytes.push_back((block >> 8) & 0xff);
    }
    if (i + 2 < encoded.size()) {
      bytes.push_back(block & 0xff);
    }
  }

  // Construct the decoded string
  decoded.assign(bytes.begin(), bytes.end());
  return decoded;
}