#include "gzip.hpp"
#include <zlib.h>

bool gz_compress_file(std::string in_file, std::string out_file) {
    const int BUFF_SIZE = 32768;
    char buffer[BUFF_SIZE];
    auto file = fopen(in_file.c_str(), "rb");
    auto gz_file = gzopen(out_file.c_str(), "wb");
    while (!feof(file)) {
        auto len = fread(buffer, 1, BUFF_SIZE, file);
        gzwrite(gz_file, buffer, len);
    }
    gzclose(gz_file);
    return true;
}

bool gz_decompress_file(std::string in_file, std::string out_file) {
    const int BUFF_SIZE = 32768;
    char buffer[BUFF_SIZE];
    auto gz_file = gzopen(in_file.c_str(), "rb");
    auto file = fopen(out_file.c_str(), "wb");
    while (!gzeof(gz_file)) {
        auto len = gzread(gz_file, buffer, BUFF_SIZE);
        fwrite(buffer, 1, len, file);
    }
    gzclose(gz_file);
    return true;
}
