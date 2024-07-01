#ifndef _GZIP_H_
#define _GZIP_H_

#include <string>

extern bool gz_compress_file(std::string in_file, std::string out_file);
extern bool gz_decompress_file(std::string in_file, std::string out_file);

#endif // _GZIP_H_
