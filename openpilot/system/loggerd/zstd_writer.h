#pragma once

#include <zstd.h>

#include <string>
#include <vector>
#include <capnp/common.h>

class ZstdFileWriter {
public:
  ZstdFileWriter(const std::string &filename, int compression_level);
  ~ZstdFileWriter();
  void write(void* data, size_t size);
  // Finish the Zstandard frame and close the file. This is deliberately
  // explicit so callers can keep an incomplete-log marker when finalization
  // fails instead of silently treating a truncated stream as complete.
  bool close();
  inline void write(kj::ArrayPtr<capnp::byte> array) { write(array.begin(), array.size()); }

private:
  bool flushCache(bool last_chunk);

  size_t input_cache_capacity_ = 0;
  std::vector<char> input_cache_;
  std::vector<char> output_buffer_;
  ZSTD_CStream *cstream_;
  FILE* file_ = nullptr;
  bool closed_ = false;
  bool failed_ = false;
};
