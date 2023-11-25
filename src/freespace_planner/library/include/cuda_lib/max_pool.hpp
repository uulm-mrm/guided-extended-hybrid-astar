//
// Created by schumann on 03.04.23.
//

#ifndef MAX_POOL_CUDA_MAX_POOL_CUH
#define MAX_POOL_CUDA_MAX_POOL_CUH

#include <ctime>
#include <cstdint>
#include <chrono>
#include <cassert>
#include <cstdio>
#include <iostream>
#include <cudnn.h>
#include <vector>

#define CUDNN_DTYPE CUDNN_DATA_INT8
// function to print out error message from cuDNN calls
#define checkCUDNN(exp)                                                                                                \
  {                                                                                                                    \
    cudnnStatus_t status = (exp);                                                                                      \
    if (status != CUDNN_STATUS_SUCCESS)                                                                                \
    {                                                                                                                  \
      std::cerr << "Error on line " << __LINE__ << ": " << cudnnGetErrorString(status) << std::endl;                   \
      std::exit(EXIT_FAILURE);                                                                                         \
    }                                                                                                                  \
  }

class Pooling
{
public:
  static void init(int pooling_dim);

  static void execute(const uint8_t* image_pointer, uint8_t* result, int in_dim, int out_dim);

  inline static cudnnHandle_t cudnn_;
  inline static cudnnTensorDescriptor_t in_desc_;
  inline static cudnnTensorDescriptor_t out_desc_;
  inline static cudnnPoolingDescriptor_t pooling_desc_;

  inline static float alpha_ = 1.0F;
  inline static float beta_ = 0.0F;
};

#endif  // MAX_POOL_CUDA_MAX_POOL_CUH
