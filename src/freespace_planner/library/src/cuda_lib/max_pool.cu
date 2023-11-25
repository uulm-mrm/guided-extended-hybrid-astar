#include "cuda_lib/max_pool.hpp"

void Pooling::init(int pooling_dim) {

  checkCUDNN(cudnnCreate(&cudnn_));

  //create descriptor handle
  checkCUDNN(cudnnCreatePoolingDescriptor(&pooling_desc_));
  //initialize descriptor
  checkCUDNN(cudnnSetPooling2dDescriptor(pooling_desc_,            //descriptor handle
                                         CUDNN_POOLING_MAX,       //mode - max pooling
                                         CUDNN_NOT_PROPAGATE_NAN, //NaN propagation mode
                                         pooling_dim,                       //window height
                                         pooling_dim,                       //window width
                                         0,                       //vertical padding
                                         0,                       //horizontal padding
                                         pooling_dim,                       //vertical stride
                                         pooling_dim));                     //horizontal stride
}

void Pooling::execute(const uint8_t* image_pointer, uint8_t *result, int in_dim, int out_dim) {

//  auto t0 = std::chrono::high_resolution_clock::now();

  const size_t in_data_bytes = in_dim * in_dim * sizeof(uint8_t);
  const size_t out_data_bytes = out_dim * out_dim * sizeof(uint8_t);

  // GPU data pointers
  uint8_t *in_data;
  uint8_t *out_data;
  // allocate arrays on GPU
  cudaMalloc(&in_data, in_data_bytes);
  cudaMalloc(&out_data, out_data_bytes);


  //create input data tensor descriptor
  checkCUDNN(cudnnCreateTensorDescriptor(&in_desc_));
  //initialize input data descriptor
  checkCUDNN(cudnnSetTensor4dDescriptor(in_desc_,                  //descriptor handle
                                        CUDNN_TENSOR_NCHW,        //data format
                                        CUDNN_DTYPE,              //data type (precision)
                                        1,                        //number of images
                                        1,                        //number of channels
                                        in_dim,                       //data height
                                        in_dim));                     //data width

  //create output data tensor descriptor
  checkCUDNN(cudnnCreateTensorDescriptor(&out_desc_));
  //initialize output data descriptor
  checkCUDNN(cudnnSetTensor4dDescriptor(out_desc_,                 //descriptor handle
                                        CUDNN_TENSOR_NCHW,        //data format
                                        CUDNN_DTYPE,              //data type (precision)
                                        1,                        //number of images
                                        1,                        //number of channels
                                        out_dim,                        //data height
                                        out_dim));                      //data width

  //copy input data to GPU array
  cudaMemcpy(in_data, image_pointer, in_data_bytes, cudaMemcpyHostToDevice);

  //Call pooling operator
  checkCUDNN(cudnnPoolingForward(cudnn_,         //cuDNN context handle
                                 pooling_desc_,  //pooling descriptor handle
                                 &alpha_,        //alpha scaling factor
                                 in_desc_,       //input tensor descriptor
                                 in_data,       //input data pointer to GPU memory
                                 &beta_,         //beta scaling factor
                                 out_desc_,      //output tensor descriptor
                                 out_data));    //output data pointer from GPU memory

  //copy output data from GPU
  cudaMemcpy(result, out_data, out_data_bytes, cudaMemcpyDeviceToHost);

  cudaFree(in_data);
  cudaFree(out_data);

//  auto t1 = std::chrono::high_resolution_clock::now();
//  auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0);
}



