// Copyright 2020 Newrizon. All rights reserved.
// Use of this source code  CCP be found in the LICENSE file,
// which is part of this source code package.

#include "inc/ccp_message_handler.h"
#include "inc/json_reader.h"
#include <glog/logging.h>
#include <boost/thread.hpp>
#include <boost/format.hpp>
#include <boost/algorithm/hex.hpp>
#include <iostream>
#include <boost/chrono.hpp>
#include <string>
#include "inc/socket_can_driver_class.h"
#include "inc/xcp_driver_config.h"
namespace newrizon {
namespace xcp {

CCPMessageHandler* CCPMessageHandler::instance_ = NULL;

CcpInfo CCPMessageHandler::ccp_info_;

CCPMessageHandler* CCPMessageHandler::GetInstance() {
  if (instance_ == NULL) {
    instance_ = new CCPMessageHandler();
  }
  return instance_;
}

CCPMessageHandler::CCPMessageHandler() {
  can_driver_ = can_driver::SocketCanDriver::GetInstance();
  counter_ = 0;
}

int CCPMessageHandler::Connect() {
  uint8_t out_buff[] = {CCP_CONNECT, counter_++};  // according to a2l file the station address should be 0x01!!!
  uint32_t out_dlc = 2;
  can_driver_->Write(ccp_info_.channel, ccp_info_.download_can_id, out_buff,
                     out_dlc);
  int64_t id;
  unsigned char msg[8];
  unsigned int dlc;
  unsigned int flag;
  uint64_t time;
  // can_driver_->ReadBlock(ccp_info_.channel, &id, msg, &dlc, &flag, &time);

  // LOG(INFO) << "connecting : " << std::endl;
  // LOG(INFO) << "channel : " << ccp_info_.channel << std::endl;
  do {
    can_driver_->ReadBlock(ccp_info_.channel, &id, msg, &dlc, &flag, &time);
    // msg[1] is return code
    if (id == ccp_info_.upload_can_id && msg[0] == 0xFF && msg[1] == 0x00) {
      // LOG(INFO) << "connecting : " << std::endl;
      break;
    }
    // LOG(INFO) << "id : " << id << std::endl;
    // for (int i = 0; i < dlc; i++) printf("%02X ", msg[i]);
  } while (true);
  // if (msg[0] == 0xFF) {
  // LOG(INFO) << "connected" << std::endl;
  // }
  // for (int i = 0; i < dlc; i++) printf("%02X \n", msg[i]);
  // printf("%02X \n", msg[1]);
  // printf("%d \n", msg[1]);

  CcpErrorCode ret = CCP_SUCESS;
  return ret;
}

CcpErrorCode CCPMessageHandler::Disconnect() {
  uint8_t out_buff[] = {CCP_DISCONNECT, counter_++, 0x01, 0x00, 0x00, 0x00};
  uint32_t out_dlc = 6;
  can_driver_->Write(ccp_info_.channel, ccp_info_.download_can_id, out_buff,
                     out_dlc);
  CcpErrorCode err = WaitResponse(5000);
  if (err == CCP_SUCESS) {
    LOG(INFO) << "disconnected" << std::endl;
    return CCP_SUCESS;
  } else {
    LOG(INFO) << "disconnected fail" << std::endl;
    return CCP_ERROR;
  }
}


int CCPMessageHandler::Download(const std::vector<uint8_t>& address,
                                const std::vector<uint8_t>& download_message) {
  SetMta(address);
  CcpErrorCode err = WaitResponse(5000);
  if (err == CCP_SUCESS) {
    LOG(INFO) << "SET_MTA Sucess" << std::endl;
  } else {
    LOG(INFO) << "SET_MTA Fail" << std::endl;
  }
  uint32_t total_length = download_message.size();
//  int block_size = newrizon::config::ccp_max_download_block_size;
  int loop_time = total_length / 6;
  uint8_t last_message_length = total_length % 6;
  LOG(INFO) << "total_length : " << total_length << std::endl;
  LOG(INFO) << "last message length : " << last_message_length;

  uint8_t out_buff[8];
  out_buff[0] = CCP_DNLOAD_6;

  for (int i = 0; i < loop_time; ++i) {
    out_buff[1] = counter_++;
    int start_index = i*6;
    // send first message
    for (int j = 0; j < 6; j++) {
      out_buff[j + 2] = download_message[start_index + j];
    }
    can_driver_->Write(ccp_info_.channel, ccp_info_.download_can_id, out_buff,
                       8);
    err = WaitResponse(5000);
  }

  // last message
  out_buff[0] = CCP_DNLOAD;
  out_buff[1] = counter_++;
  out_buff[2] = last_message_length;
  int start_index = loop_time * 6;
  for (int j = 0; j < last_message_length; j++) {
    out_buff[j + 3] = download_message[start_index + j];
  }
  can_driver_->Write(ccp_info_.channel, ccp_info_.download_can_id, out_buff,
                     3 + last_message_length);

  err = WaitResponse(5000);
  return err;
}

std::vector<uint8_t> CCPMessageHandler::Upload(
    const std::vector<uint8_t>& address, const uint32_t& message_length) {
  int i = 0;
  int j = 0;
  int64_t id;
  unsigned char msg[8];
  unsigned int dlc;
  unsigned int flag;
  uint64_t time;

  std::vector<uint8_t> ret;

  LOG(INFO) << "SET_MTA start" << std::endl;
  SetMta(address);

  CcpErrorCode err = WaitResponse(5000);
  if (err == CCP_SUCESS) {
    LOG(INFO) << "SET_MTA Sucess" << std::endl;
  } else {
    LOG(INFO) << "SET_MTA Fail" << std::endl;
    return ret;
  }

  LOG(INFO) << "message length : " << message_length;
  uint32_t loop_time = message_length / 5;
  uint8_t last_message_length = message_length % 5;

  LOG(INFO) << "loop_time : " << loop_time;
  LOG(INFO) << "last message length : " << unsigned(last_message_length);

  uint8_t out_buff[8];
  out_buff[0] = CCP_UPLOAD;
  out_buff[1] = 0;
  out_buff[2] = 0x05;
  int received_bytes = 0;
  // loop n times

  for (int n = 0; n < loop_time; ++n) {
    out_buff[1] = counter_++;
    // LOG(INFO) << "loop : " << n << std::endl;
    can_driver_->Write(ccp_info_.channel, ccp_info_.download_can_id, out_buff,
                       3);
    can_driver_->ReadBlock(ccp_info_.channel, &id, msg, &dlc, &flag, &time);
    if (id == ccp_info_.upload_can_id && msg[0] == 0xFF && msg[1] == 0x00) {
      for (i = 3; i < dlc; ++i) {
        // printf("%02X ", msg[i]);
        ret.push_back(msg[i]);
        received_bytes++;
      }
      // printf("\n");
    }
    // LOG(INFO) << "ret length after loop : " << i << " :" << ret.size()
    //           << std::endl;
  }

  // get last message
  out_buff[1] = counter_++;
  out_buff[2] = last_message_length;
  can_driver_->Write(ccp_info_.channel, ccp_info_.download_can_id, out_buff, 3);
  // printf("last message \n");
    // while (ret.size() < message_length) {
  can_driver_->ReadBlock(ccp_info_.channel, &id, msg, &dlc, &flag, &time);
  if (id == ccp_info_.upload_can_id && msg[0] == 0xFF) {
    for (i = 3; i < 3 + last_message_length; ++i) {
      // printf("%02X ", msg[i]);
      ret.push_back(msg[i]);
      received_bytes++;
    }
    // printf("\n");
  }
  LOG(INFO) << "ret length : " << ret.size() << std::endl;
  return ret;
}

void CCPMessageHandler::SetCcpInfo(const CcpInfo& ccp_info) {
  // start CCP receiving threads
  ccp_info_ = ccp_info;
  status_ = CCP_DISCONNECTED;
}

void CCPMessageHandler::DownloadCcpData(const std::vector<CcpData>& vec_data) {
  Connect();
  std::vector<CcpData>::const_iterator it;
  for (it = vec_data.begin(); it != vec_data.end(); ++it) {
    Download(it->address, it->data);
  }
  Disconnect();
}

void CCPMessageHandler::UploadCcpData(std::vector<CcpData>* vec_data) {
  Connect();
  std::vector<CcpData>::iterator it;
  LOG(INFO) << "uploadsize: "<<(*vec_data)[0].data.size();
  for (it = vec_data->begin(); it != vec_data->end(); ++it) {
    uint32_t value_bytes = 1;
    uint32_t data_length = 1;
    if (it->value_type == "FLOAT32_IEEE") {
      value_bytes = 4; }
    for (int i = 0; i < it->dim.size(); i++) {
      data_length = data_length * it->dim[i];
    }
    uint32_t total_bytes = data_length * value_bytes;
    LOG(INFO) << "data_length: " << data_length;
    LOG(INFO) << "value_bytes: " << value_bytes;
    LOG(INFO) << "total bytes: " << total_bytes;

    auto uploaded_data = Upload(it->address, total_bytes);
    if (IsSame(it->data, uploaded_data)) {
      LOG(INFO) << "uploaded data is same with the json file" << std::endl;
    } else {
      it->data = uploaded_data;
    }
  }
  Disconnect();
  // PrintCcpData(vec_data);
}

std::vector<XcpData> CCPMessageHandler::CompareData(std::string path, std::vector<XcpData>& vec_data){
  /*
  sweep over an array of XCP data and compare with previous data, dissect the difference and return a new array of XCP data.
  each element of the array will be a segment of the difference.
  */
  // previous data
  newrizon::xcp::JsonReader json_reader;
  json_reader.LoadJsonFromPath(path);
  std::vector<XcpData>* prev_data = json_reader.GetData();
  // initialize vectors to store float values
  std::vector<float> NewValue;
  std::vector<float> PrevValue;
  // initialize vectors to store diff
  std::vector<int> diffIndex;
  std::vector<std::string> diffAddress;
  std::vector<uint8_t> diffValue;
  
  // stream for original address
  int adddec;
  std::stringstream ogAdss;
  unsigned int len = vec_data.size();
  // iterate for every XCP data
  for (unsigned int j = 0; j < len; j++) {
    XcpData temp_prev = (*prev_data)[j];
    XcpData temp_in = (vec_data)[j];
    // read address
    for (size_t i = 0; i < 4; ++i) {
      ogAdss << boost::format("%02x") % unsigned(temp_in.address[i]);
    }
    // convert address to int to make calculation
    std::string addstring = ogAdss.str();
    adddec = stoi(addstring,0,16);
    // iterate over every value and convert to float
    for (size_t k = 0; k < temp_in.data.size(); k=k+4) {
        // convert to uint32
        uint32_t combByte = temp_in.data[k] | (temp_in.data[k+1] << 8) | (temp_in.data[k+2] << 16) | (temp_in.data[k+3] << 24); 
        float floatValue;
        // convert to float
        memcpy(&floatValue, &combByte, 4);
        NewValue.push_back(floatValue);
      } 
    // prev data convert to float
    for (size_t k = 0; k < temp_prev.data.size(); k=k+4) {
        // convert to uint32
        uint32_t combByte = temp_prev.data[k] | (temp_prev.data[k+1] << 8) | (temp_prev.data[k+2] << 16) | (temp_prev.data[k+3] << 24); 
        float floatValue;
        // convert to float
        memcpy(&floatValue, &combByte, 4);
        PrevValue.push_back(floatValue);
      } 
  }
  // comparing the float values in the array and push back to diff vectors
  for(unsigned int i = 0;i!=NewValue.size();i++)
  {
    if(abs(NewValue[i]-PrevValue[i])>0.1)
    {
      diffIndex.push_back(i);
      diffValue.push_back(vec_data[0].data[4*i]);
      diffValue.push_back(vec_data[0].data[4*i+1]);
      diffValue.push_back(vec_data[0].data[4*i+2]);
      diffValue.push_back(vec_data[0].data[4*i+3]);
      int newaddress = adddec + 4*i;
      std::stringstream newss;
      newss << std::hex << newaddress;
      std::string updateAddress = newss.str(); 
      diffAddress.push_back(updateAddress);  //array of strings
    }
  }
  // initialize vector to store different XCP
  std::vector<XcpData> diff_data; 
  // return message if no difference
  if(diffIndex.size()==0)
  {
    LOG(INFO) << "No difference between tables";
    return diff_data;
  }
  // remove continuous numbers
  auto it = diffIndex.begin()+1;
  auto is = diffAddress.begin()+1;
  auto iv = diffValue.begin()+4;
  int prev = diffIndex[0];  
  // push initial diff
  if (diff_data.size()==0)
  {
    XcpData diff_test;
    diff_test.data.push_back(diffValue[0]);
    diff_test.data.push_back(diffValue[1]);
    diff_test.data.push_back(diffValue[2]);
    diff_test.data.push_back(diffValue[3]);
    diff_data.push_back(diff_test);
  }
  // iterate over vectors and push values
  while (it !=diffIndex.end()){ //update the current diff segment and transfer data from diffValue to diff_data
    if (*it - prev == 1) {
      diff_data.back().data.push_back(*iv);
      diff_data.back().data.push_back(*(iv+1));
      diff_data.back().data.push_back(*(iv+2));  
      diff_data.back().data.push_back(*(iv+3));            
      diffIndex.erase(it);
      diffAddress.erase(is);        
      diffValue.erase(iv+3);
      diffValue.erase(iv+2);
      diffValue.erase(iv+1);
      diffValue.erase(iv);
      prev = prev + 1;
    } else {  //start a new diff segment
      XcpData diff_test;
      diff_test.data.push_back(*iv);
      diff_test.data.push_back(*(iv+1));
      diff_test.data.push_back(*(iv+2));
      diff_test.data.push_back(*(iv+3));
      diff_data.push_back(diff_test); 
      prev = *it;
      it = it+1;  
      is = is+1; 
      iv = iv+4;            
    }   
  }
  // push address
  for (int i = 0; i!=diffAddress.size();i++)
  {
    boost::algorithm::unhex(diffAddress[i], std::back_inserter(diff_data[i].address));    
  }     

  return diff_data;
} 


CcpErrorCode CCPMessageHandler::WaitResponse(const int& timeout_ms) {
  int i;
  int64_t id;
  unsigned char msg[8];
  unsigned int dlc;
  unsigned int flag;
  uint64_t time;
  // std::string hex_str = "ff";

  // auto vec_hex = StringToVectorUInt8(hex_str);

  auto tick = boost::posix_time::second_clock::local_time();
  int delta_ms = 0;
  while (delta_ms < timeout_ms) {
    can_driver_->ReadBlock(ccp_info_.channel, &id, msg, &dlc, &flag, &time);
    auto now = boost::posix_time::second_clock::local_time();
    boost::posix_time::time_duration diff = tick - now;
    delta_ms = diff.total_milliseconds();
    // if match hex string
    // for (i = 0; i < vec_hex.size(); ++i) {
    //   if (msg[i] != vec_hex[i]) {
    //     continue;
    //   }
    // }
    if(msg[0] != 0xff){
      continue;
    }

    // TODO should check the second byte of CRM-DTO command return code to be 0x00:qw
    if (msg[1] == 0x00) {
      return CCP_SUCESS;
    }else{
      return CCP_ERROR;
    }
  }
    // if (dlc != vec_hex.size()) {
    //   continue;
    // }
    // for (i = 0; i < dlc; ++i) {
    //   if (msg[i] != vec_hex[i]) {
    //     continue;
    //   }
    // }
    // return CCP_SUCESS
  // }
  return CCP_ERROR;
}

void CCPMessageHandler::SetMta(const std::vector<uint8_t>& address) {
  uint8_t out_buff[8] = {CCP_SET_MTA, counter_++, 0x00,       0x00,
                         address[3],  address[2], address[1], address[0]};
  can_driver_->Write(ccp_info_.channel, ccp_info_.download_can_id, out_buff, 8);
}

}  // namespace xcp
}  // namespace newrizon
