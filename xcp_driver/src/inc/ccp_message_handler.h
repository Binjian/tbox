// Copyright 2020 Newrizon. All rights reserved.
// Use of this source code  can be found in the LICENSE file,
// which is part of this source code package.

#ifndef SRC_INC_CCP_MESSAGE_HANDLER_H_
#define SRC_INC_CCP_MESSAGE_HANDLER_H_

#include <vector>
#include <string>
#include <boost/thread.hpp>
#include "inc/xcp_data.h"
#include "inc/socket_can_driver_class.h"

namespace newrizon {
namespace xcp {

enum CcpErrorCode { CCP_SUCESS = 0, CCP_ERROR = 1, CCP_TIMEOUT = 2 };
enum CcpStauts { CCP_DISCONNECTED = 0, CCP_CONNECTED = 1, CCP_DOWNLOADING = 2 };

enum CcpCommandCode {
  CCP_CONNECT = 0x01,
  CCP_SET_MTA = 0x02,
  CCP_DNLOAD = 0x03,
  CCP_UPLOAD = 0x04,
  CCP_DNLOAD_6 = 0x23,
  CCP_SHORT_UP = 0X0F,
  CCP_DISCONNECT = 0x07
};

struct CcpCommand{
  uint8_t msg[8];
  uint8_t length;
};

class CCPMessageHandler {
 public:
  static CCPMessageHandler* GetInstance();
  void SetCcpInfo(const CcpInfo& ccp_info);
  void DownloadCcpData(const std::vector<CcpData>& vec_data);
  void UploadCcpData(std::vector<CcpData>* vec_data);
  std::vector<XcpData> CompareData(std::string path,std::vector<XcpData>& vec_data);

 private:
  CCPMessageHandler();
  static CCPMessageHandler* instance_;
  static CcpInfo ccp_info_;
  can_driver::SocketCanDriver* can_driver_;
  CcpStauts status_;
  uint8_t counter_;
  int Connect();
  // int Download(uint8_t* address, uint8_t* msg, uint8_t message_length);
  int Download(const std::vector<uint8_t>& address,
               const std::vector<uint8_t>& download_message);
  int DownloadDataBlock(const std::vector<uint8_t>& download_message);
  std::vector<uint8_t> Upload(const std::vector<uint8_t>& address,
                              const uint32_t& message_length);
  CcpErrorCode Disconnect();

  void SetMta(const std::vector<uint8_t>& address);
  // return
  //   0:sucess; -1:timeout
  CcpErrorCode WaitResponse(const int& timeout_ms);

  // void ReceiveCanMessage(int channel);
  boost::thread* can_receive_thread_;
};

}  // namespace xcp
}  // namespace newrizon

#endif  // SRC_INC_CCP_MESSAGE_HANDLER_H_
