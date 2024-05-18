// Copyright 2020 Newrizon. All rights reserved.
// Use of this source code  can be found in the LICENSE file,
// which is part of this source code package.

#include <glog/logging.h>
#include <cstdlib>

#include <iostream>
#include <iterator>

#include <boost/program_options.hpp>
#include <boost/thread.hpp>

#include "inc/ccp_message_handler.h"
#include "inc/json_reader.h"
#include "inc/socket_can_driver_class.h"
#include "inc/xcp_data.h"
#include "inc/xcp_driver_config.h"
#include "inc/xcp_message_handler.h"

using std::string;
// using LOG(INFO);
using newrizon::xcp::CCPMessageHandler;
using newrizon::xcp::XcpData;
using newrizon::xcp::XCPMessageHandler;
using std::cerr;
using std::cout;
using std::endl;
using std::exception;
namespace po = boost::program_options;

int g_log_level = -1;

po::variables_map ParseArgs(int argc, char **argv)
{
  google::InitGoogleLogging(argv[0]);
  po::variables_map vm;
  try
  {
    po::options_description desc("Allowed options");
    desc.add_options()("help", "produce help message")(
        "mode", po::value<string>(), "set mode download or upload")(
        "protocol", po::value<string>(), "set protocol xcp/ccp")(
        "input", po::value<string>(), "set input path")(
        "output", po::value<string>(), "set output path")(
        "diff", po::value<string>(), "set diff algorithm");

    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help"))
    {
      LOG(INFO) << desc << "\n";
      exit(0);
    }

    if (vm.count("protocol"))
    {
      string protocol = vm["protocol"].as<string>();
      if (protocol != "xcp" && protocol != "ccp")
      {
        LOG(INFO) << "protocol was not set.\n";
        exit(0);
      }
      LOG(INFO) << "protocol was set to " << protocol << ".\n";
    }

    if (vm.count("mode"))
    {
      string mode = vm["mode"].as<string>();
      LOG(INFO) << "mode was set to " << mode << ".\n";
      if (mode == "upload")
      {
        if (vm.count("output"))
        {
          LOG(INFO) << "output json : " << vm["output"].as<string>() << ".\n";
        }
        else
        {
          LOG(INFO) << "output json path was not set.\n";
          exit(0);
        }
      }
    }
    else
    {
      LOG(INFO) << "mode was not set.\n";
      exit(0);
    }

    if (vm.count("input"))
    {
      LOG(INFO) << "input json : " << vm["input"].as<string>() << ".\n";
    }
    else
    {
      LOG(INFO) << "input json path was not set.\n";
      exit(0);
    }
    return vm;
  }
  catch (exception &e)
  {
    cerr << "error: " << e.what() << "\n";
    exit(0);
  }
  catch (...)
  {
    cerr << "Exception of unknown type!\n";
    exit(0);
  }
}
// TODO multi-thread
int main(int argc, char **argv)
{
  if (const char *env_p = std::getenv("GLOG_logtostderr"))
  {
    g_log_level = std::stoi(env_p);
    LOG(INFO) << "Glog level is: " << g_log_level;
  }

  po::variables_map vm = ParseArgs(argc, argv);

  std::string protocol = vm["protocol"].as<string>();
  std::string mode = vm["mode"].as<string>();
  std::string input_json_path = vm["input"].as<string>();
  std::string output_json_path = vm["output"].as<string>();
  std::string diff_mode_flag = vm["diff"].as<string>();

  newrizon::xcp::JsonReader json_reader;
  json_reader.LoadJsonFromPath(input_json_path);
  PrintXcpData(json_reader.GetData());
  newrizon::xcp::XcpInfo info = json_reader.GetXcpInfo();
  std::vector<uint32_t> bypass_ids;
  bypass_ids.push_back(info.upload_can_id);

  newrizon::can_driver::SocketCanDriver::GetInstance()->Start();
  newrizon::can_driver::SocketCanDriver::GetInstance()->SetFilter(info.channel,
                                                                  bypass_ids);

  if (protocol == "ccp")
  {
    LOG(INFO) << "using ccp protocol" << std::endl;
    CCPMessageHandler *ccp_handler = CCPMessageHandler::GetInstance();
    ccp_handler->SetCcpInfo(json_reader.GetXcpInfo());
    if (mode == "download")
    {
      LOG(INFO) << "download" << std::endl;
      if (diff_mode_flag == "on")
      {
        std::vector<XcpData> diff = ccp_handler->CompareData(output_json_path, *json_reader.GetData());
        std::vector<XcpData> *diff_pointer = &diff;
        ccp_handler->DownloadCcpData(*diff_pointer);
      }
      else if (diff_mode_flag == "off")
      {
        ccp_handler->DownloadCcpData(*json_reader.GetData());
      }
      // save previous download.json
      json_reader.SaveJson(output_json_path);
    }
    else if (mode == "upload")
    {
      LOG(INFO) << "upload" << std::endl;
      ccp_handler->UploadCcpData(json_reader.GetData());
      LOG(INFO) << "save to json file : " << output_json_path << std::endl;
      json_reader.SaveJson(output_json_path);
    }
    else
    {
    }
  }
  else if (protocol == "xcp")
  {
    LOG(INFO) << "using xcp protocol" << std::endl;
    XCPMessageHandler *xcp_handler = XCPMessageHandler::GetInstance();
    xcp_handler->SetXcpInfo(json_reader.GetXcpInfo());
    if (mode == "download")
    {
      LOG(INFO) << "download" << std::endl;
      LOG(INFO) << "compared finished!"
                << "\n";
      xcp_handler->DownloadXcpData(*json_reader.GetData());
    }
    else if (mode == "upload")
    {
      LOG(INFO) << "upload" << std::endl;
      xcp_handler->UploadXcpData(json_reader.GetData());
      LOG(INFO) << "save to json file : " << output_json_path << std::endl;
      json_reader.SaveJson("../json/upload.json");
    }
    else
    {
    }
  }
  else
  {
  }
  // while (1) {
  //   boost::this_thread::sleep_for(boost::chrono::milliseconds(
  //       newrizon::config::can_driver_main_thread_interval));
  // }

  return 0;
}
