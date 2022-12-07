#pragma once

#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/string.hpp>

#include <cellular_driver/easywsclient.hpp>

#include <rapidjson/document.h>
#include <rapidjson/prettywriter.h>

using std::placeholders::_1;

#define PREPASS_URL "ws://127.0.0.1:8081/PrePass/PrePassDecision"
#define SAFE_SPECT_URL "ws://127.0.0.1:8082/SafeSpect/SafeSpect"

using namespace easywsclient;

namespace cellular_driver
{

  class CellularDriver : public rclcpp::Node
  {
    private:
      WebSocket::pointer prePassConnection;
      WebSocket::pointer safeSpectConnection;

      rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

    protected:

      void processJson(rapidjson::Document&);
      void handle_request_cb(const std_msgs::msg::String::SharedPtr);

      void wsHandler(const std::string &);
      void processInboundWSMessages();
      void connect(WebSocket::pointer *, char *);

    public:

      CellularDriver();
      ~CellularDriver();

      void initialize();
 
      void run();
  };
}
