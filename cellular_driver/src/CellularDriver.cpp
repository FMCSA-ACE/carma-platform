/***********************************************************************************/
/* Copyright (C) 2022 eScience and Technologies, Inc.                              */
/*                                                                                 */
/* Licensed under the Apache License, Version 2.0 (the "License"); you may not     */
/* use this file except in compliance with the License. You may obtain a copy of   */
/* the License at                                                                  */
/*                                                                                 */
/* http://www.apache.org/licenses/LICENSE-2.0                                      */
/*                                                                                 */
/* Unless required by applicable law or agreed to in writing, software             */
/* distributed under the License is distributed on an "AS IS" BASIS, WITHOUT       */
/* WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the        */
/* License for the specific language governing permissions and limitations under   */
/* the License.                                                                    */
/*                                                                                 */
/* Developer:  Wayne Staats                                                        */
/*                                                                                 */
/* Class:    CellularDriver                                                        */
/* Purpose:  Implements the management of the WebSocket connections to the         */
/*           external entities.  The class uses easywsclient David Baird           */
/*           rapidjson (rapidjson.org) for parsing/creating json documents.        */
/***********************************************************************************/  


#include "cellular_driver/CellularDriver.hpp"


namespace cellular_driver
{
  CellularDriver::CellularDriver():Node("cellular_driver")
  {
    myPrepassConnection = NULL;
    mySafeSpectConnection = NULL;
    this -> declare_parameter("connection_strings"); 
  }

  CellularDriver::~CellularDriver()
  {
    if (mySafeSpectConnection != NULL)
    {
       delete mySafeSpectConnection;
    }
    
    if (myPrepassConnection != NULL)
    {
      delete myPrepassConnection;
    }
  }
  
  /*********************************************************************************/  
  /* Method:    processWebsockets                                                  */
  /* Purpose:   This method loops through each websocket connection.  If the       */
  /*            socket experienced an error, the socket is reopened.  Once in      */
  /*            open state, check if any data is availabe to process.              */
  /*            The parameter to the dispatch method is a callback that actually   */
  /*            processes the data.                                                */
  /* Note:      After re-establishing a connection, the code still needs to        */
  /*            if the connection is open.  It is possible that a server is        */
  /*            offline such that a connection cannot be made.  In this case, a    */
  /*            static "dummy_socket" is created as a placeholder in the data      */
  /*            structures used to manage the websockets.  If that dummy were      */
  /*            not created, a re-connection would not occur.                      */
  /*********************************************************************************/  
  void CellularDriver::processWebsockets()
  {
    string url;
    WsHandler *handler;

    for (WebSocket::pointer ws : myConnections)
    {
      if (ws->getReadyState() != WebSocket::OPEN)
      {
        WebSocket::pointer dummy;

        dummy = WebSocket::create_dummy();
        url = mySocketMapping[ws];
        mySocketMapping.erase(ws);
        ws = WebSocket::from_url(url);
        if (ws == NULL)
        {
          if (ws != dummy)
          {
            delete ws;
            ws = dummy;
          }
        }
        mySocketMapping.insert(pair<WebSocket::pointer, string>(ws,url));
      }
      if (ws -> getReadyState() == WebSocket::OPEN)
      {
        handler = new WsHandler(this, ws);

        ws -> poll();
        ws -> dispatch(*handler);
  
        delete handler;  // make this cleaner
      }

    }
  }
    
  /*********************************************************************************/  
  /* Method:    initialize                                                         */
  /* Purpose:   This method initializes the plugin. 1)  reads the websocket URLs   */
  /*            from the parameters.yaml file and opens the websocket.  If the     */
  /*            fails, create a dummy socket; this is placeholder to map URL to    */ 
  /*            socket which is used to re-establish.                             */
  /*            2) create the publishers and subscribers.                          */
  /*********************************************************************************/  
  void CellularDriver::initialize()
  {
    vector<string> urls;
    WebSocket::pointer ws;

    rclcpp::Parameter connections_type("connection_strings", vector<string>({}));
    this -> get_parameter("connection_strings", connections_type);
    urls = connections_type.as_string_array();

    for(string url  : urls)
    {
      ws = WebSocket::from_url(url.c_str()); 
      if (ws == NULL)
      {
        ws = WebSocket::create_dummy();
      }
      myConnections.push_back(ws);
      mySocketMapping.insert(pair<WebSocket::pointer, string>(ws,url));
    }

    myPrepassPublisher = 
        this->create_publisher<std_msgs::msg::String>("prepass_decision",10);
    myHealthRequestPublisher = 
        this->create_publisher<std_msgs::msg::String>("ads_health_request",10);

    myHealthSubscription = 
      this->create_subscription<carma_v2x_msgs::msg::ADSSafety>(
      "ads_health", 10, std::bind(&CellularDriver::handle_ads_health, this, _1));

    myPositionSubscription = 
      this->create_subscription<carma_v2x_msgs::msg::FullPositionVector>(
      "ace_position", 10, std::bind(&CellularDriver::handle_position, this, _1));
  }
  
  /*********************************************************************************/  
  /* Method:    handle_position                                                    */
  /* Purpose:   This method convects the GPS position message to JSON then         */
  /*            transmits it.                                                      */
  /*********************************************************************************/  
  void CellularDriver::handle_position(
     const carma_v2x_msgs::msg::FullPositionVector::SharedPtr msg)
  {
    string x;
    StringBuffer buffer;
    Writer<StringBuffer> writer(buffer);

    writer.StartObject();
    writer.Key("type");
    writer.String("FullPositionVector");
    writer.Key("data");
    writer.StartObject();
    writer.Key("VehicleIdentificationNumber");
    writer.String("ABC");
    writer.Key("presence_vector");
    writer.Int(msg->presence_vector);
    writer.Key("lon");
    writer.Double(msg->lon);
    writer.Key("lat");
    writer.Double(msg->lat);

    writer.Key("elevation");
    writer.Double(msg->elevation);
    writer.Key("heading");
    writer.Double(msg->heading);
    writer.Key("speed");
    writer.Double(msg->speed);
    writer.EndObject();
    writer.EndObject();
    this -> respond(POSITION_MESSAGE, buffer.GetString());
  }
 
  /*********************************************************************************/  
  /* Method:    handle_ads_health                                                  */
  /* Purpose:   This method convects the ADSSafety message to JSON then            */
  /*            transmits it.                                                      */
  /*********************************************************************************/  
  void CellularDriver::handle_ads_health(const carma_v2x_msgs::msg::ADSSafety::SharedPtr msg)
  {
     int i;
     string x;
     StringBuffer buffer;
     Writer<StringBuffer> writer(buffer);

std::cerr << "got ads_data from wayne truck\n";
     writer.StartObject();
     writer.Key("type");
     writer.String("ADSSafety");
     writer.Key("data");
     writer.StartObject();
     writer.Key("Vin");
     x = msg->vin_number;
     writer.String(x.c_str());

std::cerr << "got ads_data from wayne truck 1\n";
     writer.Key("Plate");
     writer.String((msg->license_plate).c_str());
     writer.Key("carrier_name");
     writer.String((msg->carrier_name).c_str());
     writer.Key("veh_insurance_status");
     writer.String((msg->veh_insurance_status).c_str());
     writer.Key("operator_name");
     writer.String((msg->operator_name).c_str());
     writer.Key("operator_contact_info");
     writer.String((msg->operator_contact_info).c_str());
     writer.Key("gross_axle_weight");
     writer.Double(msg->gross_axle_weight);
     writer.Key("gross_veh_weight");
     writer.Double(msg->gross_veh_weight);
     writer.Key("date_of_pre_trip_inspection_tractor");
     writer.String((msg->date_of_pre_trip_inspection_tractor).c_str());
     writer.Key("date_of_pre_trip_inspection_trailer");
     writer.String((msg->date_of_pre_trip_inspection_trailer).c_str());
std::cerr << "got ads_data from wayne truck 2\n";
/*
     writer.Key("tire_data");
     writer.StartObject();
       writer.Key("time");
       writer.Int(msg->tire_data.time);

       for (i=0;i<18;i++)
       {
         x = "tire_" + std::to_string(i); 
         writer.Key(x.c_str());
         writer.StartObject();
           writer.Key("pressure");
           writer.Double(msg->tire_data.tires[i].pressure); 
           writer.Key("error_time");
           writer.Double(msg->tire_data.tires[i].time_to_error); 
           writer.Key("condition");
           writer.Int(msg->tire_data.tires[i].condition);
         writer.EndObject();
       }
     writer.EndObject();
*/
     writer.Key("ads_status");
     writer.StartObject();
       writer.Key("ads_health_status");
       writer.Bool(msg->ads_status.ads_health_status);
     writer.EndObject();
std::cerr << "got ads_data from wayne truck 3\n";

     writer.Key("truck_operational_status");
std::cerr << "truck operational status = " << (msg->truck_operational_status).c_str() << "\n";
     writer.String((msg->truck_operational_status).c_str());
     writer.Key("abs_warning");
     writer.Bool(msg->abs_warning);

std::cerr << "got ads_data from wayne truck 4\n";
     writer.Key("destination");
     writer.String((msg->destination).c_str());
     writer.Key("roadside_inspection_facility");
     writer.String((msg->roadside_inspection_facility).c_str());
     writer.Key("level_of_inspection");
     writer.Int(msg->level_of_inspection);
std::cerr << "got ads_data from wayne truck 5\n";

     writer.EndObject();
     writer.EndObject();

     this -> respond(ADS_HEALTH_REQUESTED, buffer.GetString());
  }


  /*********************************************************************************/  
  /* Method:    respond                                                            */
  /* Purpose:   This method transmits a message to the socket upon which the       */
  /*            request was received.  The vector myMessageQueue maintains this    */
  /*            mapping.  In the case of a GPS position message, it is broadcast   */ 
  /*            to all sockets.                                                   */
  /*********************************************************************************/  
  void CellularDriver::respond(int type, const char *message) 
  {
    if (type != POSITION_MESSAGE)
    {
      map<int,WebSocket::pointer>::iterator iter;
      for(iter=myMessageQueue.begin(); iter != myMessageQueue.end(); )
      {
        if (iter -> first == type)
        {
          iter -> second -> send(message);
          myMessageQueue.erase(iter++);
        } 
        else
        {
          ++iter;
        }
      }
    }
    else
    {
      for (WebSocket::pointer x : myConnections)
      {
        x -> send(message);
      }
    }
  }
 
  /*********************************************************************************/  
  /* Method:    requestADSHealth                                                   */
  /* Purpose:   This method is called by the websocket handler method whenever     */
  /*            a request for the ADSSafety message is received.  It then requests */
  /*            the data by sending a request on the appropriate ROS topic         */ 
  /*********************************************************************************/  
  void CellularDriver::requestADSHealth(WebSocket *socket)
  {
    std::string type;
    std_msgs::msg::String message;
    

    type = "GET_ADS_HEALTH";
    message.data = "GET_ADS_HEALTH";
    myHealthRequestPublisher->publish(message);
    
    myMessageQueue.insert(pair<int, WebSocket *>(ADS_HEALTH_REQUESTED, socket));
  }
 
  /*********************************************************************************/  
  /* Method:    updatePrepass                                                      */
  /* Purpose:   This method is called by the websocket handler method whenever     */
  /*            a PrePass final decision message is received.  It simply send the  */
  /*            results to the appropriate ROS topic.                              */ 
  /*********************************************************************************/  
  void CellularDriver::updatePrepass(const char *decision)
  {
    std_msgs::msg::String message;

    message.data = decision;
    myPrepassPublisher -> publish(message); 
  }
  
  /*********************************************************************************/  
  /* Method:    operator()                                                         */
  /* Purpose:   This method is called whenever a websocket has data to be          */
  /*            processes.  A C++ functur is used so that the callback function    */
  /*            can have state, which consists of the CellularDriver               */ 
  /*********************************************************************************/  
  void WsHandler::operator()(const std::string &msg)
  {
    const char *type;
    Document document,document1;
  
    if (!document.Parse(msg.c_str()).HasParseError())
    {
      type = document["type"].GetString();
      if (strcmp(type,"ADS_REQUEST") == 0)
      {
        myDriver -> requestADSHealth(mySocket);
std::cerr << "got ads_request\n";
      }
      else if (strcmp(type,"PrePassDecision") == 0)
           {
std::cerr << "got prepass decision\n";
             document1.CopyFrom(document["data"], document.GetAllocator());
             myDriver -> updatePrepass(document1["decision"].GetString());
           }
    }
  }
}
