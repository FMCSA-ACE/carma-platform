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

#include <math.h>
#include <cmath> 
#define earthRadiusKm 6371.0


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
        this->create_publisher<std_msgs::msg::Bool>("prepass_decision",10);
    myHealthRequestPublisher = 
        this->create_publisher<std_msgs::msg::String>("ads_health_request",10);

    myHealthSubscription = 
      this->create_subscription<carma_v2x_msgs::msg::ADSSafety>(
      "ads_safety_data", 10, std::bind(&CellularDriver::handle_ads_health, this, _1));
    
    myPositionSubscription = 
      this->create_subscription<gps_msgs::msg::GPSFix>("gnss_fix_fused", 2,
                                                          std::bind(&CellularDriver::handle_position, this, _1));

  }
  
  // /*********************************************************************************/  
  // /* Method:    handle_position                                                    */
  // /* Purpose:   This method convects the GPS position message to JSON then         */
  // /*            transmits it.                                                      */
  /*********************************************************************************/  
  void CellularDriver::handle_position(
     const gps_msgs::msg::GPSFix::SharedPtr msg)
  {
    const double prepass_lat = 37.187007;
    const double prepass_lon = -80.394843;
    double lat = msg.latitude;
    double lon = msg.longitude;

    double distance = distanceEarth(prepass_lat, prepass_lon, lat, lon);
    if (distance < 0.01){
      string x;
      StringBuffer buffer;
      Writer<StringBuffer> writer(buffer);

      writer.StartObject();
      writer.Key("Geofence");
      writer.Int(1);
      writer.EndObject();
      this -> respond(POSITION_MESSAGE, buffer.GetString());
    }
  }
    
  
  double distanceEarth(double lat1d, double lon1d, double lat2d, double lon2d) {
    double lat1r, lon1r, lat2r, lon2r, u, v;
    lat1r = (lat1d * M_PI / 180);
    lon1r = (lon1d * M_PI / 180);
    lat2r = (lat2d * M_PI / 180);
    lon2r = (lon2d * M_PI / 180);
    u = sin((lat2r - lat1r)/2);
    v = sin((lon2r - lon1r)/2);
    return 2.0 * earthRadiusKm * asin(sqrt(u * u + cos(lat1r) * cos(lat2r) * v * v));
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

std::cerr << "got ads_data from truck\n";
     writer.StartObject();
     writer.Key("type");
     writer.String("ADSSafety");
     writer.Key("data");
     writer.StartObject();

     writer.Key("pre_trip_inspector");
     writer.String((msg->pre_trip_inspector).c_str());
     writer.Key("inspector_id");
     writer.String((msg->inspector_id).c_str());
     writer.Key("vehicle"); 
     writer.String((msg->vehicle).c_str());
     writer.Key("vin");
     writer.String((msg->vin_number).c_str());
     writer.Key("license_plate");
     writer.String((msg->license_plate).c_str());
     writer.Key("state");
     writer.String((msg->state).c_str());
     writer.Key("carrier_name");
     writer.String((msg->carrier_name).c_str());
     writer.Key("carrier_id");
     writer.String((msg->carrier_id).c_str());
     writer.Key("usdot_number");
     writer.String((msg->usdot_number).c_str());
     writer.Key("gross_axle_weight");
     writer.String((msg->gross_axle_weight).c_str());
     writer.Key("gross_veh_weight");
     writer.String((msg->gross_veh_weight).c_str());
     writer.Key("overweight_permit_status");
     writer.String((msg->overweight_permit_status).c_str());
     writer.Key("date_of_last_inspection");
     writer.String((msg->date_of_last_inspection).c_str());
     writer.Key("date_of_pre_trip_inspection_tractor");
     writer.Double(msg->date_of_pre_trip_inspection_tractor);
     writer.Key("date_of_pre_trip_inspection_trailer");
     writer.Double(msg->date_of_pre_trip_inspection_trailer);
     writer.Key("iss_score");
     writer.String((msg->iss_score).c_str());
     writer.Key("ifta_status");
     writer.String((msg->ifta_status).c_str());
     writer.Key("irp_status");
     writer.String((msg->irp_status).c_str());
     writer.Key("ifta_status");
     writer.String((msg->ifta_status).c_str());
     writer.Key("truck_operational_status");
     writer.String((msg->truck_operational_health).c_str());
     writer.Key("tractor_operational_health");
     writer.String((msg->tractor_operational_health).c_str());
     writer.Key("trailer_operational_health");
     writer.String((msg->trailer_operational_health).c_str());
     writer.Key("level_of_inspection");
     writer.String((msg->level_of_inspection).c_str());
     writer.Key("origin");
     writer.String((msg->origin).c_str());
     writer.Key("destination");
     writer.String((msg->destination).c_str());
     writer.Key("nearest_roadside_inspection_facility");
     writer.String((msg->nearest_roadside_inspection_facility).c_str());
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
    std_msgs::msg::Bool message;

    if (decision == "Pullin"){
      message.data = true;
    } else {
      message.data = false;
    }
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
