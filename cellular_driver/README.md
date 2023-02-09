Purpose:  Implements the management of the WebSocket connections to the external entities.  The class uses easywsclient David Baird rapidjson (rapidjson.org) for parsing/creating json documents.      
  
Configuration:  list of connection strings (urls) for each external entity (PrePass Final Decision, RoadSide Inspection Station, and SafeSpect)

config/parameters.yaml:  

cellular_driver:
  ros_parameters:
    connection_strings: [ "ws://192.168.1.71:8080/PrePass/PrePassDecision/truck",
                          "ws://192.168.1.71:8080/SafeSpect/SafeSpect"]

Launch command:    “ros2 launch cellular_driver cell_launch.py”
