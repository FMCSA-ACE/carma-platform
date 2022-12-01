Purpose:  Implements the Tire Management Management System function.  The plugin uses curlpp (curlpp.org) for pulling data from the cloud using curl and rapidjson (rapidjson.org) for parsing the TPMS data received from the cloud.  This data is the tire pressure for each tire

After pulling the data from the cloud, it saves it in a file for future processing.  The cloud does not maintain the data once it is pulled.           
                                                   
Configuration:  TPMS Cloud URL, filename for storing TPMS historical data

config/parameters.yaml:

tpms_plugin:
  ros_parameters:
    tpms_cloud_url:    "http://192.168.1.71/tire3.txt"
    tpms_data_file:    "tiredata.dat"

Launch command:  “ros2 launch tpms_plugin tpms_launch.py”


