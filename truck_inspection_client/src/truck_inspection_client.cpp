/*
 * Copyright (C) 2019-2021 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */


#include "truck_inspection_client.h"

namespace truck_inspection_client
{

    void TruckInspectionClient::initialize()
    {
        nh_.reset(new ros::CARMANodeHandle());
        pnh_.reset(new ros::CARMANodeHandle("~"));
        pnh_->getParam("/vin_number", vin_number_); //get vin from global params in UniqueVehicleParams.ymal
        pnh_->getParam("license_plate", license_plate_);
        pnh_->getParam("usdot_number", usdot_number_);
        pnh_->getParam("state_short_name", state_short_name_);
        pnh_->getParam("carrier_name", carrier_name_);
        pnh_->getParam("carrier_id", carrier_id_);
        pnh_->getParam("weight", weight_);
        pnh_->getParam("date_of_last_state_inspection", date_of_last_state_inspection_);
        pnh_->getParam("date_of_last_ads_calibration", date_of_last_ads_calibration_);
        pnh_->getParam("iss_score", iss_score_);
        pnh_->getParam("permit_required", permit_required_);
        pnh_->getParam("pre_trip_ads_health_check", pre_trip_ads_health_check_);
        // Added for ADS safety data message
        pnh_->getParam("pre_trip_inspector", pre_trip_inspector_);
        pnh_->getParam("inspector_id", inspector_id_);
        pnh_->getParam("vehicle", vehicle_);
        pnh_->getParam("gross_axle_weight", gross_axle_weight_);
        pnh_->getParam("gross_veh_weight", gross_veh_weight_);
        pnh_->getParam("overweight_permit_status", overweight_permit_status_);
        pnh_->getParam("date_of_last_inspection", date_of_last_inspection_);
        pnh_->getParam("date_of_pre_trip_inspection_tractor", date_of_pre_trip_inspection_tractor_);
        pnh_->getParam("date_of_pre_trip_inspection_trailer", date_of_pre_trip_inspection_trailer_);
        pnh_->getParam("ifta_status", ifta_status_);
        pnh_->getParam("irp_status", irp_status_);
        pnh_->getParam("truck_operational_health", truck_operational_health_);
        pnh_->getParam("tractor_operational_health", tractor_operational_health_);
        pnh_->getParam("trailer_operational_health", trailer_operational_health_);
        pnh_->getParam("level_of_inspection", level_of_inspection_);
        pnh_->getParam("origin", origin_);
        pnh_->getParam("destination", destination_);
        pnh_->getParam("nearest_roadside_inspection_facility", nearest_roadside_inspection_facility_);
        pnh_->getParam("operational_time", operational_time_);

        mo_pub_ = nh_->advertise<cav_msgs::MobilityOperation>("outgoing_mobility_operation", 5);
        ads_safety_pub_ = nh_->advertise<cav_msgs::ADSSafety>("ads_safety_data", 5);
        request_sub_ = nh_->subscribe("incoming_mobility_request", 1, &TruckInspectionClient::requestCallback, this);
        ads_state_sub_ = nh_->subscribe("guidance/state", 1, &TruckInspectionClient::guidanceStatesCallback, this);
        ads_system_alert_sub_ = nh_->subscribe("system_alert", 1, &TruckInspectionClient::systemAlertsCallback, this);
        version_sub_ = nh_->subscribe("carma_system_version", 1, &TruckInspectionClient::versionCallback, this);
        bsm_sub_ = nh_->subscribe("bsm_outbound", 1, &TruckInspectionClient::bsmCallback, this);
        // subscribe ads data requests
        ads_health_request_sub_ = nh_->subscribe("ads_health_request", 1, &TruckInspectionClient::adsHealthRequestCallback, this);
        ads_pretrip_request_sub_ = nh_->subscribe("ads_pretrip_request", 1, &TruckInspectionClient::adsPreTripRequestCallback, this);

        this->ads_engaged_ = false;
        this->ads_system_alert_type_ = std::to_string(cav_msgs::SystemAlert::NOT_READY);
        this->ads_software_version_ = "System Version Unknown";
        this->driver_status_="s_0_l1_0_l2_0_g_0_c_0";
        //Checking: vin number does not exist yet in global parameter server
        while((!pnh_->getParam("/vin_number", vin_number_)) && vin_retrive_count < MAX_RETRIEVE_VIN_COUNT)
        {
            //sleep for 0.1 second
            boost::this_thread::sleep_for(boost::chrono::milliseconds(100));
            vin_retrive_count++;

            if(pnh_->getParam("/vin_number", vin_number_))
            {
                ROS_INFO("retrieved vin_number1:  %s",vin_number_.c_str());
                break; 
            }
            ROS_DEBUG("retrieving vin_number:  %s",vin_number_.c_str());
        }
        //if exceed maximum count, publish warning messages to system_alert
        if(vin_retrive_count >= MAX_RETRIEVE_VIN_COUNT)
        {
            cav_msgs::SystemAlert alert_msg;
            alert_msg.type = cav_msgs::SystemAlert::WARNING;
            alert_msg.description = ros::this_node::getName() + ": vin_number does not exist in global parameter server.";                
            ROS_ERROR_STREAM(alert_msg.description);
            nh_->publishSystemAlert(alert_msg);
        }

        ros::CARMANodeHandle::setSpinRate(20.0);
        // set vin publisher
        ros::CARMANodeHandle::setSpinCallback([this]() -> bool {
            cav_msgs::MobilityOperation msg_out;
            msg_out.strategy = this->INSPECTION_STRATEGY;
            msg_out.strategy_params = "vin_number:" + vin_number_ + ",license_plate:" + license_plate_ +",state_short_name:"+ state_short_name_;
            mo_pub_.publish(msg_out);
            return true;
        });
        ROS_INFO_STREAM("Truck inspection plugin is initialized...");
    }

    void TruckInspectionClient::run()
    {
        initialize();
        ros::CARMANodeHandle::spin();
    }

    void TruckInspectionClient::bsmCallback(const cav_msgs::BSMConstPtr& msg)
    {
        auto id_vector = msg->core_data.id;
        bsm_id_ = "";
        for(auto c : id_vector) bsm_id_ += std::to_string(c);

        current_lat_ = core_data.latitude;
        current_lon_ = core_data.longitude;
    }

    void TruckInspectionClient::guidanceStatesCallback(const cav_msgs::GuidanceStateConstPtr& msg)
    {
        this->ads_engaged_ = (msg->state == cav_msgs::GuidanceState::ENGAGED);
    }
    void TruckInspectionClient::systemAlertsCallback(const cav_msgs::SystemAlertConstPtr& msg){
        this->ads_system_alert_type_ = std::to_string(msg->type);
        this->driver_status_= std::to_string(msg->driver_status);
    }
    void TruckInspectionClient::requestCallback(const cav_msgs::MobilityRequestConstPtr& msg)
    {
        if(msg->strategy == this->INSPECTION_STRATEGY) {           
            cav_msgs::MobilityOperation mo_msg;
            mo_msg.m_header.sender_bsm_id = bsm_id_;
            mo_msg.strategy = this->INSPECTION_STRATEGY;
            std::string ads_auto_status = this->ads_engaged_ ? "Engaged" : "Not Engaged";
            std::string ads_health_status = ads_system_alert_type_ ;
            std::string params = boost::str(boost::format("vin_number:%s,license_plate:%s,carrier_name:%s,carrier_id:%s,weight:%d,ads_software_version:%s,date_of_last_state_inspection:%s,date_of_last_ads_calibration:%s,pre_trip_ads_health_check:%s,ads_health_status:%s,ads_auto_status:%s,iss_score:%d,permit_required:%s")
                                                         % vin_number_ % license_plate_ % carrier_name_ % carrier_id_ % weight_ % ads_software_version_ % date_of_last_state_inspection_ % date_of_last_ads_calibration_ % pre_trip_ads_health_check_ % ads_health_status % ads_auto_status % iss_score_ % permit_required_);
            long time = (long)(ros::Time::now().toNSec() / pow(10, 6));
            mo_msg.m_header.timestamp = time;
            mo_msg.strategy_params = params;
            mo_pub_.publish(mo_msg);
        }
    }    
    void TruckInspectionClient::versionCallback(const std_msgs::StringConstPtr& msg)
    {
        this->ads_software_version_ = msg->data;
    }
    // ADS health data request callback
    void TruckInspectionClient::adsHealthRequestCallback(const std_msgs::StringConstPtr& msg)
    {
        cav_msgs::ADSSafety ads_health_msg;

        if (msg->type == "GET_ADS_HEALTH"){
            long time = (long)(ros::Time::now().toNSec() / pow(10, 6));
            ads_health_msg.type = "ADS Health and Status";
            ads_health_msg.m_header.timestamp = time;
            ads_health_msg.vin_number = vin_number_;
            ads_health_msg.license_plate = license_plate_;
            ads_health_msg.latitude = current_lat_;
            ads_health_msg.longitude = current_lon_;
            ads_health_msg.ads_status = adsHealthStatus(ads_system_alert_type_);
            ads_health_msg.operational_time = operational_time_;
            ads_health_msg.truck_operational_health = truck_operational_health_;
            ads_safety_pub_.publish(ads_health_msg);
        }
    }
    // ADS PreTrip data request callback
    void TruckInspectionClient::adsPreTripRequestCallback(const std_msgs::StringConstPtr& msg)
    {
        cav_msgs::ADSSafety ads_pretrip_msg;
        adsHealthStatus(ads_system_alert_type_);

        long time = (long)(ros::Time::now().toNSec() / pow(10, 6));
        ads_pretrip_msg.m_header.timestamp = time;
        ads_pretrip_msg.type = "Pretripinput";
        ads_health_msg.latitude = current_lat_;
        ads_health_msg.longitude = current_lon_;
        ads_pretrip_msg.pre_trip_inspector = pre_trip_inspector_;
        ads_pretrip_msg.inspector_id = inspector_id_;
        ads_pretrip_msg.vehicle = vehicle_;
        ads_pretrip_msg.vin = vin_number_;
        ads_pretrip_msg.license_plate = license_plate_;
        ads_pretrip_msg.state = state_short_name_;
        ads_pretrip_msg.carrier_name = carrier_name_;
        ads_pretrip_msg.carrier_id = carrier_id_;
        ads_pretrip_msg.usdot_number = usdot_number_;
        ads_pretrip_msg.gross_axle_weight = gross_axle_weight_;
        ads_pretrip_msg.gross_veh_weight = gross_veh_weight_;
        ads_pretrip_msg.overweight_permit_status = overweight_permit_status_;
        ads_pretrip_msg.date_of_last_inspection = date_of_last_inspection_;
        ads_pretrip_msg.date_of_pre_trip_inspection_tractor = date_of_pre_trip_inspection_tractor_;
        ads_pretrip_msg.date_of_pre_trip_inspection_trailer = date_of_pre_trip_inspection_trailer_;
        ads_pretrip_msg.iss_score = iss_score_;
        ads_pretrip_msg.ifta_status = ifta_status_;
        ads_pretrip_msg.irp_status = irp_status_;
        ads_pretrip_msg.ads_status = adsHealthStatus(ads_system_alert_type_);
        ads_pretrip_msg.truck_operational_health = truck_operational_health_;
        ads_pretrip_msg.tractor_operational_health = tractor_operational_health_;
        ads_pretrip_msg.trailer_operational_health = trailer_operational_health_;
        ads_pretrip_msg.level_of_inspection = level_of_inspection_;
        ads_pretrip_msg.origin = origin_;
        ads_pretrip_msg.destination = destination_;
        ads_pretrip_msg.nearest_roadside_inspection_facility = nearest_roadside_inspection_facility_;
        ads_pretrip_msg.preclearance_system = "PrePass";
        ads_safety_pub_.publish(ads_pretrip_msg);
    }

    cav_msgs::ADSStatus TruckInspectionClient::adsHealthStatus(string ads_system_alert_type)
    {
        cav_msgs::ADSStatus ads_status;
        switch(ads_system_alert_type)
        {
            case std::to_string(cav_msgs::SystemAlert::CAUTION):
                ads_status.ads_health_status_ = "yellow";
                ads_status.ads_operational_status_ = cav_msgs::ADSStatus::ADS_CAUTION;
                break;
            case std::to_string(cav_msgs::SystemAlert::WARNING):
                ads_status.ads_health_status_ = "yellow";
                ads_status.ads_operational_status_ = cav_msgs::ADSStatus::ADS_WARNING;
                break;
            case std::to_string(cav_msgs::SystemAlert::FATAL):
                ads_status.ads_health_status_ = "red";
                ads_status.ads_operational_status_ = cav_msgs::ADSStatus::ADS_FATAL;
                break;
            case std::to_string(cav_msgs::SystemAlert::NOT_READY):
                ads_status.ads_health_status_ = "yellow";
                ads_status.ads_operational_status_ = cav_msgs::ADSStatus::ADS_NOT_READY;
                break;
            case std::to_string(cav_msgs::SystemAlert::DRIVERS_READY):
                ads_status.ads_health_status_ = "green";
                ads_status.ads_operational_status_ = cav_msgs::ADSStatus::ADS_READY;
                break;
            case std::to_string(cav_msgs::SystemAlert::SHUTDOWN):
                ads_status.ads_health_status_ = "red";
                ads_status.ads_operational_status_ = cav_msgs::ADSStatus::ADS_FATAL;
                break;
            default:
                break;
        }
        ads_status.driver_status = driver_status_;
        
        return ads_status;
    }

}
