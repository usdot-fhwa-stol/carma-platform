/*
 * Copyright (C) 2022 LEIDOS.
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
#include "carma_cloud_client/carma_cloud_client_node.hpp"

namespace carma_cloud_client
{
  namespace std_ph = std::placeholders;

  CarmaCloudClient::CarmaCloudClient(const rclcpp::NodeOptions &options)
      : carma_ros2_utils::CarmaLifecycleNode(options)
  {
    // Create initial config
    config_ = Config();

    // Declare parameters
    config_.url = declare_parameter<std::string>("url", config_.url);
    config_.base_req = declare_parameter<std::string>("base_req", config_.base_req);
    config_.base_ack = declare_parameter<std::string>("base_ack", config_.base_ack);
    config_.port = declare_parameter<std::string>("port", config_.port);
    config_.list = declare_parameter<std::string>("list", config_.list);
    config_.method = declare_parameter<std::string>("method", config_.method);
    config_.fetchtime = declare_parameter<int>("fetchtime", config_.fetchtime);
  }

  rcl_interfaces::msg::SetParametersResult CarmaCloudClient::parameter_update_callback(const std::vector<rclcpp::Parameter> &parameters)
  {
    
    auto error = update_params<std::string>({{"url", config_.url}, 
                                             {"base_req", config_.base_req},
                                             {"base_ack", config_.base_ack},
                                             {"port", config_.port},
                                             {"list", config_.list},
                                             {"method", config_.method}} , parameters);

    rcl_interfaces::msg::SetParametersResult result;

    result.successful = !error;

    return result;
  }

  carma_ros2_utils::CallbackReturn CarmaCloudClient::handle_on_configure(const rclcpp_lifecycle::State &)
  {
    RCLCPP_DEBUG_STREAM(this->get_logger(), "CarmaCloudClient trying to configure");
    // Reset config
    config_ = Config();

    // Load parameters
    get_parameter<std::string>("url", config_.url);
    get_parameter<std::string>("base_req", config_.base_req);
    get_parameter<std::string>("base_ack", config_.base_ack);
    get_parameter<std::string>("port", config_.port);
    get_parameter<std::string>("list", config_.list);
    get_parameter<std::string>("method", config_.method);
    get_parameter<int>("fetchtime", config_.fetchtime);

    // Register runtime parameter update callback
    add_on_set_parameters_callback(std::bind(&CarmaCloudClient::parameter_update_callback, this, std_ph::_1));

    // Setup subscribers
    tcr_sub_ = create_subscription<carma_v2x_msgs::msg::TrafficControlRequest>("outgoing_geofence_request", 10,
                                                              std::bind(&CarmaCloudClient::tcr_callback, this, std_ph::_1));                                                     


    // Return success if everything initialized successfully
    return CallbackReturn::SUCCESS;
  }

  void CarmaCloudClient::tcr_callback(carma_v2x_msgs::msg::TrafficControlRequest::UniquePtr msg)
  {
    carma_v2x_msgs::msg::TrafficControlRequest request_msg(*msg.get());

    char xml_str[10000]; 

    XMLconversion(xml_str, request_msg);

    CloudSend(xml_str, config_.url, config_.base_req, config_.method);
    
    RCLCPP_DEBUG_STREAM(  get_logger(), "tcr_sub_ callback called ");
  }

  void CarmaCloudClient::XMLconversion(char* xml_str, carma_v2x_msgs::msg::TrafficControlRequest request_msg)
  {
    j2735_v2x_msgs::msg::TrafficControlRequest j2735_tcr;

    j2735_convertor::geofence_request::convert(request_msg, j2735_tcr);

    RCLCPP_DEBUG_STREAM(  get_logger(), "converted: "); 

    size_t hexlen = 2; //size of each hex representation with a leading 0
    char reqid[j2735_tcr.tcr_v01.reqid.id.size() * hexlen + 1];
    for (int i = 0; i < j2735_tcr.tcr_v01.reqid.id.size(); i++)
    {
      sprintf(reqid+(i*hexlen), "%.2X", j2735_tcr.tcr_v01.reqid.id[i]);
    }

	  RCLCPP_DEBUG_STREAM(  get_logger(), "reqid: " << reqid);
  
    long int reqseq = j2735_tcr.tcr_v01.reqseq;
    RCLCPP_DEBUG_STREAM(  get_logger(), "reqseq: " << reqseq);

	  long int scale = j2735_tcr.tcr_v01.scale;
    RCLCPP_DEBUG_STREAM(  get_logger(), "scale: " << scale);

    int totBounds =  j2735_tcr.tcr_v01.bounds.size();
    int cnt=0;
    char bounds_str[5000];
		strcpy(bounds_str,"");

    //  get current time 
    std::time_t tm = this->now().seconds()/60 - config_.fetchtime*24*60; //  T minus fetchtime*24 hours in  min  

    while(cnt<totBounds)
    {

      uint32_t oldest = tm;
      long lat = j2735_tcr.tcr_v01.bounds[cnt].reflat; 
      long longg = j2735_tcr.tcr_v01.bounds[cnt].reflon;

    
      long dtx0 = j2735_tcr.tcr_v01.bounds[cnt].offsets[0].deltax;
      long dty0 = j2735_tcr.tcr_v01.bounds[cnt].offsets[0].deltay;
      long dtx1 = j2735_tcr.tcr_v01.bounds[cnt].offsets[1].deltax;
      long dty1 = j2735_tcr.tcr_v01.bounds[cnt].offsets[1].deltay;
      long dtx2 = j2735_tcr.tcr_v01.bounds[cnt].offsets[2].deltax;
      long dty2 = j2735_tcr.tcr_v01.bounds[cnt].offsets[2].deltay;

      int x = std::sprintf(bounds_str+strlen(bounds_str),"<bounds><TrafficControlBounds><oldest>%ld</oldest><reflon>%ld</reflon><reflat>%ld</reflat><offsets><OffsetPoint><deltax>%ld</deltax><deltay>%ld</deltay></OffsetPoint><OffsetPoint><deltax>%ld</deltax><deltay>%ld</deltay></OffsetPoint><OffsetPoint><deltax>%ld</deltax><deltay>%ld</deltay></OffsetPoint></offsets></TrafficControlBounds></bounds>",oldest,longg,lat,dtx0,dty0,dtx1,dty1,dtx2,dty2);

      cnt++;

    }	

    

    char port[config_.port.size() + 1];
    strcpy(port, config_.port.c_str());

    char list[config_.list.size() + 1];
    strcpy(list, config_.list.c_str());
 
    // with port and list
    sprintf(xml_str,"<?xml version=\"1.0\" encoding=\"UTF-8\"?><TrafficControlRequest port=\"%s\" list=\"%s\"><reqid>%s</reqid><reqseq>%ld</reqseq><scale>%ld</scale>%s</TrafficControlRequest>",port, list, reqid, reqseq,scale,bounds_str);

    RCLCPP_DEBUG_STREAM(  get_logger(), "xml_str: " << xml_str);

  }

  int CarmaCloudClient::CloudSend(const std::string &local_msg, const std::string& local_url, const std::string& local_base, const std::string& local_method)
  { 	
    CURL *req;
    CURLcode res;
    std::string urlfull = local_url + config_.port + local_base;	
    RCLCPP_DEBUG_STREAM(  get_logger(), "full url: " << urlfull);
    req = curl_easy_init();
    if(req) {
      curl_easy_setopt(req, CURLOPT_URL, urlfull.c_str());

      if(strcmp(local_method.c_str(),"POST")==0)
      {
        curl_easy_setopt(req, CURLOPT_POSTFIELDS, local_msg.c_str());
        curl_easy_setopt(req, CURLOPT_TIMEOUT_MS, 1000L); // Request operation complete within max millisecond timeout 
        res = curl_easy_perform(req);
        if(res != CURLE_OK)
        {
          RCLCPP_ERROR_STREAM(  get_logger(), "curl send failed: " << curl_easy_strerror(res));
          return 1;
        }	  
      }
      curl_easy_cleanup(req);
    }	
      
    return 0;
  }

  void CarmaCloudClient::CloudSendAsync(const std::string& local_msg,const std::string& local_url, const std::string& local_base, const std::string& local_method)
  {
    std::thread t([this, &local_msg, &local_url, &local_base, &local_method](){	
      CloudSend(local_msg, local_url, local_base, local_method);	
    });
    t.detach();
  }


} // carma_cloud_client

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(carma_cloud_client::CarmaCloudClient)
