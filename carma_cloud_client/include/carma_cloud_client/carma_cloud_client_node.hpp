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


#pragma once

#define CURL_STATICLIB 

// #include <QCommandLineOption>
// #include <QCommandLineParser>
// #include <QCoreApplication>
// #include <QHostAddress>
// #include <QRegExp>
// #include <QStringList>
// #include <QSharedPointer>
// #include <QObject>

#include <rclcpp/rclcpp.hpp>
#include <functional>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/empty.hpp>
#include <curl/curl.h>
#include <curl/easy.h>
#include <j2735_convertor/control_request_convertor.hpp>
#include <stdio.h>
#include <cstdio>
#include <iostream>
#include <zlib.h>

#include <QCoreApplication>
#include <QHostAddress>
#include <QVariantMap>

#include <qhttpengine/handler.h>
#include <qhttpengine/localauthmiddleware.h>
#include <qhttpengine/qobjecthandler.h>
#include <qhttpengine/server.h>
#include <qhttpengine/socket.h>

#include <carma_ros2_utils/carma_lifecycle_node.hpp>
#include "carma_cloud_client/carma_cloud_client_config.hpp"
#include <j2735_v2x_msgs/msg/traffic_control_request.hpp>
#include <carma_v2x_msgs/msg/traffic_control_request.hpp>
#include <carma_v2x_msgs/msg/traffic_control_message.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>

#include <v2xhubWebAPI/OAIApiRouter.h>

// using namespace OpenAPI;

namespace carma_cloud_client
{

  /**
   * \brief TODO for USER: Add class description
   * 
   */
  class CarmaCloudClient : public carma_ros2_utils::CarmaLifecycleNode
  {

  private:
    // TCR Subscriber
    carma_ros2_utils::SubPtr<carma_v2x_msgs::msg::TrafficControlRequest> tcr_sub_;
    carma_ros2_utils::PubPtr<carma_v2x_msgs::msg::TrafficControlMessage> tcm_pub_;

    // Node configuration
    Config config_;

    const char *CONTENT_ENCODING_KEY = "Content-Encoding";
    const char *CONTENT_ENCODING_VALUE = "gzip";
    


    const double DECI_MPS_PER_MPS = 10.0;
    const double DECI_S_PER_S = 10.0;
    const double MS_PER_S = 1000.0;
    const double CM_PER_M = 100.0;
    const double TENTH_MICRO_DEG_PER_DEG = 10000000.0;
    const double DECI_M_PER_M = 10.0;
    const double TWENTIETH_M_PER_M = 20.0;
    const double FIFTIETH_M_PER_M = 50.0;
    const double FIFTIETH_G_PER_M_PER_SEC_SQR = 5.10204081633;
    const double ONE_AND_A_HALF_DEG = 1.5;
    const double ONE_AND_A_HALF_DEG_PER_DEG = 0.666666666666;
    const double CENTI_DEG_PER_DEG = 100.0;
    const double THREE_TENTHS_DEG = 0.3;
    const double EIGHTIETH_DEG_PER_DEG = 80.0;
    const double DEG_360_OVER_65535_PER_DEG = 182.041666097;
    const double UNCHANGED = 1.0;
    const double SEC_PER_MIN = 60.0;
    const double DECA_DEG_PER_DEG = 10.0;

    // Integer Varients
    const uint64_t NS_PER_MS_INT = 1000000;
    
  public:
    /**
     * \brief Node constructor 
     */
    explicit CarmaCloudClient(const rclcpp::NodeOptions &);

    /**
     * \brief callback for dynamic parameter updates
     * \param parameters list of parameters
     */
    rcl_interfaces::msg::SetParametersResult 
    parameter_update_callback(const std::vector<rclcpp::Parameter> &parameters);


    /**
     * \brief TCR subscription callback
     * \param msg Traffic Control Request pointer
     */
    void tcr_callback(carma_v2x_msgs::msg::TrafficControlRequest::UniquePtr msg);

    /**
     * \brief Funtion to Convert the TCR into XML format
     * \param xml_str array of characters for xml format
     * \param request_msg input TCR msg
     */
    void XMLconversion(char* xml_str, carma_v2x_msgs::msg::TrafficControlRequest request_msg);

    /**
     * \brief Send http request to carma cloud
     * \param local_msg msg to be sent to cloud
     * \param local_url url to cloud
     * \param local_base base to be added to url
     * \param local_method method
     */
    int CloudSend(const std::string &local_msg, const std::string& local_url, const std::string& local_base, const std::string& local_method);

    /**
     * \brief Send async http request to carma cloud
     * \param local_msg msg to be sent to cloud
     * \param local_url url to cloud
     * \param local_base base to be added to url
     * \param local_method method
     */
    void CloudSendAsync(const std::string& local_msg,const std::string& local_url, const std::string& local_base, const std::string& local_method);

    void TCMHandler(QHttpEngine::Socket *socket);

    int StartWebService();

    carma_v2x_msgs::msg::TrafficControlMessage parseTCMXML(boost::property_tree::ptree& tree);

    unsigned char parse_hex(char c);

    std::vector<unsigned char> parse_string(const std::string & s);

    j2735_v2x_msgs::msg::TrafficControlPackage parse_package(boost::property_tree::ptree& tree);

    carma_v2x_msgs::msg::TrafficControlParams parse_params(boost::property_tree::ptree& tree);
    
    carma_v2x_msgs::msg::TrafficControlGeometry parse_geometry(boost::property_tree::ptree& tree);

    carma_v2x_msgs::msg::TrafficControlSchedule parse_schedule(boost::property_tree::ptree& tree);

    carma_v2x_msgs::msg::TrafficControlDetail parse_detail(boost::property_tree::ptree& tree);
    
    QByteArray UncompressBytes(const QByteArray compressedBytes) const;


    ////
    // Overrides
    ////
    carma_ros2_utils::CallbackReturn handle_on_configure(const rclcpp_lifecycle::State &prev_state);

  };

} // carma_cloud_client
