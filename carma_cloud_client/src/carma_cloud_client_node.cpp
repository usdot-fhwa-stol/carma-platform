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
    config_.webport = declare_parameter<uint16_t>("webport", config_.webport);
    config_.webip = declare_parameter<std::string>("webip", config_.webip);

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
    get_parameter<std::string>("webip", config_.webip);
    get_parameter<uint16_t>("webport", config_.webport);

    // Register runtime parameter update callback
    add_on_set_parameters_callback(std::bind(&CarmaCloudClient::parameter_update_callback, this, std_ph::_1));
    // Setup subscribers
    tcr_sub_ = create_subscription<carma_v2x_msgs::msg::TrafficControlRequest>("outgoing_geofence_request", 10,
                                                              std::bind(&CarmaCloudClient::tcr_callback, this, std_ph::_1));                                                     
    tcm_pub_ = create_publisher<j2735_v2x_msgs::msg::TrafficControlMessage>("incoming_j2735_geofence_control", 1);
    std::thread webthread(&CarmaCloudClient::StartWebService,this);
    webthread.detach(); // wait for the thread to finish 
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

  QByteArray CarmaCloudClient::UncompressBytes(const QByteArray compressedBytes) const
  {
    z_stream strm;
    strm.zalloc = nullptr;//Refer to zlib docs (https://zlib.net/zlib_how.html)
    strm.zfree = nullptr; 
    strm.opaque = nullptr;
    strm.avail_in = compressedBytes.size();
    strm.next_in = (Byte *)compressedBytes.data();
	  //checking input z_stream to see if there is any error, eg: invalid data etc.
    auto err = inflateInit2(&strm, MAX_WBITS + 16); // gzip input
    QByteArray outBuf;
	  //MAX numbers of bytes stored in a buffer 
    const int BUFFER_SIZE = 4092;
	  //There is successful, starting to decompress data
    if (err == Z_OK) 
    {
      int isDone = 0;
      do
      {
        char buffer[BUFFER_SIZE] = {0};
        strm.avail_out = BUFFER_SIZE;
        strm.next_out = (Byte *)buffer;
			  //Uncompress finished
        isDone = inflate(&strm, Z_FINISH);
        outBuf.append(buffer);
      } while (Z_STREAM_END != isDone); //Reach the end of stream to be uncompressed 
    }
    else
    {
		  RCLCPP_DEBUG_STREAM(  get_logger(), "Error initalize stream. Err code = " << err);
    }
    //Finished decompress data stream
    inflateEnd(&strm);
    return outBuf;
  }

  void CarmaCloudClient::TCMHandler(QHttpEngine::Socket *socket)
  {
    QString st; 
    while(socket->bytesAvailable()>0)
    {	
      auto readBytes = socket->readAll();
      if (socket->headers().keys().contains(CONTENT_ENCODING_KEY) && std::string(socket->headers().constFind(CONTENT_ENCODING_KEY).value().data()) == CONTENT_ENCODING_VALUE)
      {
        //readBytes is compressed in gzip format
        st.append(UncompressBytes(readBytes));			
      }
      else
      {
			  st.append(readBytes);
		  }

    }
    QByteArray array = st.toLocal8Bit();

    char* _cloudUpdate = array.data(); // would be the cloud update packet, needs parsing
    
    
    std::string tcm_string = _cloudUpdate;
    
    RCLCPP_DEBUG_STREAM(  get_logger(), "Received TCM from cloud");
    RCLCPP_DEBUG_STREAM(  get_logger(), "TCM in XML format: " << tcm_string);
    if(tcm_string.length() == 0)
    {
      RCLCPP_DEBUG_STREAM(  get_logger(), "Received TCM length is zero, and skipped.");
      return;
    }

    boost::property_tree::ptree list_tree;
    std :: stringstream ss; 
    ss << tcm_string;
    read_xml(ss, list_tree);
    

    auto child_tcm_list = list_tree.get_child_optional("TrafficControlMessageList");

    if (!child_tcm_list)
    {
      auto child_tree = list_tree.get_child("TrafficControlMessage");
      j2735_v2x_msgs::msg::TrafficControlMessage parsed_tcm = parseTCMXML(child_tree);
      tcm_pub_->publish(parsed_tcm);
    }
    else
    {
      auto tcm_list = child_tcm_list.get();

      BOOST_FOREACH(auto &node, list_tree.get_child("TrafficControlMessageList"))
      {
        j2735_v2x_msgs::msg::TrafficControlMessage parsed_tcm = parseTCMXML(node.second);
        tcm_pub_->publish(parsed_tcm);
      }

    }

 
  }

  j2735_v2x_msgs::msg::TrafficControlMessage CarmaCloudClient::parseTCMXML(boost::property_tree::ptree& tree)
  {
    j2735_v2x_msgs::msg::TrafficControlMessage tcm;

    auto child_tcmv01 = tree.get_child_optional("tcmV01");
    if (!child_tcmv01)
    {
      tcm.choice = j2735_v2x_msgs::msg::TrafficControlMessage::RESERVED;
      return tcm;
    }
    else tcm.choice = j2735_v2x_msgs::msg::TrafficControlMessage::TCMV01;

    std::string reqid_string = tree.get<std::string>("tcmV01.reqid");
    j2735_v2x_msgs::msg::Id64b output;
    for (std::size_t i = 0; i != reqid_string.size() / 2; ++i)
    {
      tcm.tcm_v01.reqid.id[i] = 16 * parse_hex(reqid_string[2 * i]) + parse_hex(reqid_string[2 * i + 1]);
    }

    tcm.tcm_v01.reqseq = tree.get<uint8_t>("tcmV01.reqseq");
    tcm.tcm_v01.msgtot = tree.get<uint16_t>("tcmV01.msgtot");
    tcm.tcm_v01.msgnum = tree.get<uint16_t>("tcmV01.msgnum");

    std::string id_string = tree.get<std::string>("tcmV01.id");
    for (std::size_t i = 0; i != id_string.size() / 2; ++i)
    {
      tcm.tcm_v01.id.id[i] = 16 * parse_hex(id_string[2 * i]) + parse_hex(id_string[2 * i + 1]);
    }

    tcm.tcm_v01.updated = tree.get<uint64_t>("tcmV01.updated");

    auto tree_package = tree.get_child_optional("tcmV01.package");
    if (!tree_package)
    {
      tcm.tcm_v01.package_exists = false;
    }
    else
    {
      tcm.tcm_v01.package_exists = true;
      tcm.tcm_v01.package = parse_package(tree_package.get());
    }

    auto tree_params = tree.get_child_optional("tcmV01.params");
    if (!tree_params)
    {
      tcm.tcm_v01.params_exists = false;
    }
    else
    {
      tcm.tcm_v01.params_exists = true;
      tcm.tcm_v01.params = parse_params(tree_params.get());
    }

    auto tree_geometry = tree.get_child_optional("tcmV01.geometry");
    if (!tree_geometry)
    {
      tcm.tcm_v01.geometry_exists = false;
    }
    else
    {
      tcm.tcm_v01.geometry_exists = true;
      tcm.tcm_v01.geometry = parse_geometry(tree_geometry.get());
    }


    return tcm;
  }


  int CarmaCloudClient::StartWebService()
  {
    //Web services 
    char *placeholderX[1]={0};
    int placeholderC=1;
    QCoreApplication a(placeholderC,placeholderX);
    
    QHostAddress address = QHostAddress(QString::fromStdString (config_.webip));
    quint16 port = static_cast<quint16>(config_.webport);
    
    QSharedPointer<OpenAPI::OAIApiRequestHandler> handler(new OpenAPI::OAIApiRequestHandler());
    handler = QSharedPointer<OpenAPI::OAIApiRequestHandler> (new OpenAPI::OAIApiRequestHandler());
    auto router = QSharedPointer<OpenAPI::OAIApiRouter>::create();
    router->setUpRoutes();

    QObject::connect(handler.data(), &OpenAPI::OAIApiRequestHandler::requestReceived, [&](QHttpEngine::Socket *socket) {
		  TCMHandler(socket);
    });

    QObject::connect(handler.data(), &OpenAPI::OAIApiRequestHandler::requestReceived, [&](QHttpEngine::Socket *socket) {
		  router->processRequest(socket);
    });
    QHttpEngine::Server server(handler.data());

    if (!server.listen(address, port)) {
        qCritical("Unable to listen on the specified port.");
        return 1;
    }

    RCLCPP_DEBUG_STREAM(this->get_logger(), "CarmaCloudClient :: Started web service");
    return a.exec();

  }

  j2735_v2x_msgs::msg::TrafficControlPackage CarmaCloudClient::parse_package(boost::property_tree::ptree& tree)
  {
    j2735_v2x_msgs::msg::TrafficControlPackage tcm_pkg;

    auto tree_label = tree.get_child_optional("label");
    if( !tree_label )
    {
      tcm_pkg.label_exists = false;
    }
    else
    {
      tcm_pkg.label_exists = true;
      tcm_pkg.label = tree.get<std::string>("label");
    }

    std::string tcids_string = tree.get<std::string>("tcids");
    
    return tcm_pkg;

  }

  j2735_v2x_msgs::msg::TrafficControlSchedule CarmaCloudClient::parse_schedule(boost::property_tree::ptree& tree)
  {
    j2735_v2x_msgs::msg::TrafficControlSchedule tcm_schedule;
    tcm_schedule.start = tree.get<uint64_t>("schedule.start");

    auto child_end = tree.get_child_optional("schedule.end");
    if (!child_end)
    {
      tcm_schedule.end_exists = false;
    }
    else
    {
      tcm_schedule.end_exists = true;
      tcm_schedule.end = tree.get<uint64_t>("schedule.end");  
    }

    auto child_dow = tree.get_child_optional("schedule.dow");
    if (!child_dow)
    {
      tcm_schedule.dow_exists = false;
    }
    else
    {
      tcm_schedule.dow_exists = true;
      std::string dow_string = tree.get<std::string>("schedule.dow");
      for (size_t i=0; i<dow_string.size(); i++)
      {
        tcm_schedule.dow.dow[i] = dow_string[i] - '0';
      }
    }
    auto child_between = tree.get_child_optional("schedule.between");
    if( !child_between )
    {
      tcm_schedule.between_exists = false;
    }
    else
    {
      tcm_schedule.between_exists = true;
      auto child_tree = child_between.get();
      
      for (auto& item : tree.get_child("schedule.between"))
      {
        j2735_v2x_msgs::msg::DailySchedule daily;
        for (auto& which : item.second)
        {
          if (which.first == "begin")
          {
            daily.begin = which.second.get_value<uint16_t>();
          }
          else if (which.first == "duration")
          {
            daily.duration = which.second.get_value<uint16_t>();
          }
        }
        tcm_schedule.between.push_back(daily);
      }
    }
    auto child_repeat = tree.get_child_optional("schedule.repeat");

    if(!child_repeat)
    {
      tcm_schedule.repeat_exists = false;
    }
    else
    {
      tcm_schedule.repeat_exists = true;
      tcm_schedule.repeat.offset = child_repeat.get().get<uint16_t>("offset");
      tcm_schedule.repeat.period = child_repeat.get().get<uint16_t>("period");
      tcm_schedule.repeat.span = child_repeat.get().get<uint16_t>("span");
    }

    return tcm_schedule;
  }

  j2735_v2x_msgs::msg::TrafficControlDetail CarmaCloudClient::parse_detail(boost::property_tree::ptree& tree)
  {
    j2735_v2x_msgs::msg::TrafficControlDetail tcm_detail;

    auto child_closed = tree.get_child_optional( "detail.closed" );
    if( child_closed )
    {
      tcm_detail.choice = j2735_v2x_msgs::msg::TrafficControlDetail::CLOSED_CHOICE;
      std::string closed_val = tree.get<std::string>("detail.closed");
      if (closed_val == "open")
      {
        tcm_detail.closed = j2735_v2x_msgs::msg::TrafficControlDetail::OPEN;
      }
      else if (closed_val == "closed")
      {
        tcm_detail.closed = j2735_v2x_msgs::msg::TrafficControlDetail::CLOSED;
      }
      else if (closed_val == "taperleft")
      {
        tcm_detail.closed = j2735_v2x_msgs::msg::TrafficControlDetail::TAPERLEFT;
      }
      else if (closed_val == "taperright")
      {
        tcm_detail.closed = j2735_v2x_msgs::msg::TrafficControlDetail::TAPERRIGHT;
      }
      else if (closed_val == "openleft")
      {
        tcm_detail.closed = j2735_v2x_msgs::msg::TrafficControlDetail::OPENLEFT;
      }
      else if (closed_val == "openright")
      {
        tcm_detail.closed = j2735_v2x_msgs::msg::TrafficControlDetail::OPENRIGHT;
      }
      else tcm_detail.closed = j2735_v2x_msgs::msg::TrafficControlDetail::CLOSED;
      
    }

    auto child_chains = tree.get_child_optional( "detail.chains" );
    if( child_chains )
    {
      tcm_detail.chains = j2735_v2x_msgs::msg::TrafficControlDetail::CHAINS_CHOICE;
      std::string chains_val = tree.get<std::string>("detail.chains");
      if (chains_val == "no")
      {
        tcm_detail.chains = j2735_v2x_msgs::msg::TrafficControlDetail::NO;
      }
      else if (chains_val == "permitted")
      {
        tcm_detail.chains = j2735_v2x_msgs::msg::TrafficControlDetail::PERMITTED;
      }
      else if (chains_val == "required")
      {
        tcm_detail.chains = j2735_v2x_msgs::msg::TrafficControlDetail::REQUIRED;
      }
    }

    auto child_direction = tree.get_child_optional( "detail.chains" );
    if( child_direction )
    {
      tcm_detail.direction = j2735_v2x_msgs::msg::TrafficControlDetail::DIRECTION_CHOICE;
      std::string direction_val = tree.get<std::string>("detail.chains");
      if( child_direction )
      {
        tcm_detail.direction = j2735_v2x_msgs::msg::TrafficControlDetail::DIRECTION_CHOICE;
        std::string direction_val = tree.get<std::string>("detail.chains");
        if (direction_val == "forward")
        {
          tcm_detail.direction = j2735_v2x_msgs::msg::TrafficControlDetail::FORWARD;
        }
        else if (direction_val == "reverse")
        {
          tcm_detail.direction = j2735_v2x_msgs::msg::TrafficControlDetail::REVERSE;
        }
      }
    }


    auto child_mins = tree.get_child_optional( "detail.minspeed" );
    if( child_mins )
    {
      tcm_detail.choice = j2735_v2x_msgs::msg::TrafficControlDetail::MINSPEED_CHOICE;
      tcm_detail.minspeed = tree.get<float>("detail.minspeed");
    }

    auto child_maxs = tree.get_child_optional("detail.maxspeed");
    if( child_maxs )
    {
      tcm_detail.choice = j2735_v2x_msgs::msg::TrafficControlDetail::MAXSPEED_CHOICE;
      tcm_detail.maxspeed = tree.get<uint16_t>("detail.maxspeed");
    }

    auto child_minhdwy = tree.get_child_optional("detail.minhdwy");
    if( child_minhdwy )
    {
      tcm_detail.choice = j2735_v2x_msgs::msg::TrafficControlDetail::MINHDWY_CHOICE;
      tcm_detail.minhdwy = tree.get<uint16_t>("detail.minhdwy");
    }

    auto child_maxvehmass = tree.get_child_optional("detail.maxvehmass");
    if( child_maxvehmass )
    {
      tcm_detail.choice = j2735_v2x_msgs::msg::TrafficControlDetail::MAXVEHMASS_CHOICE;
      tcm_detail.maxvehmass = tree.get<uint16_t>("detail.maxvehmass");
    }

    auto child_maxvehheight = tree.get_child_optional("detail.maxvehheight");
    if( child_maxvehheight )
    {
      tcm_detail.choice = j2735_v2x_msgs::msg::TrafficControlDetail::MAXVEHHEIGHT_CHOICE;
      tcm_detail.maxvehheight = tree.get<uint8_t>("detail.maxvehheight");
    }

    auto child_maxvehwidth = tree.get_child_optional("detail.maxvehwidth");
    if( child_maxvehwidth )
    {
      tcm_detail.choice = j2735_v2x_msgs::msg::TrafficControlDetail::MAXVEHWIDTH_CHOICE;
      tcm_detail.maxvehwidth = tree.get<uint8_t>("detail.maxvehwidth");
    }

    auto child_maxvehlength = tree.get_child_optional("detail.maxvehlength");
    if( child_maxvehlength )
    {
      tcm_detail.choice = j2735_v2x_msgs::msg::TrafficControlDetail::MAXVEHLENGTH_CHOICE;
      tcm_detail.maxvehlength = tree.get<uint16_t>("detail.maxvehlength");
    }

    auto child_maxvehaxles = tree.get_child_optional("detail.maxvehaxles");
    if( child_maxvehaxles)
    {
      tcm_detail.choice = j2735_v2x_msgs::msg::TrafficControlDetail::MAXVEHAXLES_CHOICE;
      tcm_detail.maxvehaxles = tree.get<uint8_t>("detail.maxvehaxles");
    }

    auto child_minvehocc = tree.get_child_optional("detail.minvehocc");
    if( child_minvehocc)
    {
      tcm_detail.choice = j2735_v2x_msgs::msg::TrafficControlDetail::MINVEHOCC_CHOICE;
      tcm_detail.minvehocc = tree.get<uint8_t>("detail.minvehocc");
    }

    auto child_maxplatoonsize = tree.get_child_optional("detail.maxplatoonsize");
    if( child_maxplatoonsize)
    {
      tcm_detail.choice = j2735_v2x_msgs::msg::TrafficControlDetail::MAXPLATOONSIZE_CHOICE;
      tcm_detail.maxplatoonsize = tree.get<uint8_t>("detail.maxplatoonsize");
    }

    auto child_minplatoonhdwy = tree.get_child_optional("detail.minplatoonhdwy");
    if( child_minplatoonhdwy)
    {
      tcm_detail.choice = j2735_v2x_msgs::msg::TrafficControlDetail::MINPLATOONHDWY_CHOICE;
      tcm_detail.minplatoonhdwy = tree.get<uint16_t>("detail.minplatoonhdwy");
    }

    return tcm_detail;
  }

  j2735_v2x_msgs::msg::TrafficControlParams CarmaCloudClient::parse_params(boost::property_tree::ptree& tree)
  {
    j2735_v2x_msgs::msg::TrafficControlParams tcm_params;
    for (auto& item : tree.get_child("vclasses"))
    {
      j2735_v2x_msgs::msg::TrafficControlVehClass vclass;
      if (item.first == "any")
      {
        vclass.vehicle_class = j2735_v2x_msgs::msg::TrafficControlVehClass::ANY;
      }
      else if (item.first == "pedestrian")
      {
        vclass.vehicle_class = j2735_v2x_msgs::msg::TrafficControlVehClass::PEDESTRIAN;
      }
      else if (item.first == "bicycle")
      {
        vclass.vehicle_class = j2735_v2x_msgs::msg::TrafficControlVehClass::BICYCLE;
      }
      else if (tree.count("micromobile") != 0)
      {
        vclass.vehicle_class = j2735_v2x_msgs::msg::TrafficControlVehClass::MICROMOBILE;
      }
      else if (item.first == "motorcycle")
      {
        vclass.vehicle_class = j2735_v2x_msgs::msg::TrafficControlVehClass::MOTORCYCLE;
      }
      else if (item.first == "passenger-car")
      {
        vclass.vehicle_class = j2735_v2x_msgs::msg::TrafficControlVehClass::PASSENGER_CAR;
      }
      else if (item.first == "light-truck-van")
      {
        vclass.vehicle_class = j2735_v2x_msgs::msg::TrafficControlVehClass::LIGHT_TRUCK_VAN;
      }
      else if (item.first == "bus")
      {
        vclass.vehicle_class = j2735_v2x_msgs::msg::TrafficControlVehClass::BUS;
      }
      else if (item.first == "two-axle-six-tire-single-unit-truck")
      {
        vclass.vehicle_class = j2735_v2x_msgs::msg::TrafficControlVehClass::TWO_AXLE_SIX_TIRE_SINGLE_UNIT_TRUCK;
      }
      else if (item.first == "three-axle-single-unit-truck")
      {
        vclass.vehicle_class = j2735_v2x_msgs::msg::TrafficControlVehClass::THREE_AXLE_SINGLE_UNIT_TRUCK;
      }
      else if (item.first == "four-or-more-axle-single-unit-truck")
      {
        vclass.vehicle_class = j2735_v2x_msgs::msg::TrafficControlVehClass::FOUR_OR_MORE_AXLE_SINGLE_UNIT_TRUCK;
      }
      else if (item.first == "four-or-fewer-axle-single-trailer-truck")
      {
        vclass.vehicle_class = j2735_v2x_msgs::msg::TrafficControlVehClass::FOUR_OR_FEWER_AXLE_SINGLE_TRAILER_TRUCK;
      }
      else if (item.first == "five-axle-single-trailer-truck")
      {
        vclass.vehicle_class = j2735_v2x_msgs::msg::TrafficControlVehClass::FIVE_AXLE_SINGLE_TRAILER_TRUCK;
      }
      else if (item.first == "six-or-more-axle-single-trailer-truck")
      {
        vclass.vehicle_class = j2735_v2x_msgs::msg::TrafficControlVehClass::SIX_OR_MORE_AXLE_SINGLE_TRAILER_TRUCK;
      }
      else if (item.first == "five-or-fewer-axle-multi-trailer-truck")
      {
        vclass.vehicle_class = j2735_v2x_msgs::msg::TrafficControlVehClass::FIVE_OR_FEWER_AXLE_MULTI_TRAILER_TRUCK;
      }
      else if (item.first =="six-axle-multi-trailer-truck")
      {
        vclass.vehicle_class = j2735_v2x_msgs::msg::TrafficControlVehClass::SIX_AXLE_MULTI_TRAILER_TRUCK;
      }
      else if (item.first =="seven-or-more-axle-multi-trailer-truck")
      {
        vclass.vehicle_class = j2735_v2x_msgs::msg::TrafficControlVehClass::SEVEN_OR_MORE_AXLE_MULTI_TRAILER_TRUCK;
      }
      else if (item.first =="rail")
      {
        vclass.vehicle_class = j2735_v2x_msgs::msg::TrafficControlVehClass::RAIL;
      }
      else if (item.first == "unclassified")
      {
        vclass.vehicle_class = j2735_v2x_msgs::msg::TrafficControlVehClass::UNCLASSIFIED;
      }
      tcm_params.vclasses.push_back(vclass);
    }
    tcm_params.schedule = parse_schedule(tree);
    
    std::string bool_str = tree.get_value<std::string>("regulatory");
    if (bool_str == "false") tcm_params.regulatory = false;
    else tcm_params.regulatory = true;
     
    tcm_params.detail = parse_detail(tree);

    return tcm_params;

  }
    
  j2735_v2x_msgs::msg::TrafficControlGeometry CarmaCloudClient::parse_geometry(boost::property_tree::ptree& tree)
  {
    j2735_v2x_msgs::msg::TrafficControlGeometry tcm_geometry;
    tcm_geometry.proj = tree.get<std::string>("proj");
    tcm_geometry.datum = tree.get<std::string>("datum");

    tcm_geometry.reftime = tree.get<uint64_t>("reftime");

    tcm_geometry.reflon = tree.get<int32_t>("reflon");
    tcm_geometry.reflat = tree.get<int32_t>("reflat");
    tcm_geometry.refelv = tree.get<int32_t>("refelv");
    tcm_geometry.heading = tree.get<int16_t>("heading");
    
    for (auto& item : tree.get_child("nodes"))
    {
      j2735_v2x_msgs::msg::PathNode pathnode;
      for (auto& which : item.second)
      {
        if (which.first == "x")
        {
          std::string x_val = which.second.get_value<std::string>();
          pathnode.x = std::stoll(x_val);
        }
        
        else if (which.first == "y")
        {
          std::string y_val = which.second.get_value<std::string>();
          pathnode.y = std::stoll(y_val);
        }
        else if (which.first == "z")
        {
          pathnode.z_exists = true;
          std::string z_val = which.second.get_value<std::string>();
          pathnode.z = std::stoll(z_val);
        }
        else if (which.first == "width")
        {
          pathnode.width_exists = true;
          std::string w_val = which.second.get_value<std::string>();
          pathnode.width = std::stoll(w_val);
          
        }
        
      }
      tcm_geometry.nodes.push_back(pathnode);
    }
    
    return tcm_geometry;

  }

  unsigned char CarmaCloudClient::parse_hex(char c)
  {
    if ('0' <= c && c <= '9') return c - '0';
    if ('A' <= c && c <= 'F') return c - 'A' + 10;
    if ('a' <= c && c <= 'f') return c - 'a' + 10;
    std::abort();
  }



} // carma_cloud_client

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(carma_cloud_client::CarmaCloudClient)
