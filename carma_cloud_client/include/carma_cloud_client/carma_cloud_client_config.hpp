#pragma once

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

#include <iostream>
#include <vector>

namespace carma_cloud_client
{

  /**
   * \brief Stuct containing the algorithm configuration values for carma_cloud_client
   */
  struct Config
  {
    //host machine IP
    std::string url ="http://127.0.0.1:"; 
    std::string base_hb = "/carmacloud/v2xhub";
    //URL for traffic control request
    std::string base_req = "/carmacloud/tcmreq";
    //URL for traffic control acknowlegement.
    std::string base_ack = "/carmacloud/tcmack";
    std::string method = "POST";
    // 33333 is the port that will send from v2xhub to carma cloud ## initally was 23665
    std::string port = "33333";
    // list determines how the TCMs will be sent in response. "true" means a list of TCMs are sent. "false" means one at a time
    std::string list = "true";
    // number of days back when a geofence is valid
    int fetchtime = 15;

    // Stream operator for this config
    friend std::ostream &operator<<(std::ostream &output, const Config &c)
    {
      output << "carma_cloud_client::Config { " << std::endl
           << "url: " << c.url << std::endl
           << "base_hb: " << c.base_hb << std::endl
           << "base_req: " << c.base_req << std::endl
           << "base_ack: " << c.base_ack << std::endl
           << "port: " << c.port << std::endl
           << "list: " << c.list << std::endl
           << "fetchtime: " << c.fetchtime << std::endl
           << "method: " << c.method << std::endl
           << "}" << std::endl;
      return output;
    }
  };

} // carma_cloud_client