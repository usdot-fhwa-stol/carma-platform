/*
 * Copyright (C) 2019 LEIDOS.
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

#ifndef __CAPABILITIES_INTERFACE_TPP__
#define __CAPABILITIES_INTERFACE_TPP__

#include <vector>
#include <map>
#include <string>
#include <functional>

namespace arbitrator 
{
            template<typename MReq, typename MRes>
            std::map<std::string, MRes> CapabiltiesInterface::multiplex_service_call_for_capability(std::string query_string, MReq msg) 
            {
                std::vector<std::string> topics = get_topics_for_capability(query_string);
                MRes response;
                std::map<std::string, MRes> responses;
                for (auto i = topics.begin(); i != topics.end(); i++) 
                {
                    service_clients_[*i].call(msg, &MRes);
                    responses.emplace(*i, response);
                }

                return responses;
            }

            template<typename Mmsg>
            void CapabiltiesInterface::multiplex_publication_for_capability(std::string query_string, Mmsg msg) 
            {
                std::vector<std::string> topics = get_topics_for_capability(query_string);
                for (auto i = topics.begin(); i != topics.end(); i++) 
                {
                    publishers_[*i].publish(msg);
                }
            }
};

#endif
