#pragma once
/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Torc Robotics, LLC
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Torc Robotics, LLC nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <openjaus/openJaus.h>

#include <string>

namespace cav
{
class NodeManager
{
public:

    static NodeManager& instance()
    {
        static NodeManager instance;
        return instance;
    }


    bool init(const std::string& file_name)
    {
        handler_.reset(new MyHandler());
        std::unique_ptr<FileLoader> configData(new FileLoader());
        if(!configData->load_cfg(file_name))
        {
            printf("Unable to open file: %s\n", file_name.c_str());
            return false;
        }

        try
        {
            nm_.reset(new ::NodeManager(configData.get(),handler_.get()));
        }
        catch(char *exceptionString)
        {
            printf("%s\n", exceptionString);
            return false;
        }
        catch(...)
        {
            printf("Node Manager Construction Failed.\n");
            return false;
        }

        return true;
    }

private:

    class MyHandler;
    std::unique_ptr<::NodeManager> nm_;
    std::unique_ptr<MyHandler> handler_;
    NodeManager()
    {

    }


    class MyHandler : public EventHandler
    {
    public:
        ~MyHandler()
        {

        }
        
        void handleEvent(NodeManagerEvent *e)
        {
            SystemTreeEvent *treeEvent;
            ErrorEvent *errorEvent;
            JausMessageEvent *messageEvent;
            DebugEvent *debugEvent;
            ConfigurationEvent *configEvent;

            switch(e->getType())
            {
                case NodeManagerEvent::SystemTreeEvent:
                    treeEvent = (SystemTreeEvent *)e;
                    printf("%s\n", treeEvent->toString().c_str());
                    delete e;
                    break;

                case NodeManagerEvent::ErrorEvent:
                    errorEvent = (ErrorEvent *)e;
                    printf("%s\n", errorEvent->toString().c_str());
                    delete e;
                    break;

                case NodeManagerEvent::JausMessageEvent:
                    messageEvent = (JausMessageEvent *)e;
                    // If you turn this on, the system gets spam-y this is very useful for debug purposes
                    if(messageEvent->getJausMessage()->commandCode != JAUS_REPORT_HEARTBEAT_PULSE)
                    {
                        //printf("%s\n", messageEvent->toString().c_str());
                    }
                    else
                    {
                        //printf("%s\n", messageEvent->toString().c_str());
                    }
                    delete e;
                    break;

                case NodeManagerEvent::DebugEvent:
                    debugEvent = (DebugEvent *)e;
                    //printf("%s\n", debugEvent->toString().c_str());
                    delete e;
                    break;

                case NodeManagerEvent::ConfigurationEvent:
                    configEvent = (ConfigurationEvent *)e;
                    printf("%s\n", configEvent->toString().c_str());
                    delete e;
                    break;

                default:
                    delete e;
                    break;
            }
        }
    };


};
};