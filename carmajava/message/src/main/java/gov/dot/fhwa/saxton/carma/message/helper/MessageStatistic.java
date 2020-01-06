/*
 * Copyright (C) 2018-2020 LEIDOS.
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

package gov.dot.fhwa.saxton.carma.message.helper;

import java.util.LinkedList;
import java.util.List;

import org.ros.node.ConnectedNode;

import gov.dot.fhwa.saxton.carma.rosutils.SaxtonLogger;

/**
 * This class is for recording the frequency of outgoing DSRC messages
 * and recording the number of incoming DSRC messages in a period of time.
 */
public class MessageStatistic {

    private class MessageCounter {
        String messageType;
        double last_outgoing_sample_time = 0.0, last_incoming_sample_time = 0.0;
        int outgoing_counter = 0, incoming_counter = 0;
        
        public MessageCounter(String messageType) {
            this.messageType = messageType;
        }
    }
    
    protected final double sample_window = 5.0;
    
    protected List<MessageCounter> counters_;
    protected ConnectedNode node_;
    protected SaxtonLogger log_;
    
    public MessageStatistic(ConnectedNode node, SaxtonLogger log) {
        this.counters_ = new LinkedList<MessageCounter>();
        this.node_ = node;
        this.log_ = log;
    }
    
    public void registerEntry(String messageType) {
        if(findMessageCounter(messageType) == null) {
            log_.info("MessageStatistic is register a new counter " + messageType);
            counters_.add(new MessageCounter(messageType));
        } else {
            log_.warn("MessageStatistic already have " + messageType + " couter");
        }
    }
    
    public void onMessageSending(String messageType) {
        MessageCounter counter = findMessageCounter(messageType);
        if(counter == null) {
            //TODO: comment this out since we do not need to record mobility message freq for now
            //log_.warn("Cannot find the right message counter based on given type.");
        } else {
            if(counter.last_outgoing_sample_time == 0) {
                counter.last_outgoing_sample_time = node_.getCurrentTime().toSeconds();
            }
            counter.outgoing_counter++;
            if(node_.getCurrentTime().toSeconds() - counter.last_outgoing_sample_time >= sample_window) {
                double freq = counter.outgoing_counter / sample_window;
                log_.info(messageType, String.format("Outgoing " + messageType + " is encoded and published in %.02f Hz", freq));
                counter.outgoing_counter = 0;
                counter.last_outgoing_sample_time = node_.getCurrentTime().toSeconds();
            }
        }
    }
    
    public void onMessageReceiving(String messageType) {
        MessageCounter counter = findMessageCounter(messageType);
        if(counter == null) {
            log_.warn("Cannot find the right message counter based on given type.");
        } else {
            if(counter.last_incoming_sample_time == 0) {
                counter.last_incoming_sample_time = node_.getCurrentTime().toSeconds();
            }
            counter.incoming_counter++;
            if(node_.getCurrentTime().toSeconds() - counter.last_incoming_sample_time >= sample_window) {
                log_.info(messageType, "There are " +  counter.incoming_counter + " incoming " + messageType + "s decoded in past " + sample_window + " seconds");
                counter.incoming_counter = 0;
                counter.last_incoming_sample_time = node_.getCurrentTime().toSeconds(); 
            }
        }
    }
    
    private MessageCounter findMessageCounter(String messageType) {
        for(MessageCounter counter : counters_) {
            if(counter.messageType.equals(messageType)) {
                return counter;
            }
        }
        return null;
    }
}
