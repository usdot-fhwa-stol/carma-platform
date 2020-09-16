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

package gov.dot.fhwa.saxton.carma.mock_drivers;

import cav_msgs.DriverStatus;
import cav_srvs.*;
import org.apache.commons.logging.Log;
import org.ros.message.MessageFactory;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeConfiguration;
import org.ros.node.parameter.ParameterTree;
import org.ros.node.service.ServiceResponseBuilder;
import org.ros.node.service.ServiceServer;
import org.ros.node.topic.Publisher;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.RandomAccessFile;
import java.util.LinkedList;
import java.util.List;

/**
 * Abstract implementation of a simulated driver. Reads a simulated data file and publishes the data.
 */
public abstract class AbstractMockDriver implements IMockDriver {
  protected final ConnectedNode connectedNode;
  protected final NodeConfiguration nodeConfiguration = NodeConfiguration.newPrivate();
  protected final MessageFactory messageFactory = nodeConfiguration.getTopicMessageFactory();

  protected final Log log;
  protected final ParameterTree params;
  protected final GraphName graphName;

  // Parameters
  protected final String rosRunID;
  protected final String dataFilePath;

  // Topics
  // Published
  //protected final Publisher<bond.Status> bondPub;
  protected final Publisher<cav_msgs.DriverStatus> discoveryPub;

  // Server
  protected final ServiceServer<cav_srvs.BindRequest, cav_srvs.BindResponse> bindService;
  protected final ServiceServer<GetDriverApiRequest, GetDriverApiResponse> getApiService;
  protected final ServiceServer<GetDriverStatusRequest, GetDriverStatusResponse> getStatusService;

  protected final String delimiter = ","; // Comma for csv file
  protected RandomAccessFile reader = null;
  protected byte driverStatus = cav_msgs.DriverStatus.OFF;

  /**
   * Constructor establishes the publishers and subscribers for the ROS network.
   *
   * @param connectedNode the ros node which this driver provides implementations for
   */
  public AbstractMockDriver(ConnectedNode connectedNode) {
    this.connectedNode = connectedNode;
    log = connectedNode.getLog();
    params = connectedNode.getParameterTree();
    this.graphName = connectedNode.getName();

    // Parameters
    rosRunID = params.getString("/run_id");
    dataFilePath = params.getString("~/data_file_path");

    // Topics
    // Published
    //bondPub = connectedNode.newPublisher("~/bond", bond.Status._TYPE); //TODO add once bind cpp is wrapped in jni
    discoveryPub = connectedNode.newPublisher("driver_discovery", cav_msgs.DriverStatus._TYPE);

    // Service
    // Server
    bindService = connectedNode.newServiceServer("~/bind", cav_srvs.Bind._TYPE,
      new ServiceResponseBuilder<BindRequest, BindResponse>() {
        @Override public void build(cav_srvs.BindRequest request, cav_srvs.BindResponse response) {
          log.info("Request for bind received");
        }
      });
    getApiService = connectedNode
      .newServiceServer("~/get_driver_api", cav_srvs.GetDriverApi._TYPE,
        new ServiceResponseBuilder<cav_srvs.GetDriverApiRequest, cav_srvs.GetDriverApiResponse>() {
          @Override public void build(cav_srvs.GetDriverApiRequest request,
            cav_srvs.GetDriverApiResponse response) {
            List<String> FQNs = new LinkedList<>();
            for (String apiElement: getDriverAPI()) {
              FQNs.add("/" + apiElement);
            }
            response.setApiList(FQNs);
          }
        });
    getStatusService = connectedNode.newServiceServer("~/get_status", GetDriverStatus._TYPE,
      new ServiceResponseBuilder<GetDriverStatusRequest, GetDriverStatusResponse>() {
        @Override public void build(GetDriverStatusRequest request,
          GetDriverStatusResponse response) {
          response.setStatus(getDriverStatus());
        }
      });
  }

  /**
   * Function which should be called in the onStart function of a containing ROS Node
   * This implementation opens a data file to use for simulation
   * @param connectedNode The node which is being started
   */
  @Override public void onStart(ConnectedNode connectedNode) {
    try {
      reader = new RandomAccessFile(dataFilePath, "r");
      driverStatus = cav_msgs.DriverStatus.OPERATIONAL;
    } catch (FileNotFoundException e) {
      log.warn(getGraphName() + " could not find file " + dataFilePath + ".No data published " + e.getMessage());
      driverStatus = cav_msgs.DriverStatus.DEGRADED;
    }
  }

  @Override public void onInterruption() {
    // Close an opened data file
    closeDataFile();
  }

  @Override protected void finalize() throws Throwable{
    closeDataFile();
    super.finalize();
  }

  @Override public void readAndPublishData() {
    
    if (reader == null) {
      return;
    }
    try {
      List<String[]> data = new LinkedList<>();
      String dataLine;
      String[] elements;
      boolean exitBeforeEOF = false;
      int prevSampleIndex = -1;
      int currentSampleIndex;
      long prevLineIndex = reader.getFilePointer();

      while((dataLine = reader.readLine()) != null) {
        // Skip the header line of all data files
        if (prevLineIndex == 0) {
          prevLineIndex = reader.getFilePointer();
          continue;
        }
        // separate on delimiter
        elements = dataLine.split(delimiter);
        // Update sample index
        if (elements.length != getExpectedColCount()) {
          log.warn(
            "Publish data requested for " + getGraphName() + " with incorrect number of data elements. "
              + "The required number of data elements is " + getExpectedColCount());
          continue; // Skip this invalid line
        }

        currentSampleIndex = Integer.parseInt(elements[getSampleIdIdx()]);
        //If this is the first sample
        if (prevSampleIndex == -1) {
          prevSampleIndex = currentSampleIndex;
        }
        // If the end of this sample set then exit the loop
        if (currentSampleIndex != prevSampleIndex) {
          exitBeforeEOF = true;
          break;
        }
        data.add(elements);
      }
      if (!exitBeforeEOF) {
        reader.seek(0);
      }
      publishData(data);

    } catch (IOException e) {
      closeDataFile();
      reader = null;
      // Log warning if the node failed to read data in the file. All publishing will be stopped in this case as the file may be corrupt.
      log.warn(getGraphName() + " failed to read data file. No data will be published " + e.getMessage());
      driverStatus = cav_msgs.DriverStatus.FAULT;
    }
  }

  /**
   * Helper function to build a driver status message
   * @return The driver status message
   */
  protected DriverStatus getDriverStatus() {
    cav_msgs.DriverStatus driverStatusMsg = discoveryPub.newMessage();
    driverStatusMsg.setName(getGraphName().toString());
    driverStatusMsg.setStatus(driverStatus);
    driverStatusMsg.setCan(false);
    driverStatusMsg.setRadar(false);
    driverStatusMsg.setGnss(false);
    driverStatusMsg.setImu(false);
    driverStatusMsg.setLidar(false);
    driverStatusMsg.setRoadwaySensor(false);
    driverStatusMsg.setComms(false);
    driverStatusMsg.setController(false);
    driverStatusMsg.setCamera(false);
    driverStatusMsg.setLightbar(false);

    for (String driverType: getDriverTypesList()) {
      switch (driverType) {
        case "can":
          driverStatusMsg.setCan(true);
          break;
        case "radar":
          driverStatusMsg.setRadar(true);
          break;
        case "gnss":
          driverStatusMsg.setGnss(true);
          break;
        case "imu":
          driverStatusMsg.setImu(true);
          break;
        case "lidar":
          driverStatusMsg.setLidar(true);
          break;
        case "roadway_sensor":
          driverStatusMsg.setRoadwaySensor(true);
          break;
        case "comms":
          driverStatusMsg.setComms(true);
          break;
        case "controller":
          driverStatusMsg.setController(true);
          break;
        case "camera":
          driverStatusMsg.setCamera(true);
          break;
        case "lightbar":
          driverStatusMsg.setLightbar(true);
          break;
        default:
          log.warn("Function getDriverStatus received an unrecognized driver type: " + driverType);
          break;
      }
    }
    return driverStatusMsg;
  }

  @Override public void publishDriverStatus() {
    discoveryPub.publish(getDriverStatus());
  }

  /**
   * Safely closes the opened data file
   */
  protected void closeDataFile() {
    if (reader != null) {
      try {
        reader.close();
      } catch (IOException ex) {
    	  log.warn(getGraphName() + " failed to close data reader. " + ex.getMessage());
      }
    }
  }

  @Override public GraphName getGraphName() {
    return graphName;
  }

  /**
   * Publishes the provided data array
   * @param data The data to be published usually provided as a direct line from a data file
   */
  protected abstract void publishData(List<String[]> data);

  /**
   * Gets the expected number of row elements in a data line
   * @return The number of expected elements
   */
  protected abstract short getExpectedColCount();

  /**
   * Gets the column number for the sample id
   * @return The column number in the data file
   */
  protected abstract short getSampleIdIdx();

  /**
   * Gets a list of driver information which this driver can provide (can, comms, sensor, position, controller)
   * @return The list of driver types which this driver satisfies
   */
  protected abstract List<String> getDriverTypesList();

  @Override public abstract List<String> getDriverAPI();

  @Override public long getPublishDelay() {
    return 100;
  }
}
