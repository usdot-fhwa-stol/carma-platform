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

package gov.dot.fhwa.saxton.carma.template;

import gov.dot.fhwa.saxton.carma.rosutils.SaxtonBaseNode;
import gov.dot.fhwa.saxton.carma.rosutils.RosServiceSynchronizer;
import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceServer;
import org.ros.node.service.ServiceResponseBuilder;
import org.ros.node.service.ServiceResponseListener;
import org.ros.exception.RemoteException;
import org.ros.exception.RosRuntimeException;

/*
 * Enter Class Description here.
 *
 * @author authorname@google.com (Damon Kohler)
 *
 * Replace ServerClient with the node name on Column D but using CamelCase.
 *
 * Command line test: rosrun carma template gov.dot.fhwa.saxton.carma.template.ServerClient
*/

public class ServerClient extends SaxtonBaseNode {

  //TODO: Replace "server_client" with Column D node name
  @Override
  public GraphName getDefaultNodeName() {
    return GraphName.of("server_client");
  }

  @Override
  public void onSaxtonStart(final ConnectedNode connectedNode) {

    // Services (SetBool Example)

    // Setup Server
    // TODO: Replace set_bool with Column G name
    ServiceServer<std_srvs.SetBoolRequest, std_srvs.SetBoolResponse> setBoolServer = connectedNode.newServiceServer(
        "set_bool", std_srvs.SetBool._TYPE,
        new ServiceResponseBuilder<std_srvs.SetBoolRequest, std_srvs.SetBoolResponse>() {

          @Override
          public void build(std_srvs.SetBoolRequest request, std_srvs.SetBoolResponse response) {

            //TODO: set this call based on your service functionality
            response.setSuccess(request.getData());
            response.setMessage("Successfully called set_bool.");
          }
        });

    // Setup Client
    final ServiceClient<std_srvs.SetBoolRequest, std_srvs.SetBoolResponse> serviceClient = this
        .waitForService("set_bool", std_srvs.SetBool._TYPE, connectedNode, 5000);

    if (serviceClient == null) {
      connectedNode.getLog().error("ServerClient Node could not find service set_bool");
    }

    // Example cav_srvs
    final ServiceClient<cav_srvs.PluginListRequest, cav_srvs.PluginListResponse> pluginsClient;

    // This CancellableLoop will be canceled automatically when the node shuts down.
    connectedNode.executeCancellableLoop(new CancellableLoop() {
      private int sequenceNumber;

      @Override
      protected void setup() {
        sequenceNumber = 0;
      }

      @Override
      protected void loop() throws InterruptedException {

        //Setup Request
        final std_srvs.SetBoolRequest request = serviceClient.newMessage();
        request.setData(true);

        //Make Service call.
        RosServiceSynchronizer.callSync(serviceClient, request,
            new ServiceResponseListener<std_srvs.SetBoolResponse>() {
              @Override
              public void onSuccess(std_srvs.SetBoolResponse response) {
                connectedNode.getLog().info(response.getMessage());
              }

              @Override
              public void onFailure(RemoteException e) {
                throw new RosRuntimeException(e);
              }
            });

        Thread.sleep(30000);
      }
    });
  }

  @Override
  protected void handleException(Throwable e) {

  }
}
