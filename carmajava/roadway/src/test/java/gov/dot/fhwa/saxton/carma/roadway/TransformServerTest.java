package gov.dot.fhwa.saxton.carma.roadway;

import cav_srvs.GetTransformRequest;
import cav_srvs.GetTransformResponse;
import geometry_msgs.TransformStamped;
import org.junit.Test;
import org.ros.concurrent.CancellableLoop;
import org.ros.exception.DuplicateServiceException;
import org.ros.exception.RemoteException;
import org.ros.exception.RosRuntimeException;
import org.ros.exception.ServiceNotFoundException;
import org.ros.message.MessageFactory;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeConfiguration;
import org.ros.node.service.*;
import org.ros.node.topic.Publisher;
import ros.RosTest;
import rosjava_test_msgs.AddTwoIntsRequest;
import rosjava_test_msgs.AddTwoIntsResponse;
import std_srvs.SetBoolResponse;
import tf2_msgs.TFMessage;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.concurrent.TimeUnit;

import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/**
 * Created by mcconnelms on 9/1/17.
 */
public class TransformServerTest extends RosTest {

  @Test
  public void testServiceAvailability() throws Exception {
    final CountDownServiceServerListener<cav_srvs.GetTransformRequest, cav_srvs.GetTransformResponse>
      countDownServiceServerListener = CountDownServiceServerListener.newDefault();

    final NodeConfiguration nodeConfiguration = NodeConfiguration.newPrivate();
    final MessageFactory messageFactory = nodeConfiguration.getTopicMessageFactory();
    
    nodeMainExecutor.execute(new TransformServer(), nodeConfiguration);
    //TODO confirm the TransformServer node was started?
    nodeMainExecutor.execute(new AbstractNodeMain() {
      @Override public GraphName getDefaultNodeName() {
        return GraphName.of("transform_server_tester");
      }

      @Override public void onStart(final ConnectedNode connectedNode) {

        final Publisher<tf2_msgs.TFMessage> tfPub = connectedNode.newPublisher("/tf", TFMessage._TYPE);
        TFMessage tfMsg = tfPub.newMessage();
        TransformStamped tfStamped = messageFactory.newFromType(TransformStamped._TYPE);
        tfStamped.getHeader().setSeq(0);
        tfStamped.getHeader().setFrameId("parent_frame");
        tfStamped.getHeader().setStamp(connectedNode.getCurrentTime());
        tfStamped.setChildFrameId("child_frame");
        tfMsg.setTransforms(new ArrayList<>(Arrays.asList(tfStamped)));

        tfPub.publish(tfMsg);

        // TODO is it possible to check ros master for message registration.
        try {
          Thread.sleep(1000);
        } catch (InterruptedException e) {
          e.printStackTrace();
        }

        // TODO assert Service is Available (Is the try catch sufficient)
        ServiceClient<GetTransformRequest, GetTransformResponse> serviceClient = null;
        try {
          serviceClient = connectedNode.newServiceClient("get_transform", cav_srvs.GetTransform._TYPE);
        } catch (ServiceNotFoundException e1) {
          e1.printStackTrace();
        }

        //Setup Request
        final cav_srvs.GetTransformRequest request = serviceClient.newMessage();
        request.setSourceFrame("parent_frame");
        request.setTargetFrame("child_frame");

        serviceClient.call(request, new ServiceResponseListener<GetTransformResponse>() {
          @Override public void onSuccess(GetTransformResponse response) {
            response.getTransform(); //TODO assert the transforms are equivalent
          }

          @Override public void onFailure(RemoteException e) {
            throw new RosRuntimeException(e);
          }
        });
      }
    }, nodeConfiguration);

    assertTrue(countDownServiceServerListener.awaitMasterRegistrationSuccess(1, TimeUnit.SECONDS));
  }
}
