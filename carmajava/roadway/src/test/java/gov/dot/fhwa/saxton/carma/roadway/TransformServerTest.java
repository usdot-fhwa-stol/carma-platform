package gov.dot.fhwa.saxton.carma.roadway;

import cav_srvs.GetTransform;
import cav_srvs.GetTransformRequest;
import cav_srvs.GetTransformResponse;
import geometry_msgs.TransformStamped;
import gov.dot.fhwa.saxton.carma.rosutils.SaxtonBaseNode;
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

import static org.junit.Assert.assertNotNull;
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
    nodeMainExecutor.execute(new SaxtonBaseNode() {
      @Override public GraphName getDefaultNodeName() {
        return GraphName.of("transform_server_tester");
      }

      @Override public void onStart(final ConnectedNode connectedNode) {
        final String SERVICE_NAME = "get_transform";

        // Assert that the service was created
        ServiceClient<GetTransformRequest, GetTransformResponse> serviceClient = this.waitForService(SERVICE_NAME, GetTransform._TYPE, connectedNode, 1000);
        assertNotNull("Service Client Not Null assertion", serviceClient);

        final Publisher<tf2_msgs.TFMessage> tfPub = connectedNode.newPublisher("/tf", TFMessage._TYPE);
        TFMessage tfMsg = tfPub.newMessage();
        final TransformStamped tfStamped = messageFactory.newFromType(TransformStamped._TYPE);
        tfStamped.getHeader().setSeq(0);
        tfStamped.getHeader().setFrameId("parent_frame");
        tfStamped.getHeader().setStamp(connectedNode.getCurrentTime());
        tfStamped.setChildFrameId("child_frame");
        tfMsg.setTransforms(new ArrayList<>(Arrays.asList(tfStamped)));

        tfPub.publish(tfMsg);

        // TODO is it possible to check ros master for message registration.
        // TODO Might be able to use a subscriber listener here. See TopicIntegrationTest in rosjava src/test
        org.ros.node.topic.CountDownSubscriberListener<tf2_msgs.TFMessage> subscriberListener = org.ros.node.topic.CountDownSubscriberListener.newDefault();
        try {
          // TODO research this more
          subscriberListener.awaitMasterRegistrationSuccess(1, TimeUnit.SECONDS);
        } catch (InterruptedException e) {
          e.printStackTrace();
        }

        //Request Non-Inverse transform. Assert false on failure or incorrect transform returned
        final cav_srvs.GetTransformRequest request = serviceClient.newMessage();
        request.setSourceFrame("parent_frame");
        request.setTargetFrame("child_frame");

        serviceClient.call(request, new ServiceResponseListener<GetTransformResponse>() {
          @Override public void onSuccess(GetTransformResponse response) {
            // Compare the requested transform with the
            assertTrue(isEqualTransform(tfStamped, response.getTransform()));
          }

          @Override public void onFailure(RemoteException e) {
            fail("Failed to call get_transform service");
          }
        });

        //Request Inverse transform. Assert false on failure or incorrect transform returned
        final cav_srvs.GetTransformRequest inverseRequest = serviceClient.newMessage();
        inverseRequest.setSourceFrame("child_frame");
        inverseRequest.setTargetFrame("parent_frame");

        serviceClient.call(inverseRequest, new ServiceResponseListener<GetTransformResponse>() {
          @Override public void onSuccess(GetTransformResponse response) {
            // Compare the requested transform with the
            assertTrue(isEqualTransform(tfStamped, response.getTransform()));
          }

          @Override public void onFailure(RemoteException e) {
            fail("Failed to call get_transform service");
          }
        });
      }
    }, nodeConfiguration);

    assertTrue(countDownServiceServerListener.awaitMasterRegistrationSuccess(1, TimeUnit.SECONDS));
  }

  // Helper function compares two TransformStamped messages for equality
  private boolean isEqualTransform(TransformStamped t1, TransformStamped t2) {
    return
      t1.getHeader().getFrameId().equals(t2.getHeader().getFrameId()) &&
      t1.getChildFrameId().equals(t2.getChildFrameId()) &&
      dEqual(t1.getTransform().getRotation().getW(), t2.getTransform().getRotation().getW(), 0.0000001) &&
      dEqual(t1.getTransform().getRotation().getW(), t2.getTransform().getRotation().getX(), 0.0000001) &&
      dEqual(t1.getTransform().getRotation().getW(), t2.getTransform().getRotation().getY(), 0.0000001) &&
      dEqual(t1.getTransform().getRotation().getW(), t2.getTransform().getRotation().getZ(), 0.0000001) &&
      dEqual(t1.getTransform().getTranslation().getX(), t2.getTransform().getTranslation().getX(), 0.0000001) &&
      dEqual(t1.getTransform().getTranslation().getY(), t2.getTransform().getTranslation().getY(), 0.0000001) &&
      dEqual(t1.getTransform().getTranslation().getZ(), t2.getTransform().getTranslation().getZ(), 0.0000001);
  }

  // Helper function compares two doubles for simple equality
  private boolean dEqual(double d1, double d2, double MAX_DIFF) {
    return Math.abs(d2-d1) < MAX_DIFF;
  }
}
