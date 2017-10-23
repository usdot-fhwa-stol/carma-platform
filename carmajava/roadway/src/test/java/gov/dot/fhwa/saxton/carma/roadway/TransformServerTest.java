package gov.dot.fhwa.saxton.carma.roadway;

import cav_srvs.GetTransform;
import cav_srvs.GetTransformRequest;
import cav_srvs.GetTransformResponse;
import geometry_msgs.TransformStamped;
import gov.dot.fhwa.saxton.carma.rosutils.RosTest;
import gov.dot.fhwa.saxton.carma.rosutils.SaxtonBaseNode;
import gov.dot.fhwa.saxton.carma.rosutils.RosServiceSynchronizer;
import org.ros.exception.RemoteException;
import org.ros.message.Duration;
import org.ros.message.MessageFactory;
import org.ros.message.Time;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.service.*;
import org.ros.node.topic.Publisher;
import org.ros.rosjava_geometry.*;
import tf2_msgs.TFMessage;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;

import org.junit.Test;

import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

/**
 * Class for running integration and unit tests of the TransformServer node in the Roadway package
 */
public class TransformServerTest extends RosTest {

  /**
   * Test of the get_transform service in the TransformServer node
   * Checks if the TransformServer node is subscribed to /tf and provides the get_transform service
   * The accuracy of the service is then tested
   */
  @Test
  public void testServiceAvailability() throws Exception {
    final MessageFactory messageFactory = nodeConfiguration.getTopicMessageFactory();
    final CountDownLatch countDownLatch = new CountDownLatch(2);

    // Create the anonymous node to test the server
    SaxtonBaseNode anonNode = new SaxtonBaseNode() {
      @Override
      public GraphName getDefaultNodeName() {
        return GraphName.of("transform_server_tester");
      }

      @Override
      public void onSaxtonStart(final ConnectedNode connectedNode) {
        final String SERVICE_NAME = "get_transform";
        final int subscriberTimeout = 1000; //ms

        // Build the transforms to be tested
        final Vector3 translation = new Vector3(1, 1, 1);
        final Vector3 zAxis = new Vector3(0, 0, 1); // Vector for rotating around z axis
        final Quaternion rotZAxis = Quaternion.fromAxisAngle(zAxis, Math.PI / 4.0);
        final Transform transform = new Transform(translation, rotZAxis);
        final Transform invertedTransform = transform.invert();

        // Build ros messages
        geometry_msgs.Transform transformMsg = messageFactory.newFromType(geometry_msgs.Transform._TYPE);
        transformMsg = transform.toTransformMessage(transformMsg);
        geometry_msgs.Transform invertedTransformMsg = messageFactory.newFromType(geometry_msgs.Transform._TYPE);
        invertedTransformMsg = invertedTransform.toTransformMessage(invertedTransformMsg);

        // Assert that the service was created
        ServiceClient<GetTransformRequest, GetTransformResponse> serviceClient = this.waitForService(SERVICE_NAME,
            GetTransform._TYPE, connectedNode, 1000);
        assertNotNull("Service Client Not Null assertion", serviceClient);

        // Setup transform publisher
        final Publisher<tf2_msgs.TFMessage> tfPub = connectedNode.newPublisher("/tf", TFMessage._TYPE);
        Time startTime = connectedNode.getCurrentTime();
        Time endTime = startTime.add(Duration.fromMillis(subscriberTimeout));
        // Ensure that there is a subscriber to the /tf topic (should be the transform server)
        while (!tfPub.hasSubscribers()) {
          if (startTime.compareTo(endTime) >= 1) {
            fail("No subscribers to /tf detected after timeout: " + subscriberTimeout);
          }
        }
        // Build message to publish
        TFMessage tfMsg = tfPub.newMessage();

        // Make transform stamped messages from transform and inverted transform
        final TransformStamped tfStamped = messageFactory.newFromType(TransformStamped._TYPE);
        tfStamped.getHeader().setSeq(0);
        tfStamped.getHeader().setFrameId("parent_frame");
        tfStamped.getHeader().setStamp(connectedNode.getCurrentTime());
        tfStamped.setChildFrameId("child_frame");
        tfStamped.setTransform(transformMsg);
        tfMsg.setTransforms(new ArrayList<>(Arrays.asList(tfStamped)));

        final TransformStamped tfInvertedStamped = messageFactory.newFromType(TransformStamped._TYPE);
        tfInvertedStamped.getHeader().setSeq(tfStamped.getHeader().getSeq() + 1);
        tfInvertedStamped.getHeader().setFrameId(tfStamped.getChildFrameId());
        tfInvertedStamped.getHeader().setStamp(tfStamped.getHeader().getStamp());
        tfInvertedStamped.setChildFrameId(tfStamped.getHeader().getFrameId());
        tfInvertedStamped.setTransform(invertedTransformMsg);

        // Publish non-inverted transform message
        tfPub.publish(tfMsg);

        //Request Non-Inverse transform. Assert false on failure or incorrect transform returned
        final cav_srvs.GetTransformRequest request = serviceClient.newMessage();
        request.setParentFrame(tfStamped.getHeader().getFrameId());
        request.setChildFrame(tfStamped.getChildFrameId());

        try {
          RosServiceSynchronizer.callSync(serviceClient, request, new ServiceResponseListener<GetTransformResponse>() {
            @Override
            public void onSuccess(GetTransformResponse response) {
              // Compare the requested transform with the
              assertTrue(isEqualTransform(tfStamped, response.getTransform()));
              countDownLatch.countDown();
            }

            @Override
            public void onFailure(RemoteException e) {
              fail("Failed to call get_transform service");
            }
          });
        } catch (InterruptedException e) {
          fail("Failed to call get_transform service");
        }

        //Request Inverse transform. Assert false on failure or incorrect transform returned
        final cav_srvs.GetTransformRequest inverseRequest = serviceClient.newMessage();
        inverseRequest.setParentFrame(tfInvertedStamped.getHeader().getFrameId());
        inverseRequest.setChildFrame(tfInvertedStamped.getChildFrameId());

        try {
          RosServiceSynchronizer.callSync(serviceClient, inverseRequest,
              new ServiceResponseListener<GetTransformResponse>() {
                @Override
                public void onSuccess(GetTransformResponse response) {
                  // Compare the requested transform with the
                  assertTrue(isEqualTransform(tfInvertedStamped, response.getTransform()));
                  countDownLatch.countDown();
                }

                @Override
                public void onFailure(RemoteException e) {
                  fail("Failed to call get_transform service");
                }
              });
        } catch (InterruptedException e) {
          fail("Failed to call get_transform service");
        }
      }

      @Override
      protected void handleException(Throwable e) {
        fail("Handle exception reached in test case");
      }
    };

    // Start the transform server node
    TransformServer tfServer = new TransformServer();
    nodeMainExecutor.execute(tfServer, nodeConfiguration);
    // Start the anonymous node to test the server
    nodeMainExecutor.execute(anonNode, nodeConfiguration);
    assertTrue(countDownLatch.await(10, TimeUnit.SECONDS)); // Check if service calls were successful
    // Shutdown nodes
    nodeMainExecutor.shutdownNodeMain(anonNode);
    // Shutting down the transform server from this test results in a exception on printing the service address
    nodeMainExecutor.shutdownNodeMain(tfServer);
    // Stack trace is automatically logged
    // ROS is shutdown automatically in cleanup from ROS Test
  }

  //

  /**
   * Helper function compares two TransformStamped messages for equality
   *
   * @param t1 the first transform
   * @param t2 the second transform
   * @return True is both transforms are equivalent withing 0.0000001
   */
  private boolean isEqualTransform(TransformStamped t1, TransformStamped t2) {
    return t1.getHeader().getFrameId().equals(t2.getHeader().getFrameId())
        && t1.getChildFrameId().equals(t2.getChildFrameId()) && Transform.fromTransformMessage(t1.getTransform())
            .almostEquals(Transform.fromTransformMessage(t2.getTransform()), 0.0000001);
  }
}
