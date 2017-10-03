package gov.dot.fhwa.saxton.carma.rosutils;

import org.ros.node.service.ServiceResponseListener;
import org.ros.exception.RemoteException;

/**
 * Value holder for ROS service response data and/or an error
 */
public class RosServiceResponse<T> {
  protected T value;
  protected RemoteException error;

  public RosServiceResponse(T value) {
    this.value = value;
    this.error = null;
  }

  public RosServiceResponse(RemoteException error) {
    this.error = error;
    this.value = null;
  }

  /**
  * If the call returns correctly the get method will return the value of the response object. If
  * the call failed the get method will re-throw the error captured during communications.
   */
  public T get() throws RemoteException {
    if (value != null) {
      return value;
    } else {
      throw error;
    }
  }

  public void get(ServiceResponseListener<T> listener) {
    if (value != null) {
      listener.onSuccess(value);
    } else {
      listener.onFailure(error);
    }
  }
}