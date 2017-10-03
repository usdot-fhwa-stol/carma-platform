package gov.dot.fhwa.saxton.carma.rosutils;

import org.ros.node.service.*;
import org.ros.exception.*;

public class RosServiceSynchronizer {
  public static <R, T> RosServiceResult<T> synch(ServiceClient<R, T> client, R request) {
    final RosServiceResult<T> res = new RosServiceResult<>();

    client.call(request, new ServiceResponseListener<T>() {
      @Override
      public void onSuccess(T success) {
        res.complete(new RosServiceResponse<T>(success));
      }

      @Override
      public void onFailure(RemoteException e) {
        res.complete(new RosServiceResponse<T>(e));
      }
    });

    return res;
  }
}