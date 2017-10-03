package gov.dot.fhwa.saxton.carma.rosutils;

import java.util.concurrent.*;

/**
 * BlockingQueue backed Future<T> styled class for dealing with future tasks that may fail
 * particularly in the context of ROS services
 */
public class RosServiceResult<T> {
  protected BlockingQueue<RosServiceResponse<T>> value = new ArrayBlockingQueue<>(1);

  public void complete(RosServiceResponse<T> value) {
    this.value.add(value);
  }

  public RosServiceResponse<T> get() throws InterruptedException {
    return value.take();
  }

  public RosServiceResponse<T> get(long timeout, TimeUnit unit) throws InterruptedException {
    return value.poll(timeout, unit);
  }
}