package gov.dot.fhwa.saxton.carma.route;

/**
 * Created by mcconnelms on 8/24/17.
 */
public enum RouteWorkerState {
  WAITING_FOR_AVAILABLE_ROUTES,
  WAITING_FOR_SYSTEM_START,
  WAITING_FOR_ROUTE_SELECTION,
  FOLLOWING_ROUTE
}
