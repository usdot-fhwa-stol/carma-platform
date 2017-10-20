package gov.dot.fhwa.saxton.carma.roadway;

/**
 * Factory for creating obstacles based on an input string
 * TODO: Decide if this class is really needed
 */
public class ObstacleFactory {
  IObstacle makeObstacle(String type){return new Obstacle();} //TODO
}
