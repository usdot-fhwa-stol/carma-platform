package gov.dot.fhwa.saxton.carma.geometry.cartesian.cartesian;

/**
 * Created by mcconnelms on 9/22/17.
 */
public interface DimensionalObject {

  /**
   * Gets the number of dimensions which fully define the respective object
   * @return the number of dimensions. A value greater than 0.
   */
  int getNumDimensions();
}
