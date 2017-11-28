package gov.dot.fhwa.saxton.carma.plugins.speedharm;

import org.ros.message.Duration;

/**
 * Helper class for testing the SpeedHarmonizationManeuver class
 */
public class FakeSpeedHarmInputs implements ISpeedHarmInputs {
  private double speedCommand;
  private double maxAccelLimit;
  private Duration timeSinceUpdate = Duration.fromMillis(0);

  public FakeSpeedHarmInputs(double speedCommand, double maxAccelLimit) {
    this.speedCommand = speedCommand;
    this.speedCommand = speedCommand;
    this.maxAccelLimit = maxAccelLimit;
  }

  /**
   * Methods which are used to spoof the inputs for speed harm maneuvers
   */
  public void setSpeedCommand(double speedCommand) {
    this.speedCommand = speedCommand;
  }

  public void setMaxAccelLimit(double maxAccelLimit) {
    this.maxAccelLimit = maxAccelLimit;
  }

  public void setTimeSinceLastUpdate(Duration lastUpdateTime) {
    this.timeSinceUpdate = lastUpdateTime;
  }

  @Override public double getSpeedCommand() {
    return speedCommand;
  }

  @Override public double getMaxAccelLimit() {
    return maxAccelLimit;
  }

  @Override public Duration getTimeSinceLastUpdate() {
    return timeSinceUpdate;
  }
}
