package gov.dot.fhwa.saxton.carma.guidance.maneuvers;

import gov.dot.fhwa.saxton.carma.guidance.IGuidanceCommands;

/**
 * Represents a longitudinal maneuver in which the vehicle steadily increases its speed.
 */
public class SpeedUp extends LongitudinalManeuver {

    private double                  workingAccel_;              // m/s^2 that we will actually use
    private double                  timeStep_;                  // duration of a single computational time step, sec
    private double                  prevCmd_;                   // speed command from the previous time step, m/s


    /**
     * ASSUMES that the target speed has been specified such that it does not exceed and infrastructure speed limit.
     */
    @Override
    public void plan(IManeuverInputs inputs, IGuidanceCommands commands, double startDist) throws IllegalStateException, ArithmeticException {
        super.plan(inputs, commands, startDist);

        timeStep_ = (double)inputs_.getTimeStep() / 1000.0; //convert from ms to sec
        prevCmd_ = startSpeed_;

        //verify proper speed relationships
        if (endSpeed_ <= startSpeed_) {
            throw new ArithmeticException("SpeedUp maneuver being planned with startSpeed = " + startSpeed_ +
                                            ", endSpeed = " + endSpeed_);
        }

        //if speed change is going to be only slight then
        double deltaV = endSpeed_ - startSpeed_; //always positive
        workingAccel_ = maxAccel_;
        if (deltaV < SMALL_SPEED_CHANGE) {
            //cut the acceleration rate to half the limit
            workingAccel_ = 0.5 * maxAccel_;
        }

        //compute the distance to be covered during a linear (in time) speed change, assuming perfect vehicle response
        double idealLength = (startSpeed_*deltaV + 0.5*deltaV*deltaV) / workingAccel_;

        //compute the time it will take to perform this ideal speed change and the interpolation factor
        //TODO: do we need this - maybe caller will want it?     deltaT_ = deltaV / workingAccel_;

        //add the distance covered by the expected vehicle lag
        double lagDistance = startSpeed_*inputs_.getResponseLag();
        endDist_ = startDist_ + idealLength + lagDistance;
    }


    @Override
    public void executeTimeStep() throws IllegalStateException {

        //if current location is outside the defined boundaries for this maneuver throw an exception
        double currentLocation = inputs_.getDistanceFromRouteStart();
        if (currentLocation < startDist_  ||  currentLocation > endDist_) {
            throw new IllegalStateException("SpeedUp maneuver attempted to execute at distance " + currentLocation
                    + ". Maneuver start dist = " + startDist_ + ", end dist = " + endDist_);
        }

        //compute command based on linear interpolation on time steps
        //Note that commands will begin changing immediately, although the actual speed will not change much until
        // the response lag has passed. Thus, we will hit the target speed command sooner than we pass the end distance.
        double cmd = prevCmd_ + workingAccel_*timeStep_;
        if (cmd > endSpeed_) {
            cmd = endSpeed_;
        }
        prevCmd_ = cmd;

        //invoke the ACC override
        cmd = accOverride(cmd);

        //send the command to the vehicle
        commands_.setCommand(cmd, workingAccel_);
    }
}
