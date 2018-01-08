package gov.dot.fhwa.saxton.carma.plugins.platooning;

import org.ros.message.Time;

import gov.dot.fhwa.saxton.carma.guidance.IGuidanceCommands;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.ComplexManeuverBase;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.IAccStrategy;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.IManeuverInputs;

public class PlatooningManeuver extends ComplexManeuverBase {

    protected IPlatooningCommandInputs commandInputs_;

    /**
     * Constructor where provides all relevant inputs
     *
     * @param commandInputs Input which provides the current desired commands from the platooning speed control logic
     * @param inputs Input which provides the current state of the vehicle
     * @param commands The target for calculated commands
     * @param startDist The distance along the route to the maneuver starting point
     * @param endDist The distance along the route which marks the maneuver end point
     * @param minCompletionTime The minimum anticipated execution time
     * @param maxCompletionTime The maximum anticipated execution time
     * @param minExpectedSpeed The minimum expected speed
     * @param maxExpectedSpeed The maximum expected speed
     */
    protected PlatooningManeuver(IPlatooningCommandInputs commandInputs, IManeuverInputs inputs, IGuidanceCommands commands,
            IAccStrategy accStrategy, double startDist, double endDist, Time minCompletionTime, Time maxCompletionTime,
            double minExpectedSpeed, double maxExpectedSpeed) {
        super(inputs, commands, accStrategy, startDist, endDist, minCompletionTime, maxCompletionTime, minExpectedSpeed,
                maxExpectedSpeed);
        commandInputs_ = commandInputs;
    }

    protected PlatooningManeuver(IPlatooningCommandInputs commandInputs, IManeuverInputs inputs, IGuidanceCommands commands,
            IAccStrategy accStrategy, double startDist, double endDist, Time minCompletionTime,
            Time maxCompletionTime) {
        super(inputs, commands, accStrategy, startDist, endDist, minCompletionTime, maxCompletionTime);
        commandInputs_ = commandInputs;
    }

    protected PlatooningManeuver(IPlatooningCommandInputs commandInputs, IManeuverInputs inputs, IGuidanceCommands commands,
            IAccStrategy accStrategy, double startDist, double endDist, double minExpectedSpeed,
            double maxExpectedSpeed) {
        super(inputs, commands, accStrategy, startDist, endDist, minExpectedSpeed, maxExpectedSpeed);
        commandInputs_ = commandInputs;
    }
    
    @Override
    protected double generateSpeedCommand() {
        if(checkTimeout()) {
            // TODO handle speed command timeout
        }
        return commandInputs_.getLastSpeedCommand();
    }

    @Override
    protected double generateMaxAccelCommand() {
        if(checkTimeout()) {
            // TODO handle speed command timeout
        }
        return commandInputs_.getMaxAccelLimit();
    }

    private boolean checkTimeout() {
        return commandInputs_.isTimeout();
    }
}
