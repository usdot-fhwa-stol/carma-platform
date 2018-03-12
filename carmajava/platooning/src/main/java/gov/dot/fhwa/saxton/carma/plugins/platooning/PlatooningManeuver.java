package gov.dot.fhwa.saxton.carma.plugins.platooning;

import org.ros.message.Time;

import gov.dot.fhwa.saxton.carma.guidance.IGuidanceCommands;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.ComplexManeuverBase;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.IAccStrategy;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.IManeuverInputs;
import gov.dot.fhwa.saxton.carma.guidance.plugins.IPlugin;

public class PlatooningManeuver extends ComplexManeuverBase {

    protected IPlatooningCommandInputs commandInputs_;

    /**
     * Constructor where provides all relevant inputs
     *
     * @param planner The plugin responsible for this maneuver
     * @param commandInputs Input which provides the current desired commands from the platooning speed control logic
     * @param currentState Input which provides the current state of the vehicle
     * @param commandsOutputs The target for calculated commands
     * @param startDist The distance along the route to the maneuver starting point
     * @param endDist The distance along the route which marks the maneuver end point
     * @param minCompletionTime The minimum anticipated execution time
     * @param maxCompletionTime The maximum anticipated execution time
     * @param minExpectedSpeed The minimum expected speed at the end of maneuver
     * @param maxExpectedSpeed The maximum expected speed at the end of maneuver
     */
    protected PlatooningManeuver(IPlugin planner, IPlatooningCommandInputs commandInputs,
            IManeuverInputs currentState, IGuidanceCommands commandsOutputs, IAccStrategy accStrategy,
            double startDist, double endDist, Time minCompletionTime, Time maxCompletionTime,
            double minExpectedSpeed, double maxExpectedSpeed) {
        super(planner, currentState, commandsOutputs, accStrategy,
                startDist, endDist, minCompletionTime, maxCompletionTime, minExpectedSpeed, maxExpectedSpeed);
        commandInputs_ = commandInputs;
    }

    protected PlatooningManeuver(IPlugin planner, IPlatooningCommandInputs commandInputs,
            IManeuverInputs currentState, IGuidanceCommands commandsOutputs, IAccStrategy accStrategy,
            double startDist, double endDist, Time minCompletionTime, Time maxCompletionTime) {
        super(planner, currentState, commandsOutputs, accStrategy,
                startDist, endDist, minCompletionTime, maxCompletionTime);
        commandInputs_ = commandInputs;
    }

    protected PlatooningManeuver(IPlugin planner, IPlatooningCommandInputs commandInputs,
            IManeuverInputs currentState, IGuidanceCommands commandsOutputs, IAccStrategy accStrategy,
            double startDist, double endDist, double minExpectedSpeed, double maxExpectedSpeed) {
        super(planner, currentState, commandsOutputs, accStrategy,
                startDist, endDist, minExpectedSpeed, maxExpectedSpeed);
        commandInputs_ = commandInputs;
    }
    
    @Override
    protected double generateSpeedCommand() {
        return commandInputs_.getLastSpeedCommand();
    }

    @Override
    protected double generateMaxAccelCommand() {
        return commandInputs_.getMaxAccelLimit();
    }

    @Override
    public String toString() {
        return "PlatooningManeuver [startDist_=" + startDist_ + ", endDist_=" + endDist_ + ", minCompletionTime_="
                + minCompletionTime_ + ", maxCompletionTime_=" + maxCompletionTime_ + ", minExpectedSpeed_="
                + minExpectedSpeed_ + ", maxExpectedSpeed_=" + maxExpectedSpeed_ + "]";
    }
}
