package gov.dot.fhwa.saxton.carma.plugins.platooning;

import org.ros.message.Time;

import gov.dot.fhwa.saxton.carma.guidance.IGuidanceCommands;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.ComplexManeuverBase;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.IAccStrategy;
import gov.dot.fhwa.saxton.carma.guidance.maneuvers.IManeuverInputs;

public class PlatooningManeuver extends ComplexManeuverBase {

    protected CommandGenerator commandInputs_;

    protected PlatooningManeuver(CommandGenerator commandInputs, IManeuverInputs inputs, IGuidanceCommands commands,
            IAccStrategy accStrategy, double startDist, double endDist, Time minCompletionTime, Time maxCompletionTime,
            double minExpectedSpeed, double maxExpectedSpeed) {
        super(inputs, commands, accStrategy, startDist, endDist, minCompletionTime, maxCompletionTime, minExpectedSpeed,
                maxExpectedSpeed);
        commandInputs_ = commandInputs;
    }

    protected PlatooningManeuver(CommandGenerator commandInputs, IManeuverInputs inputs, IGuidanceCommands commands,
            IAccStrategy accStrategy, double startDist, double endDist, Time minCompletionTime,
            Time maxCompletionTime) {
        super(inputs, commands, accStrategy, startDist, endDist, minCompletionTime, maxCompletionTime);
        commandInputs_ = commandInputs;
    }

    protected PlatooningManeuver(CommandGenerator commandInputs, IManeuverInputs inputs, IGuidanceCommands commands,
            IAccStrategy accStrategy, double startDist, double endDist, double minExpectedSpeed,
            double maxExpectedSpeed) {
        super(inputs, commands, accStrategy, startDist, endDist, minExpectedSpeed, maxExpectedSpeed);
        commandInputs_ = commandInputs;
    }
    
    @Override
    protected double generateSpeedCommand() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    protected double generateMaxAccelCommand() {
        // TODO Auto-generated method stub
        return 0;
    }

}
