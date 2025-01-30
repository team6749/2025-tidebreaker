package frc.robot.loggers;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

@CustomLoggerFor(TrapezoidProfile.State.class)
public class TrapezoidPofileStateLogger extends ClassSpecificLogger<TrapezoidProfile.State> {

    public TrapezoidPofileStateLogger() {
        super(TrapezoidProfile.State.class);
    }

    @Override
    public void update(EpilogueBackend backend, TrapezoidProfile.State motor) {
        backend.log("Position", motor.position);
        backend.log("Velocity", motor.velocity);
    }
}
