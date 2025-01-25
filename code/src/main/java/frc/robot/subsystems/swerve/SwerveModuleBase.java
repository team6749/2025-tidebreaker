package frc.robot.subsystems.swerve;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Voltage;

// Defines the standard IO for a swerve module.
@Logged
public interface SwerveModuleBase {
    
    // Run every robot loop
    void periodic ();

    // Tells the module to run to this state
    void runClosedLoop (SwerveModuleState state);

    // should apply open loop voltage to the motors.
    // Useful for running characterization
    void runOpenLoop (Voltage drive, Voltage turn);

    // Returns the current position of the swerve module
    public SwerveModulePosition getPosition();

    // Returns the current state of the swerve module
    public SwerveModuleState getState();

    // Stops motors (sets output to zero volts)
    void stop();

    // sets the brake mode on the module, this should not be called
    void setBrakeMode (boolean brake);

}
