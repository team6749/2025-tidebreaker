package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

@Logged
public class SwerveModuleSim {

    private static final Time timestep = Milliseconds.of(20);

    private static final DCMotor driveMotorModel = DCMotor.getKrakenX60(1);
    private static final DCMotor turnMotorModel = DCMotor.getKrakenX60(1);

    @NotLogged
    private final DCMotorSim driveSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(driveMotorModel, 0.03, SwerveConstants.driveReduction),
            driveMotorModel);
    @NotLogged
    private final DCMotorSim turnSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(turnMotorModel, 0.005, SwerveConstants.angleReduction),
            turnMotorModel);

    private PIDController driveController = new PIDController(2, 0, 0);
    private PIDController turnController = new PIDController(3, 0, 0);

    SwerveModuleState targetState = new SwerveModuleState();

    SwerveModuleSim () {
        turnController.enableContinuousInput(-Math.PI, Math.PI);
    }

    void periodic() {

        driveController.setSetpoint(targetState.speedMetersPerSecond);
        turnController.setSetpoint(targetState.angle.getRadians());
        
        // Jank easy feed forward
        Double driveVolts = (targetState.speedMetersPerSecond * 2.2) + driveController.calculate(getState().speedMetersPerSecond);
        Double turnVolts = turnController.calculate(getState().angle.getRadians());


        // Test max speed
        // driveVolts = 12d;

        // TODO eventually calculate the current draw of all simulated systems
        // double drawAmps = driveSim.getCurrentDrawAmps() +
        // turnSim.getCurrentDrawAmps();
        // double voltage = BatterySim.calculateDefaultBatteryLoadedVoltage(drawAmps);

        

        driveSim.setInputVoltage(clampVolts(driveVolts, 12));
        turnSim.setInputVoltage(clampVolts(turnVolts, 12));
        driveSim.update(timestep.in(Seconds));
        turnSim.update(timestep.in(Seconds));
    }

    @Logged
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                driveSim.getAngularPosition().in(Rotations)
                        * SwerveConstants.wheelCircumference.in(Meters),
                Rotation2d.fromRadians(turnSim.getAngularPositionRad()));
    }

    @Logged
    public SwerveModuleState getState() {
        return new SwerveModuleState(
                MetersPerSecond.of(driveSim.getAngularVelocity().in(RotationsPerSecond)
                        * SwerveConstants.wheelCircumference.in(Meters)),
                Rotation2d.fromRadians(turnSim.getAngularPositionRad()));
    }

    public void setState(SwerveModuleState state) {
        targetState = state;
    }

    void stop() {
        targetState = new SwerveModuleState();
        driveSim.setInputVoltage(0);
        turnSim.setInputVoltage(0);
    }

    private double clampVolts(double input, double maxVoltage) {
        return Math.min(Math.max(input, -maxVoltage), maxVoltage);
    }
}
