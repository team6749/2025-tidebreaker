package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

@Logged
public class SwerveModuleSim implements SwerveModuleBase {

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
    private PIDController turnController = new PIDController(4, 0, 0);

    SwerveModuleSim() {
        turnController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void periodic() {
        driveSim.update(Constants.simulationTimestep.in(Seconds));
        turnSim.update(Constants.simulationTimestep.in(Seconds));
    }

    @Override
    public void runClosedLoop(SwerveModuleState targetState) {
        SwerveModuleState currentState = getState();

        targetState.optimize(currentState.angle);
        targetState.cosineScale(currentState.angle);

        driveController.setSetpoint(targetState.speedMetersPerSecond);
        turnController.setSetpoint(targetState.angle.getRadians());

        // Jank easy feed forward
        Double driveVolts = (targetState.speedMetersPerSecond * 2.2)
                + driveController.calculate(currentState.speedMetersPerSecond);
        Double turnVolts = turnController.calculate(currentState.angle.getRadians());

        // Test max speed
        // driveVolts = 12d;

        // TODO eventually calculate the current draw of all simulated systems
        // double drawAmps = driveSim.getCurrentDrawAmps() +
        // turnSim.getCurrentDrawAmps();
        // double voltage = BatterySim.calculateDefaultBatteryLoadedVoltage(drawAmps);

        driveSim.setInputVoltage(clampVolts(driveVolts, 12));
        turnSim.setInputVoltage(clampVolts(turnVolts, 12));
    }

    @Override
    public void runOpenLoop(Voltage drive, Voltage turn) {
        driveSim.setInputVoltage(drive.in(Volts));
        turnSim.setInputVoltage(turn.in(Volts));
    }

    @Override
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                driveSim.getAngularPosition().in(Rotations)
                        * SwerveConstants.wheelCircumference.in(Meters),
                Rotation2d.fromRadians(turnSim.getAngularPositionRad()));
    }


    @Override
    public SwerveModuleState getState() {
        return new SwerveModuleState(
                MetersPerSecond.of(driveSim.getAngularVelocity().in(RotationsPerSecond)
                        * SwerveConstants.wheelCircumference.in(Meters)),
                Rotation2d.fromRadians(turnSim.getAngularPositionRad()));
    }

    @Override
    public void stop() {
        driveSim.setInputVoltage(0);
        turnSim.setInputVoltage(0);
    }

    private double clampVolts(double input, double maxVoltage) {
        return Math.min(Math.max(input, -maxVoltage), maxVoltage);
    }

    @Override
    public void setBrakeMode(boolean brake) {
        // Virtual Swerve module has no brake mode.
    }

}
