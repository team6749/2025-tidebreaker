// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;

@Logged
public class SwerveModuleReal implements SwerveModuleBase {
  public TalonFX driveMotor;
  public TalonFX angleMotor;
  public CANcoder encoder;
  private LinearVelocity velocity = MetersPerSecond.zero();
  private Angle angle = Radians.zero();
  private Distance position = Meters.zero();
  public PIDController drivePID = new PIDController(1.8, 0, 0);
  public PIDController anglePID = new PIDController(3.7, 0, 0);
  public SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(1, 2);

  /** Creates a new SwerveModule. */
  public SwerveModuleReal(int driveMotorPort, int angleMotorPort, int encoderPort) {

    driveMotor = new TalonFX(driveMotorPort);
    angleMotor = new TalonFX(angleMotorPort);
    anglePID.enableContinuousInput(-Math.PI, Math.PI);
    encoder = new CANcoder(encoderPort);
    angleMotor.setNeutralMode(NeutralModeValue.Brake);

    var angleMotorCurrentLimits = new CurrentLimitsConfigs().withStatorCurrentLimit(Amps.of(60))
        .withStatorCurrentLimitEnable(true);
    angleMotor.getConfigurator().apply(angleMotorCurrentLimits);

  }

  @Override
  public void periodic() {
    velocity = MetersPerSecond.of(driveMotor.getVelocity().getValue().in(RotationsPerSecond)
        / SwerveConstants.driveReduction * SwerveConstants.wheelCircumference.in(Meters));
    angle = Rotations.of(encoder.getAbsolutePosition().getValueAsDouble());
    position = Meters.of(driveMotor.getPosition().getValue().in(Rotations) / SwerveConstants.driveReduction
        * SwerveConstants.wheelCircumference.in(Meters));
    // This method will be called once per scheduler run
  }


  @Override
  public SwerveModuleState getState() {
    return new SwerveModuleState(velocity, Rotation2d.fromRadians(angle.in(Radians)));
  }

  public void stop() {
    driveMotor.stopMotor();
    angleMotor.stopMotor();
  }

  @Override
  public void runClosedLoop(SwerveModuleState inState) {
    SwerveModuleState currentState = getState();
    SwerveModuleState desiredState = new SwerveModuleState(inState.speedMetersPerSecond, inState.angle);
    desiredState.optimize(currentState.angle);
    desiredState.cosineScale(currentState.angle);
    Voltage driveOutput = Volts
        .of(drivePID.calculate(currentState.speedMetersPerSecond, desiredState.speedMetersPerSecond));

    Voltage angleOutput = Volts
        .of(anglePID.calculate(MathUtil.angleModulus(currentState.angle.getRadians()),
            desiredState.angle.getRadians()));
    driveMotor.setVoltage((driveFeedForward.calculate(desiredState.speedMetersPerSecond) + driveOutput.in(Volts)));
    angleMotor.setVoltage(-angleOutput.in(Volts));
  }

  @Override
  public void runOpenLoop(Voltage drive, Voltage turn) {
    driveMotor.setVoltage(drive.in(Volts));
    SwerveModuleState currentState = getState();
    SwerveModuleState desiredState = new SwerveModuleState(MetersPerSecond.of(0), Rotation2d.fromRadians(0));
        Voltage angleOutput = Volts
        .of(anglePID.calculate(MathUtil.angleModulus(currentState.angle.getRadians()),
            desiredState.angle.getRadians()));
            angleMotor.setVoltage(-angleOutput.in(Volts));
  }

  @Override
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(position, Rotation2d.fromRadians(angle.in(Radians)));
  }

  @Override
  public void setBrakeMode(boolean brake) {
    driveMotor.setNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

}