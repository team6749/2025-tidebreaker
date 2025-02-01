// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import javax.sound.sampled.Line;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutVelocity;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import pabeles.concurrency.IntOperatorTask.Min;

public class SwerveModuleReal implements SwerveModuleBase {
  public TalonFX driveMotor;
  public TalonFX angleMotor;
  public CANcoder encoder;
  public String name;
  private LinearVelocity velocity = MetersPerSecond.zero();
  private Rotation2d angle = Rotation2d.kZero;
  private Distance position = Meters.zero();
  public PIDController anglePID = new PIDController(0, 0, 0);
  public PIDController drivePID = new PIDController(0, 0, 0);
  public SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(1,2);
  /** Creates a new SwerveModule. */
  public SwerveModuleReal(int DriveMotorPort, int AngleMotorPort, int encoder) {

    driveMotor = new TalonFX(DriveMotorPort);
    angleMotor = new TalonFX(AngleMotorPort);
    drivePID.enableContinuousInput(-Math.PI, Math.PI);
  }
// hello, i dont do code for a reason -Austin

  @Override
  public void periodic() {
    velocity = MetersPerSecond.of(driveMotor.getVelocity().getValue().in(RotationsPerSecond) / SwerveConstants.driveReduction * SwerveConstants.wheelCircumference.in(Meters));
    angle = Rotation2d.fromDegrees(angleMotor.getPosition().getValueAsDouble() * 360);
    position = Meters.of(driveMotor.getPosition().getValue().in(Rotations) / SwerveConstants.driveReduction * SwerveConstants.wheelCircumference.in(Meters));
    // This method will be called once per scheduler run
    }
    @Override
    public SwerveModuleState getState() {
      return new SwerveModuleState(velocity,angle);
    }
    public void stop() {
      driveMotor.stopMotor();
      angleMotor.stopMotor();
    }

    @Override
    public void runClosedLoop(SwerveModuleState inState) {
      SwerveModuleState currentState = getState();
      SwerveModuleState desiredState = new SwerveModuleState(inState.speedMetersPerSecond,inState.angle);
      desiredState.optimize(currentState.angle);
      desiredState.cosineScale(currentState.angle);
      Voltage driveOutput = Volts.of(drivePID.calculate(currentState.speedMetersPerSecond, desiredState.speedMetersPerSecond));
      Voltage angleOutput = Volts.of(anglePID.calculate(currentState.angle.getRadians(), desiredState.angle.getRadians()));
      driveMotor.setVoltage((driveFeedForward.calculate(desiredState.speedMetersPerSecond) + driveOutput.in(Volts)));
      angleMotor.setVoltage(angleOutput.in(Volts));
    }

    @Override
    public void runOpenLoop(Voltage drive, Voltage turn) {
      driveMotor.setVoltage(drive.in(Volts));
      angleMotor.setVoltage(turn.in(Volts));
    }

    @Override
    public SwerveModulePosition getPosition() {
      return new SwerveModulePosition(position, angle);
    }

    @Override
    public void setBrakeMode(boolean brake) {
      driveMotor.setNeutralMode(brake ? NeutralModeValue.Brake:NeutralModeValue.Coast);
    }

  }