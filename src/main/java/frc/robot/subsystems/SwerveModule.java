// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.units.measure.MutVelocity;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import pabeles.concurrency.IntOperatorTask.Min;

public class SwerveModule extends SubsystemBase {
  public TalonFX driveMotor;
  public TalonFX angleMotor;
  public CANcoder encoder;
  public double voltage;
  public String name;
  public double feedForward;
  public PIDController anglePID = new PIDController(Constants.SwerveConstants.angleKP, Constants.SwerveConstants.angleKI, Constants.SwerveConstants.angleKD);
  public PIDController drivePID = new PIDController(0, 0, 0);
  public Translation2d locationFromMotor;
  public SwerveModuleState state;
  public int moduleNumber;
  /** Creates a new SwerveModule. */
  public SwerveModule(String ModuleName, int DriveMotorPort, int AngleMotorPort, Translation2d ModuleLocation) {
    driveMotor = new TalonFX(DriveMotorPort);
    angleMotor = new TalonFX(AngleMotorPort);
    locationFromMotor = ModuleLocation;
    drivePID.enableContinuousInput(0, 360);
    name = ModuleName;
  }
// hello, i dont do code for a reason -Austin

  @Override
  public void periodic() {
    getDesiredState(state);
    double power = drivePID.calculate(state.speedMetersPerSecond);
    double angle = state.angle.getDegrees();
    driveMotor.set((feedForward + power) / Constants.SwerveConstants.maxVelocity);
    angleMotor.set(angle);
    // This method will be called once per scheduler run
    }

    public double getDriveVelocity() {  
      return driveMotor.getVelocity().getValueAsDouble() * Constants.SwerveConstants.WheelRadius * 2 * Math.PI;
    }
    public Rotation2d getAngle() {
      return Rotation2d.fromDegrees(angleMotor.getVelocity().getValueAsDouble() * 360);
    }
    public double getDrivePosition() {
      return driveMotor.getPosition().getValueAsDouble() * Constants.SwerveConstants.WheelRadius * 2 * Math.PI;
    }


    public SwerveModuleState getDesiredState(SwerveModuleState desiredStates) {
      if (Math.abs(desiredStates.speedMetersPerSecond) < 0.001) {
        stop();
      }
      desiredStates.optimize(getAngle());
      state = desiredStates;
      return state;
    }

    public SwerveModulePosition getModulePosition() {
      return new SwerveModulePosition(getDrivePosition(), getAngle());
    }

    public SwerveModuleState getState() {
      return new SwerveModuleState(getDriveVelocity(),getAngle());
    }
    public void stop() {
      driveMotor.stopMotor();
      angleMotor.stopMotor();
    }

  }