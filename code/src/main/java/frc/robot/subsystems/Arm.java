// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveConstants;

public class Arm extends SubsystemBase {
  PIDController armPID = new PIDController(0, 0, 0);
  ArmFeedforward feedForward = new ArmFeedforward(0, 0,0);
  double[] setPoints = {0,1};
  TalonFX armMotor = new TalonFX(Constants.armMotorPort);//todo put in actual motor
  Rotation2d angle = Rotation2d.kZero;
  Angle maxAngle = Radians.of(Math.PI);
  Angle minAngle = Radians.of(0);
  AngularVelocity maxVelocity = RadiansPerSecond.of(0);
  AngularAcceleration maxAcceleration = RadiansPerSecondPerSecond.of(0); //to do, find these values.
  private final TrapezoidProfile trapezoidProfile = new TrapezoidProfile(
    new TrapezoidProfile.Constraints(maxVelocity.in(RadiansPerSecond),
        maxAcceleration.in(RadiansPerSecondPerSecond)));


  public final DCMotor armMotorModel = DCMotor.getFalcon500(1);
  private final DCMotorSim armSim = new DCMotorSim(
    LinearSystemId.createDCMotorSystem(armMotorModel, 0.03, SwerveConstants.driveReduction),
    armMotorModel);


  /** Creates a new Arm. */
  public Arm() {
  }

  public AngularVelocity getVelocity() {
    return RadiansPerSecond.of(armMotor.getVelocity().getValueAsDouble());
  }

  @Override
  public void periodic() {
    if(RobotBase.isSimulation()) {
      armSim.update(Constants.simulationTimestep.in(Seconds));
    }
    angle = Rotation2d.fromRotations(armMotor.getPosition().getValueAsDouble()/ Constants.armGearRatio);
    // This method will be called once per scheduler run
  }

  public void runClosedLoop(Rotation2d desiredAngle) {
    Rotation2d currentAngle = getAngle();
    TrapezoidProfile.State currentState = new State(currentAngle.getRadians(),getVelocity().in(RadiansPerSecond));
    TrapezoidProfile.State desiredState = new State(desiredAngle.getRadians(),0);
    trapezoidProfile.calculate(Constants.simulationTimestep.in(Seconds), currentState, desiredState);
    Voltage output = Volts.of(armPID.calculate(currentState.position,desiredState.position));
    Voltage feedForwardSystem = Volts.of(feedForward.calculate(currentState.velocity,desiredState.velocity));
    armMotor.setVoltage(output.in(Volts) + feedForwardSystem.in(Volts));
    //Spencer's code says this doesn't work so that'll be fun to debug.
  }

  public Command armCommand(Rotation2d desiredAngle) {
    Command command = Commands.run(() ->{
      runClosedLoop(desiredAngle);
    });
    return command;
  }

  public void neutralState() {
    armMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public Rotation2d getAngle() {
      return angle;
  }
}
