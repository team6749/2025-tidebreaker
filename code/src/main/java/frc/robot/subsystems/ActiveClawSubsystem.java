// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volt;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
@Logged
public class ActiveClawSubsystem extends SubsystemBase {
  TalonFX clawMotorLeft;
  TalonFX clawMotorRight;
  ConstrainedArmSubsystem armSubsystem;
  Voltage idleVoltage = Volts.of(0.1);
  Voltage shootLowVoltage = Volts.of(-2.5);
  Voltage shootHighVoltage = Volts.of(1);
  boolean clawInverted = true;
  boolean isBrakeModeOn = true;
  DigitalInput clawLimitSwitch;
  /** Creates a new ActiveClaw. */
  public ActiveClawSubsystem(ConstrainedArmSubsystem arm) {
    clawMotorLeft = new TalonFX(Constants.clawMotorLeftID);
    clawMotorRight = new TalonFX(Constants.clawMotorRightID); 
    armSubsystem = arm;
  brakeMode(isBrakeModeOn);
    clawLimitSwitch = new DigitalInput(4); //placeholder before limit switch is on
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private void runVolts(Voltage voltage) {
    clawMotorLeft.setVoltage(-voltage.in(Volts));
    clawMotorRight.setVoltage(voltage.in(Volts));
  }

  public Command clawIdleState() {
    Command command = Commands.runEnd(() -> runVolts(Volts.of(stopOnIntake())),() -> stop(),this);
    command.setName("clawIdleState");
    return command;
  }

  public Command clawLowShoot() {
    Command command = Commands.runEnd(() -> runVolts((armSubsystem.getPosition().in(Radians) > 0) ? shootHighVoltage : shootLowVoltage),() -> stop(), this);
    command.setName("clawLowShoot");
    return command;
  }

  public void brakeMode(boolean isBrakeModeOn) {
    clawMotorLeft.setNeutralMode(isBrakeModeOn? NeutralModeValue.Brake: NeutralModeValue.Coast);
    clawMotorRight.setNeutralMode(isBrakeModeOn? NeutralModeValue.Brake: NeutralModeValue.Coast);
  }

  public double stopOnIntake() {
    return (clawLimitSwitch.get() ? 0:1);
  }
  public void stop() {
    clawMotorLeft.stopMotor();
    clawMotorRight.stopMotor();
  }
}
