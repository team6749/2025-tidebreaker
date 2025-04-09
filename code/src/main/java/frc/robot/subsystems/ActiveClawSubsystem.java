// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volt;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
@Logged
public class ActiveClawSubsystem extends SubsystemBase {
  TalonFX clawMotor;
  Voltage idleVoltage = Volts.of(0.5);
  Voltage shootLowVoltage = Volts.of(-1);
  Voltage shootHighVoltage = Volts.of(1);
  DigitalInput clawLimitSwitch;
  /** Creates a new ActiveClaw. */
  public ActiveClawSubsystem() {
    clawMotor = new TalonFX(Constants.clawMotorID); //placeholder before the claw is made
    brakeMode(true);
    clawLimitSwitch = new DigitalInput(6);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private void runVolts(Voltage voltage) {
    clawMotor.setVoltage(voltage.in(Volts));
  }

  public Command clawIdleState() {
    Command command = Commands.runEnd(() -> runVolts(Volts.of(stopOnIntake())),() -> stop(),this);
    command.setName("clawIdleState");
    return command;
  }

  public Command clawLowShoot() {
    Command command = Commands.run(() -> runVolts(shootLowVoltage), this);
    command.setName("clawLowShoot");
    return command;
  }
  public Command clawHighShoot() {
    Command command = Commands.run(() -> runVolts(shootHighVoltage), this);
    command.setName("clawhighShoot");
    return command;
  }
  public void brakeMode(boolean isBrakeModeOn) {
    clawMotor.setNeutralMode(isBrakeModeOn? NeutralModeValue.Brake: NeutralModeValue.Coast);
  }

  public double stopOnIntake() {
    return (clawLimitSwitch.get() ? 0:1);
  }
  public void stop() {
    clawMotor.stopMotor();
  }
}
