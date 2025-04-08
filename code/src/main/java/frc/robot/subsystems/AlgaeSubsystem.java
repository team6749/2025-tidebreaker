// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volt;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaeSubsystem extends SubsystemBase {
  TalonFX algaeMotor;
  Voltage idleVoltage = Volts.of(0.5);
  Voltage shootVoltage = Volts.of(-1);
  DigitalInput algaeLimitSwitch;
  /** Creates a new ActiveClaw. */
  public AlgaeSubsystem() {
    brakeMode(true);
    algaeMotor = new TalonFX(Constants.algaeMotorID); //placeholder before the algae subsystem is made
    algaeLimitSwitch = new DigitalInput(5555);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private void runVolts(Voltage voltage) {
    algaeMotor.setVoltage(voltage.in(Volts));
  }

  public Command algaeIdleState() {
    Command command = Commands.run(() -> runVolts(Volts.of(stopOnIntake())), this);
    return command;
  }

  public Command algaeShoot() {
    Command command = Commands.run(() -> runVolts(shootVoltage), this);
    return command;
  }

  public void brakeMode(boolean isBrakeModeOn) {
    algaeMotor.setNeutralMode(isBrakeModeOn? NeutralModeValue.Brake: NeutralModeValue.Coast);
  }
  public double stopOnIntake() {
    return (algaeLimitSwitch.get() ? 0:1);
  }

}
