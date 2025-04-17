// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

@Logged
public class ActiveClawSubsystem extends SubsystemBase {
  static Voltage idleVoltage = Volts.of(0.75);
  static Voltage shootLowVoltage = Volts.of(-2.5);
  static Voltage shootHighVoltage = Volts.of(1);

  private TalonFX clawMotorLeft;
  private TalonFX clawMotorRight;
  private ConstrainedArmSubsystem armSubsystem;
  private  Debouncer debounce = new Debouncer(0.15);


  private boolean isBrakeModeOn = true;
  private LinearVelocity clawVelocity = MetersPerSecond.of(0);
  private LinearVelocity triggerVelocity = MetersPerSecond.of(0);
  private DigitalInput clawLimitSwitch;

  // Default to allow for easy preload without limit switch
  boolean isStallDetected = true;

  /** Creates a new ActiveClaw. */
  public ActiveClawSubsystem(ConstrainedArmSubsystem arm) {
    clawMotorLeft = new TalonFX(Constants.clawMotorLeftID);
    clawMotorRight = new TalonFX(Constants.clawMotorRightID);
    armSubsystem = arm;
    brakeMode(isBrakeModeOn);
    clawLimitSwitch = new DigitalInput(4); // placeholder before limit switch is on
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    clawVelocity = MetersPerSecond.of((((Math.abs(clawMotorLeft.getVelocity().getValueAsDouble()))
        + Math.abs(clawMotorRight.getVelocity().getValueAsDouble())) / 2) * Inches.of(4).in(Meters) * Math.PI);

    LinearVelocity possibleNewTriggerVelocity = clawVelocity.minus(MetersPerSecond.of(1));
    if (possibleNewTriggerVelocity.gt(triggerVelocity)) {
      triggerVelocity = possibleNewTriggerVelocity;
    }

    if (clawVelocity.lt(triggerVelocity)) {
      isStallDetected = true;
    }
  }

  private void runVolts(Voltage voltage) {
    clawMotorLeft.setVoltage(-voltage.in(Volts));
    clawMotorRight.setVoltage(voltage.in(Volts));
  }

  public void clearStallDetect () {
    isStallDetected = false;
    triggerVelocity = MetersPerSecond.zero();
  }

  public Command clawIdleState() {
    Command command = Commands.startRun(() -> {
      clearStallDetect();
    }, () -> {
      if (hasCoral()) {
        stop();
      } else {
        runVolts(idleVoltage);
      }
    }, this).finallyDo(() -> stop());
    command.setName("clawIdleState");
    return command;
  }

  public Command clawLowShoot() {
    Command command = Commands.runEnd(
        () -> runVolts((armSubsystem.getPosition().in(Radians) > 0) ? shootHighVoltage : shootLowVoltage), () -> stop(),
        this);
    command.setName("clawLowShoot");
    return command;
  }

  public boolean hasCoral () {
    return clawLimitSwitch.get() || debounce.calculate(isStallDetected);
  }

  public boolean isLimitSwitch() {
    return clawLimitSwitch.get();
  }

  public void brakeMode(boolean isBrakeModeOn) {
    clawMotorLeft.setNeutralMode(isBrakeModeOn ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    clawMotorRight.setNeutralMode(isBrakeModeOn ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  public void stop() {
    clawMotorLeft.stopMotor();
    clawMotorRight.stopMotor();
  }
}
