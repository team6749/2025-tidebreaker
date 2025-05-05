// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.time.Duration;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

@Logged
public class ActiveClawSubsystem extends SubsystemBase {
  static Voltage idleVoltage = Volts.of(1.5);
  static Voltage shootLowVoltage = Volts.of(-2.5);
  static Voltage shootHighVoltage = Volts.of(-2);
  static Time stallDetectResetTimerDuration = Seconds.of(0.5);

  private TalonFX clawMotor;

  @NotLogged
  private ConstrainedArmSubsystem armSubsystem;
  private Debouncer debounce = new Debouncer(0.15);

  private boolean isBrakeModeOn = true;
  private LinearVelocity clawVelocity = MetersPerSecond.of(0);
  private LinearVelocity triggerVelocity = MetersPerSecond.of(0);
  private DigitalInput beamBreakInput;

  Timer stallDetectResetTimer = new Timer();

  // Default to allow for easy preload
  boolean instantStallDetected = true;
  boolean isStalled = true;

  /** Creates a new ActiveClaw. */
  public ActiveClawSubsystem(ConstrainedArmSubsystem arm) {
    clawMotor = new TalonFX(Constants.clawMotorID);
    armSubsystem = arm;
    brakeMode(isBrakeModeOn);
    stallDetectResetTimer.start();
    beamBreakInput = new DigitalInput(4); 
  }

  @Override
  public void periodic() {
    isStalled = debounce.calculate(instantStallDetected);

    clawVelocity = getClawVelocity();

    LinearVelocity possibleNewTriggerVelocity = clawVelocity.minus(MetersPerSecond.of(0.5));
    if (stallDetectResetTimer.hasElapsed(stallDetectResetTimerDuration.in(Seconds))
        && possibleNewTriggerVelocity.gt(triggerVelocity)) {
      triggerVelocity = possibleNewTriggerVelocity;
    }

    if (clawVelocity.lt(triggerVelocity) && triggerVelocity.gt(MetersPerSecond.of(0))) {
      instantStallDetected = true;
    }
  }

  public LinearVelocity getClawVelocity() {
    //gear ratio of 4:1
    return MetersPerSecond.of(-clawMotor.getVelocity().getValueAsDouble() / 2.0 * Inches.of(4).in(Meters) / 4.0 * Math.PI);
  }

  private void runVolts(Voltage voltage) {
    clawMotor.setVoltage(-voltage.in(Volts));
  }

  public void clearStallDetect() {
    instantStallDetected = false;
    isStalled = false;
    stallDetectResetTimer.reset();
    triggerVelocity = MetersPerSecond.zero();
  }

  public Command clawIdleState() {
    Command command = Commands.run(
       () -> {
        runVolts(idleVoltage);
      if (hasCoral()) {
        stop();
      } else {
        runVolts(idleVoltage);
      }
    }, this).finallyDo(() -> stop());
    command.setName("clawIdleState");
    return command;
  }

  public Command clawShoot() {
    Command command = Commands.runEnd(
        () -> runVolts((armSubsystem.getPosition().in(Radians) > 0) ? shootHighVoltage : shootLowVoltage), () -> stop(),
        this);
    command.setName("clawLowShoot");
    return command;
  }

  public boolean hasCoral() {
    return beamBreakInput.get() || isStalled;
  }

  public boolean isbeamBreak() {
    return beamBreakInput.get();
  }

  public void brakeMode(boolean isBrakeModeOn) {
    clawMotor.setNeutralMode(isBrakeModeOn ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  public void stop() {
    clawMotor.stopMotor();
  }
}
