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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

@Logged
public class ActiveClawSubsystem extends SubsystemBase {
  static Voltage idleVoltage = Volts.of(3.387);
  static Voltage shootL2Voltage = Volts.of(-5);
  static Voltage shootL1Voltage = Volts.of(-1);
  static Voltage shootHighVoltage = Volts.of(3);
  static Time stallDetectResetTimerDuration = Seconds.of(0.5);

  private TalonFX clawMotor;

  @NotLogged
  private ConstrainedArmSubsystem armSubsystem;
  private RobotContainer robotContainer;
  private Debouncer debounce = new Debouncer(.03);

  private boolean isBrakeModeOn = true;
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
    isStalled = debounce.calculate(isNotBeamBreak());
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
  }

  public Command clawIdleState() {
    Command command = Commands.run(
       () -> {
      if(hasCoral()) {
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
        () -> runVolts(shootVoltageLogic()), () -> stop(),
        this);
    command.setName("clawLowShoot");
    return command;
  }

  public Voltage shootVoltageLogic() {
    Voltage shootingVolts;
    shootingVolts = armSubsystem.getPosition().in(Radians) > 0 ? shootHighVoltage : shootL2Voltage;
    shootingVolts = CommandScheduler.getInstance().isScheduled(robotContainer.moveToLevel1()) ? shootL1Voltage : shootL2Voltage;
    return shootingVolts;
  }

  public boolean hasCoral() {
    return isStalled;
  }

  public boolean isNotBeamBreak() {
    return !beamBreakInput.get();
  }

  public void brakeMode(boolean isBrakeModeOn) {
    clawMotor.setNeutralMode(isBrakeModeOn ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  public void stop() {
    clawMotor.stopMotor();
  }
}
