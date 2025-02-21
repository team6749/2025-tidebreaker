// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

@Logged 
public class ClimberSubsystem extends SubsystemBase {
  Timer timer = new Timer();
  TalonFX climberMotor = new TalonFX(Constants.climberMotorPort);
  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    timer.start();
  }

  @Override
  public void periodic() {
    Climb();
    // This method will be called once per scheduler run
  }

  public void Climb() {
    SmartDashboard.putNumber("timer", timer.getMatchTime());
    if (timer.getMatchTime() < 215) {
      climberMotor.setVoltage(Volts.of(2).in(Volts));
    } 
  }
}
