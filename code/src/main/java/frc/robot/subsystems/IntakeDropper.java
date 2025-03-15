// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class IntakeDropper extends SubsystemBase {
  
    Servo servo = new Servo(0);
    public static final double HOLD = 1;
    public static final double DROP = 0;

    public void disabledInit () {
      servo.set(HOLD);
    }

  /** Creates a new IntakeDropper. */
  public IntakeDropper() {
    servo.set(HOLD);
  }

  @Override
  public void periodic() {

  }
    // This method will be called once per scheduler run
    
    public Command drop() {
      return Commands.run(() ->servo.set(DROP), this);
    }

    public Command hold() {
      return Commands.run(() -> servo.set(HOLD), this);
    }
}
