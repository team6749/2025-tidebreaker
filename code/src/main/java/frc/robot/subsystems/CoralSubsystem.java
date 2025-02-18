// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
@Logged
public class CoralSubsystem extends SubsystemBase {
  Arm armSubsystem = new Arm();
  Elevator elevatorSubsystem = new Elevator();
  /** Creates a new coralSubsystem. */
  public CoralSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
    public Command goToCoralLayer(Distance goalHeight, Angle goalAngle) {
    return Commands.runEnd(() -> {SmartDashboard.putNumber("elevator Subsystem calling", goalHeight.in(Meters));
      armSubsystem.goToPositionArm(armSubsystem.canExtend(goalAngle, elevatorSubsystem.inElevatorDangerZone()));
      elevatorSubsystem.goToPositionCommand(goalHeight);
      SmartDashboard.putBoolean("arm in danger", armSubsystem.inArmDangerZone());
      SmartDashboard.putNumber("coralLayer input", elevatorSubsystem.canRaise(goalHeight, false).in(Meters));
    }, () -> {}, armSubsystem,elevatorSubsystem);
  }
}//elevatorSubsystem.canRaise(goalHeight, armSubsystem.inArmDangerZone())
