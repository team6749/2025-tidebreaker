// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Localization;
import frc.robot.subsystems.swerve.SwerveDrive;

@Logged
public class RobotContainer {

  SwerveDrive swerveSubsystem;
  Arm arm = new Arm();
  Localization localizationSubsystem;

  XboxController controller = new XboxController(0);

  public RobotContainer() {
    swerveSubsystem = new SwerveDrive();
    localizationSubsystem = new Localization(swerveSubsystem);



    configureBindings();
  }

  private void configureBindings() {
    swerveSubsystem.setDefaultCommand(swerveSubsystem.basicDriveCommand(controller));
    // if(controller.getAButton() || controller.getBButton() || controller.getYButton()) {
    //   runArmCommand();
    // }
    
    //swerveSubsystem.setDefaultCommand(swerveSubsystem.testModuleSpeeds(new SwerveModuleState(MetersPerSecond.of(2),Rotation2d.kZero)));
  }

  public Command getAutonomousCommand() {
    return swerveSubsystem.constantChassisSpeedsCommand(
        new ChassisSpeeds(2, 0, 2));
  }

  // public Command runArmCommand() {
  //   Command command = Commands.run(() -> {
  //     if(controller.getBButton()) {
  //       arm.armCommand(Rotation2d.fromRadians(2 * Math.PI));
  //       System.out.println("l4");
  //     }
  //     else if (controller.getAButton()){
  //       arm.armCommand(Rotation2d.fromRadians(Math.PI));
  //       System.out.println("l3");
  //     }
  //     else if(controller.getYButton()) {
  //       arm.armCommand(Rotation2d.fromRadians(0));
  //       System.out.println("l2");
  //     }
  //   }, arm);
  //   return command; /*Replace the controller buttons with button board(which we probaly will use), and fill in the actual desired angles */
  // }
}
