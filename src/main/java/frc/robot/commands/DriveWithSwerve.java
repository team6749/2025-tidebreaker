// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;

public class DriveWithSwerve extends Command {
    public XboxController controller; 
    public ADIS16470_IMU gyro;
    public double desiredX = Math.cos(Math.atan(controller.getLeftY()/controller.getLeftX()) - gyro.getAngle()) * Constants.SwerveConstants.bodyHeading; //field oriented
    public double desiredY = Math.sin(Math.atan(controller.getLeftY()/controller.getLeftX()) - gyro.getAngle()) * Constants.SwerveConstants.bodyHeading;
    public double desiredRotation = controller.getRightX() * 360;
    public double desiredSpeeds = ControllerCurve(deadZone(Math.sqrt(Math.pow(desiredX, 2) + Math.pow(desiredY, 2))));
    public ChassisSpeeds desiredChassisSpeeds;
    public SwerveDrive SwerveChassis;
  /** Creates a new DriveWithSwerve. */
  public DriveWithSwerve(SwerveDrive subSystem, int controllerPort) {
    SwerveChassis = subSystem;
    addRequirements(subSystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    desiredChassisSpeeds = new ChassisSpeeds(desiredY, desiredX, desiredRotation);
    SwerveChassis.setModuleStates(SwerveChassis.ConvertToSwerveModuleState(desiredChassisSpeeds));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
    public double deadZone(double input) {
    if(input < Math.abs(Constants.DrivingConstants.deadZone)) {
        input = 0;
    }
    return input;
  }

  public double ControllerCurve(double input) {
    return Math.pow(input, 3);
  }

}
