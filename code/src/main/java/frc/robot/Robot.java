// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.logging.FileBackend;
import edu.wpi.first.epilogue.logging.NTEpilogueBackend;
import edu.wpi.first.epilogue.logging.errors.ErrorHandler;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The main robot class for the FRC robot, managing all robot states and command execution.
 * 
 * <p>This class extends {@link TimedRobot} and handles:
 * <ul>
 *   <li>Command-based scheduling using {@link CommandScheduler}</li>
 *   <li>Logging of robot events and errors using {@link DataLogManager} and {@link Epilogue}</li>
 *   <li>Initialization and management of the {@link RobotContainer}</li>
 *   <li>Match timing updates via {@link Timer#getMatchTime()}</li>
 *   <li>Handling transitions between different robot modes (autonomous, teleop, disabled, test)</li>
 * </ul>
 * 
 * This class is responsible for coordinating subsystems, executing commands, and ensuring 
 * logging is properly configured for debugging and performance analysis.
 */
@Logged
public class Robot extends TimedRobot {

  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public Robot() {
    // !!! IMPORTANT !!!!
    // ensure that all real robots have a usb drive for log data. Otherwise it will
    // be written to the roborio's SD card/internal storage, which has limited write endurance
    DataLogManager.start();
    Epilogue.configure(config -> {
      if (isSimulation() || true) {
        // If running in simulation, then we'd want to re-throw any errors that
        // occur so we can debug and fix them!
        config.errorHandler = ErrorHandler.crashOnError();
        // Data passed into network tables is logged by the DataLogManager
        config.backend = new NTEpilogueBackend(NetworkTableInstance.getDefault());
      }
      else{
        config.errorHandler = ErrorHandler.printErrorMessages();
        // On the real robot only log to disk, to avoid too much network bandwidth
        // Dashboard values are sent separately
        config.backend = new FileBackend(DataLogManager.getLog());}
    });
    Epilogue.bind(this);

    // Configure logging for the command scheduler
    CommandScheduler.getInstance()
        .onCommandInitialize(
            command -> {
              Shuffleboard.addEventMarker(
                  "Command initialized", command.getName(), EventImportance.kNormal);
              DataLogManager.log("Command initialized: " + command.getName());
            });
    CommandScheduler.getInstance()
        .onCommandInterrupt(
            command -> {

              Shuffleboard.addEventMarker(
                  "Command interrupted", command.getName(), EventImportance.kNormal);
              DataLogManager.log("Command interrupted: " + command.getName());
            });
    CommandScheduler.getInstance()
        .onCommandFinish(
            command -> {
              Shuffleboard.addEventMarker(
                  "Command finished", command.getName(), EventImportance.kNormal);
              DataLogManager.log("Command finished: " + command.getName());
            });

    // Initialize Robot
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    // Log Match Timer to Dashboard
    SmartDashboard.putNumber("Match Timer", Timer.getMatchTime());

  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }
}
