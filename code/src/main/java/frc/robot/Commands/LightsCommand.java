// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ConstrainedArmSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LightsSubsystem;

public class LightsCommand extends Command {
    Elevator _elevator;
    ConstrainedArmSubsystem _arm;
    LightsSubsystem _lights;

    /** Creates a new LightsCommand. */
    public LightsCommand(LightsSubsystem lights, ConstrainedArmSubsystem arm,
            Elevator elevator) {
        _lights = lights;
        _arm = arm;
        _elevator = elevator;
        addRequirements(lights);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    if (_elevator.getIsCoralLimitSwitchActivated()) { // works
            _lights.blueViolet();
        } else if (_arm.isAtTarget() && _elevator.isAtTarget()) { // works
            _lights.green();
        }
        else if (!_elevator.getIsCoralLimitSwitchActivated() && !(_arm.isAtTarget() && _elevator.isAtTarget())){
            if (DriverStation.getAlliance().get() == Alliance.Blue) {
                _lights.blue();
            } else {
                _lights.red();
            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
