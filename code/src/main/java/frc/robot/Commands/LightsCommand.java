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
    boolean coralSubsystemAtTarget = false;
    ClimberSubsystem m_climber;
    ConstrainedArmSubsystem m_arm;
    Elevator m_elevator;
    LightsSubsystem m_lights;

    /** Creates a new LightsCommand. */
    public LightsCommand(LightsSubsystem lights, ClimberSubsystem climber, ConstrainedArmSubsystem arm,
            Elevator elevator) {
                m_climber = climber;
                m_arm = arm;
                m_elevator = elevator;
                m_lights = lights;
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
        if (m_climber.isClimbing()) { // works
            m_lights.violet();
        } else if (coralSubsystemAtTarget()) { // works
            m_lights.aqua();
        } else if (beam.isBeamBreakTriggered()) { // works
            m_lights.green();
        } else if (!beam.isBeamBreakTriggered() && !coralSubsystemAtTarget() && !m_climber.isClimbing() && !m_lights.isCoopertition()){
            if (DriverStation.getAlliance().get() == Alliance.Blue) {
                m_lights.blue();
            } else {
                m_lights.red();
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

    public boolean coralSubsystemAtTarget() {
            coralSubsystemAtTarget = false;
        if(m_elevator.returnNearState() && m_arm.returnNearState()) {
            coralSubsystemAtTarget = true;
        }
        return coralSubsystemAtTarget;
    }
}
