package frc.robot.Commands;

import frc.robot.Commands.POICommands;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveDrive;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

public class POICommands {
        SwerveDrive driveSubsystem;

        public POICommands(SwerveDrive swerveDrive) {
                driveSubsystem = swerveDrive;
        }

        PathConstraints constraints = new PathConstraints(3, 2, Degrees.of(360).in(Radians), Degrees.of(720).in(Radians));  // The constraints for this
                                                                                             // path.
        // PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); //
        // You can also use unlimited constraints, only limited by motor torque and
        // nominal battery voltage

        // Create the path using the waypoints created above
        public Command pathToCoralA() throws FileVersionException, IOException, ParseException {
                Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
                                PathPlannerPath.fromPathFile("coral_a"),
                                constraints).finallyDo(() -> driveSubsystem.stop());
                pathfindingCommand.setName("Coral A");
                return pathfindingCommand;
        }

        public Command pathToCoralB() throws FileVersionException, IOException, ParseException {
                Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
                                PathPlannerPath.fromPathFile("coral_b"),
                                constraints).finallyDo(() -> driveSubsystem.stop());
                pathfindingCommand.setName("Coral B");
                return pathfindingCommand;
        }

        public Command pathToCoralC() throws FileVersionException, IOException, ParseException {
                Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
                                PathPlannerPath.fromPathFile("coral_c"),
                                constraints).finallyDo(() -> driveSubsystem.stop());
                pathfindingCommand.setName("Coral C");
                return pathfindingCommand;
        }

        public Command pathToCoralD() throws FileVersionException, IOException, ParseException {
                Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
                                PathPlannerPath.fromPathFile("coral_d"),
                                constraints).finallyDo(() -> driveSubsystem.stop());
                pathfindingCommand.setName("Coral D");
                return pathfindingCommand;
        }

        public Command pathToCoralE() throws FileVersionException, IOException, ParseException {
                Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
                                PathPlannerPath.fromPathFile("coral_e"),
                                constraints).finallyDo(() -> driveSubsystem.stop());
                pathfindingCommand.setName("Coral E");
                return pathfindingCommand;
        }

        public Command pathToCoralF() throws FileVersionException, IOException, ParseException {
                Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
                                PathPlannerPath.fromPathFile("coral_f"),
                                constraints).finallyDo(() -> driveSubsystem.stop());
                pathfindingCommand.setName("Coral F");
                return pathfindingCommand;
        }

        public Command pathToCoralG() throws FileVersionException, IOException, ParseException {
                Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
                                PathPlannerPath.fromPathFile("coral_g"),
                                constraints).finallyDo(() -> driveSubsystem.stop());
                pathfindingCommand.setName("Coral G");
                return pathfindingCommand;
        }

        public Command pathToCoralH() throws FileVersionException, IOException, ParseException {
                Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
                                PathPlannerPath.fromPathFile("coral_h"),
                                constraints).finallyDo(() -> driveSubsystem.stop());
                pathfindingCommand.setName("Coral H");
                return pathfindingCommand;
        }

        public Command pathToCoralI() throws FileVersionException, IOException, ParseException {
                Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
                                PathPlannerPath.fromPathFile("coral_i"),
                                constraints).finallyDo(() -> driveSubsystem.stop());
                pathfindingCommand.setName("Coral I");
                return pathfindingCommand;
        }

        public Command pathToCoralJ() throws FileVersionException, IOException, ParseException {
                Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
                                PathPlannerPath.fromPathFile("coral_j"),
                                constraints).finallyDo(() -> driveSubsystem.stop());
                pathfindingCommand.setName("Coral J");
                return pathfindingCommand;
        }

        public Command pathToCoralK() throws FileVersionException, IOException, ParseException {
                Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
                                PathPlannerPath.fromPathFile("coral_k"),
                                constraints).finallyDo(() -> driveSubsystem.stop());
                pathfindingCommand.setName("Coral K");
                return pathfindingCommand;
        }

        public Command pathToCoralL() throws FileVersionException, IOException, ParseException {
                Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
                                PathPlannerPath.fromPathFile("coral_l"),
                                constraints).finallyDo(() -> driveSubsystem.stop());
                pathfindingCommand.setName("Coral L");
                return pathfindingCommand;
        }

        public Command pathToAlgaeAB() throws FileVersionException, IOException, ParseException {
                Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
                                PathPlannerPath.fromPathFile("algae_ab"),
                                constraints).finallyDo(() -> driveSubsystem.stop());
                pathfindingCommand.setName("Algae AB");
                return pathfindingCommand;
        }

        public Command pathToAlgaeCD() throws FileVersionException, IOException, ParseException {
                Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
                                PathPlannerPath.fromPathFile("algae_cd"),
                                constraints).finallyDo(() -> driveSubsystem.stop());
                pathfindingCommand.setName("Algae CD");
                return pathfindingCommand;
        }

        public Command pathToAlgaeEF() throws FileVersionException, IOException, ParseException {
                Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
                                PathPlannerPath.fromPathFile("algae_ef"),
                                constraints).finallyDo(() -> driveSubsystem.stop());
                pathfindingCommand.setName("Algae EF");
                return pathfindingCommand;
        }

        public Command pathToAlgaeGH() throws FileVersionException, IOException, ParseException {
                Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
                                PathPlannerPath.fromPathFile("algae_gh"),
                                constraints).finallyDo(() -> driveSubsystem.stop());
                pathfindingCommand.setName("Algae GH");
                return pathfindingCommand;
        }

        public Command pathToAlgaeIJ() throws FileVersionException, IOException, ParseException {
                Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
                                PathPlannerPath.fromPathFile("algae_ij"),
                                constraints).finallyDo(() -> driveSubsystem.stop());
                pathfindingCommand.setName("Algae IJ");
                return pathfindingCommand;
        }

        public Command pathToAlgaeKL() throws FileVersionException, IOException, ParseException {
                Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
                                PathPlannerPath.fromPathFile("algae_kl"),
                                constraints).finallyDo(() -> driveSubsystem.stop());
                pathfindingCommand.setName("Algae KL");
                return pathfindingCommand;
        }

        public Command pathToRightIntake() throws FileVersionException, IOException, ParseException {
                Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
                                PathPlannerPath.fromPathFile("intake_right"),
                                constraints).finallyDo(() -> driveSubsystem.stop());
                pathfindingCommand.setName("intake right");
                return pathfindingCommand;
        }

        public Command pathToLeftIntake() throws FileVersionException, IOException, ParseException {
                Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
                                PathPlannerPath.fromPathFile("Intake_left"),
                                constraints).finallyDo(() -> driveSubsystem.stop());
                pathfindingCommand.setName("intake left");
                return pathfindingCommand;
        }
}