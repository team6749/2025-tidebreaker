package frc.robot;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.wpilibj2.command.Command;

public class POICommands {
PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.
// PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); // You can also use unlimited constraints, only limited by motor torque and nominal battery voltage

// Create the path using the waypoints created above
public Command pathToCoralA() throws FileVersionException, IOException, ParseException {
        Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
                PathPlannerPath.fromPathFile("Example Path"),
                constraints);
        return pathfindingCommand;
        }
public Command pathToCoralB() throws FileVersionException, IOException, ParseException {
        Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
                PathPlannerPath.fromPathFile("Example Path"),
                constraints);
        return pathfindingCommand;
        }

public Command pathToCoralC() throws FileVersionException, IOException, ParseException {
        Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
                PathPlannerPath.fromPathFile("Example Path"),
                constraints);
        return pathfindingCommand;
        }
public Command pathToCoralD() throws FileVersionException, IOException, ParseException {
        Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
                PathPlannerPath.fromPathFile("Example Path"),
                constraints);
        return pathfindingCommand;
        }
public Command pathToCoralE() throws FileVersionException, IOException, ParseException {
        Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
                PathPlannerPath.fromPathFile("Example Path"),
                constraints);
        return pathfindingCommand;
        }

public Command pathToCoralF() throws FileVersionException, IOException, ParseException {
        Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
                PathPlannerPath.fromPathFile("Example Path"),
                constraints);
        return pathfindingCommand;
        }
public Command pathToCoralG() throws FileVersionException, IOException, ParseException {
        Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
                PathPlannerPath.fromPathFile("Example Path"),
                constraints);
        return pathfindingCommand;
        }
public Command pathToCoralH() throws FileVersionException, IOException, ParseException {
        Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
                PathPlannerPath.fromPathFile("Example Path"),
                constraints);
        return pathfindingCommand;
        }

public Command pathToAlgaeAB() throws FileVersionException, IOException, ParseException {
        Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
                PathPlannerPath.fromPathFile("Example Path"),
                constraints);
        return pathfindingCommand;
        }
public Command pathToAlgaeCD() throws FileVersionException, IOException, ParseException {
        Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
                PathPlannerPath.fromPathFile("Example Path"),
                constraints);
        return pathfindingCommand;
        }
public Command pathToAlgaeEF() throws FileVersionException, IOException, ParseException {
        Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
                PathPlannerPath.fromPathFile("Example Path"),
                constraints);
        return pathfindingCommand;
        }

public Command pathToAlgaeGH() throws FileVersionException, IOException, ParseException {
        Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
                PathPlannerPath.fromPathFile("Example Path"),
                constraints);
        return pathfindingCommand;
        }
public Command pathToAlgaeIJ() throws FileVersionException, IOException, ParseException {
        Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
                PathPlannerPath.fromPathFile("Example Path"),
                constraints);
        return pathfindingCommand;
        }

public Command pathToAlgaeKL() throws FileVersionException, IOException, ParseException {
        Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
                PathPlannerPath.fromPathFile("Example Path"),
                constraints);
        return pathfindingCommand;
        }
}