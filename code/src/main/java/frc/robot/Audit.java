package frc.robot;

import java.io.IOException;
import java.nio.file.Files;
import java.util.Arrays;
import java.util.stream.Collectors;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Audit {

    private String branch;
    private String head;
    private String machine;
    private String status;
    private boolean isCleanDeploy;

    Audit() {
        // Get Audit data
        var deploy = Filesystem.getDeployDirectory();
        try {
            branch = Files.readString(deploy.toPath().resolve("audit/branch.txt")).trim();
            head = Files.readString(deploy.toPath().resolve("audit/HEAD.txt")).trim();
            machine = Files.readString(deploy.toPath().resolve("audit/machine.txt")).trim();
            status = Files.readString(deploy.toPath().resolve("audit/status.txt")).trim();
            isCleanDeploy = branch.equals("main") && status.equals("");
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void printDeployInformation() {
        System.out
                .println("\n----- Deploy Information -----\n" + branch + " " + machine + " " + head + "\n" + status);
    }

    public void updateSmartDashboard() {
        SmartDashboard.putBoolean("Deploy/isClean", isCleanDeploy);
        SmartDashboard.putString("Deploy/Branch", branch);
        // Converted to csv because smart dashboard does not render multi-line strings
        SmartDashboard.putString("Deploy/status", Arrays.stream(status.split("\n"))
                .collect(Collectors.joining(",")));
        SmartDashboard.putString("Deploy/HEAD", head);
        SmartDashboard.putString("Deploy/machine", machine);
    }

}
