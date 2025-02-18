package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class Input {
    XboxController controller = new XboxController(0);
    XboxController controller2 = new XboxController(1);
    // PS5Controller controller2 = new PS5Controller(1);
    JoystickButton a = new JoystickButton(controller, 1);
    JoystickButton x = new JoystickButton(controller, 3);
    JoystickButton b = new JoystickButton(controller2, 2);
    JoystickButton y = new JoystickButton(controller2, 4);
    JoystickButton rightBumper = new JoystickButton(controller2, 5);
    JoystickButton leftBumper = new JoystickButton(controller2, 6);
    DoubleSupplier rightTrigger = () -> controller2.getRawAxis(3);
    DoubleSupplier leftTrigger =  () -> controller2.getRawAxis(2);
    JoystickButton start = new JoystickButton(controller, 8);

    public Input() {
       
    }

    public double getLeftX() {
        return controller.getLeftX();
    }

    public double getLeftY() {
        return controller.getLeftY();
    }

    public double getRightX() {
        return controller.getRightX();
    }

    public double getRightY() {
        return controller.getRightY();
    }

    public JoystickButton getA() {
        return a;
    }

    public JoystickButton getX() {
        return x;
    }

    public JoystickButton getB() {
        return b;
    }

    public JoystickButton getY() {
        return y;
    }

    public JoystickButton getRightBumper() {
        return rightBumper;
    }

    public JoystickButton getLeftBumper() {
        return leftBumper;
    }

    public DoubleSupplier getRightTrigger() {
        return rightTrigger;
    }

    public DoubleSupplier getLeftTrigger() {
        return leftTrigger;
    }

    public JoystickButton getStart() {
        return start;
    }

    public boolean isUserControlActive() {
        return  Math.abs(controller.getLeftX()) > 0.1 ||
        Math.abs(controller.getLeftY()) > 0.1 ||
        Math.abs(controller.getRightX()) > 0.1 ||
        Math.abs(controller.getRightY()) > 0.1 ||
        controller.getRightTriggerAxis() > 0.1 ||
        controller.getLeftTriggerAxis() > 0.1;
    }
}
