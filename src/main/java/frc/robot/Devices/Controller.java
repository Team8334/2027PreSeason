package frc.robot.Devices;

import org.wpilib.driverstation.GenericHID;
import org.wpilib.driverstation.HIDDevice;
import org.wpilib.driverstation.XboxController;

import java.util.HashMap;

 /* Class: Controller
 * Description: Makes the object of controller to be used elsewhere.
 *              Also has debounce code for the buttons.
 * Author: Trevor
 */

public class Controller implements HIDDevice{

    private final XboxController controller;

    private final HashMap<Integer, Boolean> debounceButtons =
            new HashMap<>();

    public Controller(int port) {
        controller = new XboxController(port);
    }

    public GenericHID getHID() {
        return controller.getHID();
    }

    public boolean getDebouncedButton(int button) {
        if (!debounceButtons.containsKey(button)) {
            debounceButtons.put(button, false);
        }
        if (getHID().getRawButton(button) && debounceButtons.get(button))
        {
            debounceButtons.put(button, false);
            return false;
        }
        else if (getHID().getRawButtonPressed(button))
        {
            debounceButtons.put(button, true);
            return true;
        }
        else
        {
            return debounceButtons.get(button);
        }
    }

    public boolean getDebouncedButton(Button button) {
        if (!debounceButtons.containsKey(button.value)) {
            debounceButtons.put(button.value, false);
        }
        
        if (getHID().getRawButton(button.value) && debounceButtons.get(button.value))
        {
            debounceButtons.put(button.value, false);
            return false;
        }
        else if (getHID().getRawButtonPressed(button.value))
        {
            debounceButtons.put(button.value, true);
            return true;
        }
        else
        {
            return debounceButtons.get(button.value);
        }
    }
     
    public boolean isOperational() {
        return getHID().isConnected();
    }

    public String getName() {
        return "Controller";
    }
}