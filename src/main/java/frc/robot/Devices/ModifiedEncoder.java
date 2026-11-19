package frc.robot.Devices;

import org.wpilib.hardware.rotation.DutyCycleEncoder;
import org.wpilib.hardware.rotation.Encoder;

/*
 * Class: Modified Encoder
 * Description: Creates the modified encoder and functions we 
 *              can invoke to use it.
 * Author: Unkown
 */

public class ModifiedEncoder {

    private DutyCycleEncoder dutyCycleEncoder;
    private Encoder encoder;
    

    public ModifiedEncoder(int CANID) {
        dutyCycleEncoder = new DutyCycleEncoder(CANID);
    }

    public double get(){
        return dutyCycleEncoder != null ? dutyCycleEncoder.get() : 0;
    }

    public void setDistancePerPulse(double distancePerPulse) {
        if (encoder != null){
            encoder.setDistancePerPulse(distancePerPulse);
        }
    }

    public double getAbsolutePosition() {
        return dutyCycleEncoder != null ? (dutyCycleEncoder.get() * 360) : 0;
    }
}