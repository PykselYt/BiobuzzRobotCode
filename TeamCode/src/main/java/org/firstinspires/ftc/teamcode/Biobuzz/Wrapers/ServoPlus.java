package org.firstinspires.ftc.teamcode.Biobuzz.Wrapers;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.configuration.ServoFlavor;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.ServoType;


@ServoType(flavor = ServoFlavor.CUSTOM)
@DeviceProperties(name = "ServoPlus", xmlTag = "servoPlus")
public class ServoPlus extends ServoImpl implements Servo, HardwareDevice {
    public ServoPlus(ServoController controller, int portNumber, Direction direction) {
        super(controller, portNumber, direction);
    }
    public ServoPlus(ServoController controller, int portNumber) {
        super(controller, portNumber);
    }
    public ServoPlus(Servo s){
        super(s.getController(), s.getPortNumber(), s.getDirection());
    }
    public double MaxAngle = 355;

    synchronized public void setMaxAngle(double angle){
        MaxAngle = angle;
    }
    private double thisAngle = 0;
    synchronized public void setAngle(double angle){
//        if(!Robot.hubs.get(0).isEngaged()) return;
        if(isEqualToAngle(angle)) return;
        thisAngle = angle;
        setPosition(angle / MaxAngle);
    }
    public double getAngle(){
        if(encoder == null)
            return thisAngle;
        else return encoder.getVoltage() / encoder.getMaxVoltage() * 360.f;
    }
    public boolean isEqualToAngle(double angle){
        return Math.abs(angle - getAngle()) < 0.1;
    }
    // -------------------- CR Implementation --------------------

    public void setEncoder(AnalogInput ai){
        encoder = ai;
    }

    private AnalogInput encoder = null;

}