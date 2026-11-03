package org.firstinspires.ftc.teamcode.Biobuzz.Wrapers;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.configuration.ServoFlavor;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.ServoType;


@ServoType(flavor = ServoFlavor.CUSTOM)
@DeviceProperties(name = "CRServoPlus", xmlTag = "CrservoPlus")
public class CrServoPlus extends ServoImpl implements Servo, HardwareDevice {

    public CrServoPlus(ServoController controller, int portNumber, Direction direction) {
        super(controller, portNumber, direction);
    }
    public CrServoPlus(ServoController controller, int portNumber) {
        super(controller, portNumber);
    }
    public CrServoPlus(Servo s){
        super(s.getController(), s.getPortNumber(), s.getDirection());
    }

    public synchronized void setPower(double val){
        setPosition((val+1.0)/2.0);
    }
}
