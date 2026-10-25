package org.firstinspires.ftc.teamcode.Biobuzz.Wrapers;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.ServoConfigurationType;

public class ServoWithPWMChange extends CRServoImplEx implements CRServo, HardwareDevice {
    public ServoWithPWMChange(ServoControllerEx controller, int portNumber, @NonNull ServoConfigurationType servoType) {
        super(controller, portNumber, servoType);
        this.setPwmRange(new PwmRange(500,2500));
    }
}
