package org.firstinspires.ftc.teamcode.Biobuzz.SwerveTest;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Biobuzz.RobotComponents.RobotInitializers;
import org.firstinspires.ftc.teamcode.Biobuzz.SwerveTest.EfficientPDFController;
import org.firstinspires.ftc.teamcode.Biobuzz.Wrapers.CachedMotor;

public class SwervePod {

    public CachedMotor motor;
    public CRServoImplEx servo;
    public AnalogInput encoder;

    public double P = 0,D = 0,F = 0;

    public double targetAngle;
    public double lastServoPower;
    public double lastMotorPower;
    private double CalcAngle;
    public double motorPower;

    public EfficientPDFController controller;
    public PIDCoefficients bigCoef = new PIDCoefficients(0,0,0), smallCoef=new PIDCoefficients(0,0,0);
    boolean usingSmallCoef = false;

    private double Offset = 0;
    private final boolean reverseServo;

    public SwervePod(CachedMotor motorInstance, CRServoImplEx CRServoInstance, AnalogInput analogInstance, boolean reverseServo) {
        this.servo = CRServoInstance;
        this.encoder = analogInstance;
        this.motor = motorInstance;
        this.reverseServo = reverseServo;


        this.servo.setPwmRange(new PwmControl.PwmRange(1050, 1950));
        controller = new EfficientPDFController(bigCoef.p, bigCoef.i, bigCoef.d);
        controller.setTarget(0);
    }


    public void set(double angle, double power) {
        double current = getAngle();

        double normalError = error(angle, current);
        double reverseAngle = normalize(angle + 180);
        double reverseError = error(reverseAngle, current);

        RobotLog.ii("intrat in set", String.valueOf(power));

        if (Math.abs(reverseError) < Math.abs(normalError)) {
            targetAngle = reverseAngle;
            motorPower = -power;
        } else {
            targetAngle = normalize(angle);
            motorPower = power;
        }
    }
    public void update(){
        update(1);
    }
    public void update(double wheelSlow) {

        CalculategetAngle();


        double error = getError();

        if(error > 4 ){
            usingSmallCoef = true;
            controller.setPDFCoefficients(smallCoef.p, smallCoef.i, smallCoef.d);
        }
        if(error <= 4 ){
            usingSmallCoef = false;
            controller.setPDFCoefficients(bigCoef.p, bigCoef.i, bigCoef.d);
        }

        double servoPower = controller.calculate(error);
        if(Math.abs(servoPower - lastServoPower) > 0.01){
            servo.setPower(servoPower);
            lastServoPower = servoPower;
        }

        double shouldMotorPower = motorPower * wheelSlow;
        if(Math.abs(shouldMotorPower - lastMotorPower) > 0.01){
            motor.setPower(shouldMotorPower);
            lastMotorPower = shouldMotorPower;
        }



    }

    public double getError(){
        return error(targetAngle, getAngle());
    }

    public double getAngle() {
        return CalcAngle;
    }

    public void CalculategetAngle() {
        CalcAngle = reverseServo?360-normalize(encoder.getVoltage() / 3.3 * 360 - Offset):normalize(encoder.getVoltage() / 3.3 * 360 - Offset);
    }

    public double error(double target, double current) {
        double error = normalize(target) - normalize(current);

        if (error > 180)
            error -= 360;

        if (error < -180)
            error += 360;

        return error;
    }

    private double normalize(double angle) {
        angle %= 360;

        if (angle < 0)
            angle += 360;

        return angle;
    }

    public void SetEncoderOffset(double Offset){
        this.Offset = Offset;
    }

    public void stop() {
        motor.setPower(0);
        servo.setPower(0);
    }
}