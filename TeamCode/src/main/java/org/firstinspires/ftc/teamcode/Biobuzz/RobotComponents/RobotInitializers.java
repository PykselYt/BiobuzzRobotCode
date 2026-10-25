package org.firstinspires.ftc.teamcode.Biobuzz.RobotComponents;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogInputController;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.ServoConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Biobuzz.MathHelpers.ExtendedMaths;
import org.firstinspires.ftc.teamcode.Biobuzz.SwerveTest.SwerveKinematics;
import org.firstinspires.ftc.teamcode.Biobuzz.SwerveTest.SwervePod;
import org.firstinspires.ftc.teamcode.Biobuzz.Wrapers.CachedMotor;
import org.firstinspires.ftc.teamcode.Biobuzz.Wrapers.MonoChromeGlobalShutterCamera;
import java.util.List;

@Config
public class RobotInitializers {
    public static List<LynxModule> hubs;

    public static Telemetry Dashtelemetry;
    public static IMU imu;
    public static DcMotorControllerEx ControlHubMotors, ExpansionHubMotors;
    public static ServoControllerEx ControlHubServos, ExpansionHubServos, ServoHub;
    public static DigitalChannelController ControlHubDigital,ExpansionHubDigital;

    public static AnalogInputController ControlHubAnalog,ExpansionHubAnalog;
    public static MonoChromeGlobalShutterCamera camera;
    public static double VOLTAGE = 12;
    public static boolean isDisabled(){
        return !hubs.get(0).isEngaged();
    }
    public static void disable(){
        for(LynxModule l : hubs){
            l.disengage();
        }
    }
    public static void enable(){
        for(LynxModule l : hubs){
            l.engage();
        }
    }
    public static void  enableDashTelemetry(){
        Dashtelemetry = FtcDashboard.getInstance().getTelemetry();
    }
    public static void InitializeHubs(HardwareMap hm){
        InitializeHubs(hm, false);
    }

    public static void InitializeHubs(HardwareMap hm, boolean b) {
        enableDashTelemetry();
        if(hubs != null) return;
        hubs = hm.getAll(LynxModule.class);
//        if(b)
//            disable();

        ControlHubMotors = hm.get(DcMotorControllerEx.class, "Control Hub");
        //ExpansionHubMotors = hm.get(DcMotorControllerEx.class, "Expansion Hub 2");
        RobotLog.i("Motors initialized");
        ControlHubServos = hm.get(ServoControllerEx.class, "Control Hub");
        //ExpansionHubServos = hm.get(ServoControllerEx.class, "Expansion Hub 2");
        RobotLog.i("Servos (chub ehub) initialized");
        ControlHubDigital = hm.get(DigitalChannelController.class,"Control Hub");
        //ExpansionHubDigital = hm.get(DigitalChannelController.class,"Expansion Hub 2");
        RobotLog.i("Digital initialized");
        ControlHubAnalog = hm.get(AnalogInputController.class,"Control Hub");
        //ExpansionHubAnalog = hm.get(AnalogInputController.class,"Expansion Hub 2");
        RobotLog.i("Analog initialized");

        ServoHub = hm.get(ServoControllerEx.class,"Servo Hub 1");

        //IMUBNO085.controller = hm.get(DigitalChannelController.class, "Expansion Hub 1");

        MotorConfigurationType mct;

        for(int i = 0; i < 4; i++){
            mct = ControlHubMotors.getMotorType(i);
            mct.setAchieveableMaxRPMFraction(1);

            ControlHubMotors.setMotorType(i, mct);

            /*mct = ExpansionHubMotors.getMotorType(i);
            mct.setAchieveableMaxRPMFraction(1);

            ExpansionHubMotors.setMotorType(i, mct);*/



        }

//        imu = hm.get(IMUBNO085.class, "extIMU");
        imu = hm.get(IMU.class, "imu");

        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
        )));
//        Orientation o = new Orientation(
//                AxesReference.EXTRINSIC,
//                AxesOrder.ZXY,
//                AngleUnit.DEGREES,
//                0, 0, 0, 0
//        );
//        Robot.imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(o)));

        imu.resetYaw();

        VOLTAGE = hm.getAll(VoltageSensor.class).get(0).getVoltage();

        for(LynxModule l : hubs){
            l.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }
    public static void clearCache(){
        clearCache(true);
    }
    public static long loopTime = 0;
    private static ElapsedTime logFreq = new ElapsedTime();
    public static void clearCache(boolean update){
        for(LynxModule l : hubs){
            l.clearBulkCache();
        }
        if(update) {
            Dashtelemetry.update();
            Dashtelemetry.addData("frequency", String.valueOf(1000.f / (System.currentTimeMillis() - loopTime)));
        }
        if(logFreq.seconds() >= 1) {
            RobotLog.ii("frequency", String.valueOf(1000.f / (System.currentTimeMillis() - loopTime)));
            logFreq.reset();
        }
        loopTime = System.currentTimeMillis();
    }
    public static void InitializeFull(HardwareMap hm){
        InitializeHubs(hm, true);
        Localizer.Initialize(hm);
        InitializeChassis();
        ExtendedMaths.InitializeMathematics();

    }


    public static void InitializeChassis(){

        SwerveKinematics.backRightModule = new SwervePod(new CachedMotor(ControlHubMotors,0 , DcMotorSimple.Direction.REVERSE),
                new CRServoImplEx(ServoHub,3, ServoConfigurationType.getStandardServoType()),
                new AnalogInput(ControlHubAnalog,2),true);

        SwerveKinematics.backRightModule.SetEncoderOffset(-55);
        SwerveKinematics.backRightModule.bigCoef = new PIDCoefficients(-0.0175, -0.00065, 0);
        SwerveKinematics.backRightModule.smallCoef = new PIDCoefficients(-0.0175, -0.00065, 0);
        SwerveKinematics.backRightModule.controller.setPDFCoefficients(-0.0175, -0.00065, 0);


        SwerveKinematics.frontRightModule = new SwervePod(new CachedMotor(ControlHubMotors,1, DcMotorSimple.Direction.REVERSE),
                new CRServoImplEx(ServoHub,4, ServoConfigurationType.getStandardServoType()),
                new AnalogInput(ControlHubAnalog,3),true);

        SwerveKinematics.frontRightModule.SetEncoderOffset(3);
        SwerveKinematics.frontRightModule.bigCoef = new PIDCoefficients(-0.0175, -0.0007, 0);
        SwerveKinematics.frontRightModule.smallCoef = new PIDCoefficients(-0.0175, -0.0007, 0);
        SwerveKinematics.frontRightModule.controller.setPDFCoefficients(-0.0175, -0.0007, 0);

        SwerveKinematics.backLeftModule = new SwervePod(new CachedMotor(ControlHubMotors,2, DcMotorSimple.Direction.FORWARD),
                new CRServoImplEx(ServoHub,1, ServoConfigurationType.getStandardServoType()),
                new AnalogInput(ControlHubAnalog,0),true);

        SwerveKinematics.backLeftModule.SetEncoderOffset(117);
        SwerveKinematics.backLeftModule.bigCoef = new PIDCoefficients(-0.0166,-0.00071,0);
        SwerveKinematics.backLeftModule.smallCoef = new PIDCoefficients(-0.0166,-0.00071,0);
        SwerveKinematics.backLeftModule.controller.setPDFCoefficients(-0.0166, -0.00071, 0);


        SwerveKinematics.frontLeftModule = new SwervePod(new CachedMotor(ControlHubMotors,3, DcMotorSimple.Direction.REVERSE),
                new CRServoImplEx(ServoHub,2, ServoConfigurationType.getStandardServoType()),
                new AnalogInput(ControlHubAnalog,1),true);

        SwerveKinematics.frontLeftModule.SetEncoderOffset(45);
        SwerveKinematics.frontLeftModule.bigCoef = new PIDCoefficients(-0.0175, -0.00065, 0);
        SwerveKinematics.frontLeftModule.smallCoef = new PIDCoefficients(-0.0175, -0.00065, 0);
        SwerveKinematics.frontLeftModule.controller.setPDFCoefficients(-0.0175, -0.00065, 0);
    }


}
