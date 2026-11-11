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
import com.qualcomm.robotcore.hardware.DigitalChannelImpl;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;
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
import org.firstinspires.ftc.teamcode.Biobuzz.Wrapers.CrServoPlus;
import org.firstinspires.ftc.teamcode.Biobuzz.Wrapers.MonoChromeGlobalShutterCamera;
import org.firstinspires.ftc.teamcode.Biobuzz.Wrapers.ServoPlus;

import java.util.List;

@Config
public class RobotInitializers {
    public static List<LynxModule> hubs;

    public static Telemetry Dashtelemetry;
    public static IMU imu;
    public static DcMotorControllerEx ControlHubMotors;
    public static ServoControllerEx ControlHubServos, ServoHub1, ServoHub2;
    public static DigitalChannelController ControlHubDigital;

    public static AnalogInputController ControlHubAnalog;
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

        RobotLog.i("Motors initialized");
        ControlHubServos = hm.get(ServoControllerEx.class, "Control Hub");

        RobotLog.i("Servos (chub ehub) initialized");
        ControlHubDigital = hm.get(DigitalChannelController.class,"Control Hub");

        RobotLog.i("Digital initialized");
        ControlHubAnalog = hm.get(AnalogInputController.class,"Control Hub");

        RobotLog.i("Analog initialized");

        ServoHub1 = hm.get(ServoControllerEx.class,"Servo Hub 1");
        ServoHub2 = hm.get(ServoControllerEx.class,"Servo Hub 3");


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
        InitializeStorage();
        InitializeTurret();
        InitializeShooter();
        ExtendedMaths.InitializeMathematics();

    }


    public static void InitializeChassis(){

        SwerveKinematics.backRightModule = new SwervePod(new CrServoPlus(ServoHub2,4 , Servo.Direction.REVERSE),
                new CRServoImplEx(ServoHub1,1, ServoConfigurationType.getStandardServoType()),
                new AnalogInput(ControlHubAnalog,0),true);

        SwerveKinematics.backRightModule.SetEncoderOffset(-100);
        SwerveKinematics.backRightModule.bigCoef = new PIDCoefficients(-0.012191, -0.0004893, 0);
        SwerveKinematics.backRightModule.smallCoef = new PIDCoefficients(-0.049650, -0.0005559, 0);
        SwerveKinematics.backRightModule.controller.setPDFCoefficients(-0.012191, -0.0004893, 0);


        SwerveKinematics.frontRightModule = new SwervePod(new CrServoPlus(ServoHub2,5, Servo.Direction.FORWARD),
                new CRServoImplEx(ServoHub1,0, ServoConfigurationType.getStandardServoType()),
                new AnalogInput(ControlHubAnalog,1),true);

        SwerveKinematics.frontRightModule.SetEncoderOffset(155);
        SwerveKinematics.frontRightModule.bigCoef = new PIDCoefficients(-0.015457, -0.0004329, 0);
        SwerveKinematics.frontRightModule.smallCoef = new PIDCoefficients(-0.031216, -0.0001447, 0);
        SwerveKinematics.frontRightModule.controller.setPDFCoefficients(-0.015457, -0.0004329, 0);

        SwerveKinematics.backLeftModule = new SwervePod(new CrServoPlus(ServoHub2,3, Servo.Direction.FORWARD),
                new CRServoImplEx(ServoHub2,0, ServoConfigurationType.getStandardServoType()),
                new AnalogInput(ControlHubAnalog,3),true);

        SwerveKinematics.backLeftModule.SetEncoderOffset(167);
        SwerveKinematics.backLeftModule.bigCoef = new PIDCoefficients(-0.018017,-0.0007337,0);
        SwerveKinematics.backLeftModule.smallCoef = new PIDCoefficients(-0.025105,-0.0007741,0);
        SwerveKinematics.backLeftModule.controller.setPDFCoefficients(-0.018017,-0.0007337,0);


        SwerveKinematics.frontLeftModule = new SwervePod(new CrServoPlus(ServoHub2,2, Servo.Direction.FORWARD),
                new CRServoImplEx(ServoHub2,1, ServoConfigurationType.getStandardServoType()),
                new AnalogInput(ControlHubAnalog,2),true);

        SwerveKinematics.frontLeftModule.SetEncoderOffset(-146);
        SwerveKinematics.frontLeftModule.bigCoef = new PIDCoefficients(-0.010499, -0.0004717, 0);
        SwerveKinematics.frontLeftModule.smallCoef = new PIDCoefficients(-0.028317, -0.0004998, 0);
        SwerveKinematics.frontLeftModule.controller.setPDFCoefficients(-0.010499, -0.0004717, 0);
    }


    public static void InitializeStorage(){
        Storage.motorfata = new CachedMotor(ControlHubMotors, 0, DcMotorSimple.Direction.FORWARD);
        Storage.motorspate = new CachedMotor(ControlHubMotors, 1, DcMotorSimple.Direction.FORWARD);
        Storage.ramp = new ServoPlus(ServoHub1, 4);
        Storage.upPos = 50  ;
        Storage.downPos = 270;
        Storage.ramp.setAngle(Storage.upPos);

        Storage.bb1 = new DigitalChannelImpl(ControlHubDigital, 1);
        Storage.bb2 = new DigitalChannelImpl(ControlHubDigital, 5);
        Storage.bb3 = new DigitalChannelImpl(ControlHubDigital, 3);
        Storage.bb4 = new DigitalChannelImpl(ControlHubDigital, 7);
    }

    public static void InitializeTurret(){
        Turret.mk21 = new ServoImplEx(ServoHub1, 2,ServoConfigurationType.getStandardServoType());
        Turret.mk22 = new ServoImplEx(ServoHub1, 5,ServoConfigurationType.getStandardServoType());
    }

    public static void InitializeShooter(){
        Shooter.Launcher1 = new CachedMotor(ControlHubMotors, 3, DcMotorSimple.Direction.REVERSE);
        Shooter.Launcher2 = new CachedMotor(ControlHubMotors, 2, DcMotorSimple.Direction.FORWARD);
        Shooter.FlyWheelEncoder = new CachedMotor(ControlHubMotors, 2, DcMotorSimple.Direction.REVERSE);
        Shooter.FlyWheelEncoder.ActivateEncoder(28);

        Shooter.Pitch = new ServoPlus(ServoHub1, 3, Servo.Direction.FORWARD);
    }

    //Intake CHM0 --
    //Storage CHM1 FORWARD
    //Flywheel2 CHM2 FORWARD
    //Flywheel1 CHM3 REVERSE

    //Pitch SH1 3
    //Turret1 SH1 2
    //Turret2 SH1 5
    //
    //Ramp SH1 4 : 15 up : 275 down


}
