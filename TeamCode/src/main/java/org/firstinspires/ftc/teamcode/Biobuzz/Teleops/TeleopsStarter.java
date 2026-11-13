package org.firstinspires.ftc.teamcode.Biobuzz.Teleops;

import static org.firstinspires.ftc.teamcode.Biobuzz.RobotComponents.Shooter.Pitch;
import static org.firstinspires.ftc.teamcode.Biobuzz.TeleopLogic.MainHandler.PinpointDown;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Biobuzz.RobotComponents.Chassis;
import org.firstinspires.ftc.teamcode.Biobuzz.RobotComponents.Localizer;
import org.firstinspires.ftc.teamcode.Biobuzz.RobotComponents.RobotInitializers;
import org.firstinspires.ftc.teamcode.Biobuzz.RobotComponents.Shooter;
import org.firstinspires.ftc.teamcode.Biobuzz.RobotComponents.ShooterCalculator;
import org.firstinspires.ftc.teamcode.Biobuzz.RobotComponents.Storage;
import org.firstinspires.ftc.teamcode.Biobuzz.RobotComponents.Turret;
import org.firstinspires.ftc.teamcode.Biobuzz.TeleopLogic.MainHandler;

@Config
public class TeleopsStarter {

    public static TEAM team = TEAM.RED;

    public static Gamepad gm1;
    public static Gamepad gm2;

    public static boolean ActivateTelemetry = true;
    public static double rotMult = 1, speedMult = 1;
    public static boolean reverse = false;
    public static double AngleOffsetForDriving = 180;
    public MainHandler mainHandler;
    public static Storage storage;
    public static Shooter shooter;
    public static Turret turret;
    public static SparkFunOTOS.Pose2D resetPosition = new SparkFunOTOS.Pose2D(0,0,0);


    public static boolean DrivingFieldCentric = true;


    public static Gamepad prevgm1,prevgm2;


    public static boolean prepareShoot;

    public TeleopsStarter(HardwareMap hardwareMap, TEAM color){

        TeleopsStarter.gm1 = new Gamepad();
        TeleopsStarter.gm2 = new Gamepad();


        prevgm1 = new Gamepad();
        prevgm2 = new Gamepad();

        RobotInitializers.InitializeFull(hardwareMap);
        RobotInitializers.disable();

        storage = new Storage();
        turret = new Turret();

        shooter = new Shooter();


        ShooterCalculator.shouldCompensateHood = true;

        ShooterCalculator.TurretOffsetStatic = 0;

        storage.lockPuta();


        mainHandler = new MainHandler();
        team = color;
        /*
        TeleopsStarter.speedMult = TeleopsStarter.rotMult = 1;
        storage.lockPuta();
        TeleopsStarter.prepareShoot = false;
        Pitch.setAngle(Storage.blockPos);
        shooter.setPowerEquilibrated(0.75);
        */

    }



    public void InitUpdate(){
        RobotInitializers.clearCache();
        Chassis.usedTrajectory=Chassis.trajectoryStates.USENOTHING;
        Chassis.kinematics.resetHoldHeading();

        //ShooterCalculator.shouldCompensateHood = true;
        //shooter.UpdateTurretAndPitch();

        if(gm1.cross){
            DrivingFieldCentric = true;
        }
        prevgm1.copy(gm1);
        prevgm2.copy(gm2);
        Localizer.Update();
    }


    public static double setRPM = 1000;
    public static double setAngle = 67;

    public void update() {

        RobotInitializers.clearCache(ActivateTelemetry);

        Chassis.drive(DrivingFieldCentric, -gm1.left_stick_x * speedMult, -gm1.left_stick_y * speedMult, (gm1.left_trigger - gm1.right_trigger) * rotMult);

        Chassis.Update();


        mainHandler.update();


        shooter.LauncherSpeedController.setTargetPosition(setRPM);


        Localizer.Update();
        if(prepareShoot){
            turret.updateTurret(Localizer.getCurrentPosition());
            //shooter.UpdateTurretAndPitch();
            shooter.updateFlyWheelSpeed();
        }

        if(!Storage.putaBlock){
            Pitch.setAngle(shooter.FormulaCalculator.NeededServoPosForPitchGiven(setAngle));
            //Pitch.setAngle(150);
        }else{
            Pitch.setAngle(Storage.blockPos);
        }



        RobotInitializers.Dashtelemetry.addData("pin freq", Localizer.pinPoint.getFrequency());

        prevgm1.copy(gm1);
        prevgm2.copy(gm2);
    }
}
