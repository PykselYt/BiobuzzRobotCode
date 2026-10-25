package org.firstinspires.ftc.teamcode.Biobuzz.Teleops;

import static org.firstinspires.ftc.teamcode.Biobuzz.TeleopLogic.MainHandler.PinpointDown;

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

public class TeleopsStarter {

    public static TEAM team = TEAM.RED;

    public static Gamepad gm1;
    public static Gamepad gm2;

    public static boolean ActivateTelemetry = true;
    public static double rot = 1;
    public static boolean reverse = false;
    public static double AngleOffsetForDriving = 0;
    public MainHandler mainHandler;
    public static Storage storage;
    public static Shooter shooter;
    public static Turret turret;
    public static SparkFunOTOS.Pose2D resetPosition = new SparkFunOTOS.Pose2D(0,0,0);


    public static boolean DrivingFieldCentric = true;


    public static Gamepad prevgm1,prevgm2;

    public TeleopsStarter(HardwareMap hardwareMap, TEAM color){

        TeleopsStarter.gm1 = new Gamepad();
        TeleopsStarter.gm2 = new Gamepad();


        prevgm1 = new Gamepad();
        prevgm2 = new Gamepad();

        RobotInitializers.InitializeFull(hardwareMap);
        RobotInitializers.disable();
        //Camera.stopCamera();

        storage = new Storage();
        shooter = new Shooter();
        turret = new Turret();

        ShooterCalculator.shouldCompensateHood = true;

        ShooterCalculator.TurretOffsetStatic = 0;

        storage.lockPuta();

        mainHandler = new MainHandler();
        team = color;
    }



    public void InitUpdate(){
        RobotInitializers.clearCache();
        Chassis.usedTrajectory=Chassis.trajectoryStates.USENOTHING;
        ShooterCalculator.shouldCompensateHood = true;
        shooter.UpdateTurretAndPitch();

        if(gm1.cross){
            DrivingFieldCentric = true;
        }
        prevgm1.copy(gm1);
        prevgm2.copy(gm2);
        Localizer.Update();
    }


    public void update() {

        RobotInitializers.clearCache(ActivateTelemetry);

        Chassis.drive(DrivingFieldCentric, gm1.left_stick_y, -gm1.left_stick_x, -gm1.right_stick_x * rot);

        Chassis.Update();


        mainHandler.update();

        if(!PinpointDown) {
            //
            // if(TurnTurretOn)
            //shooter.FormulaCalculator.OffsetForTurretAuto(Shooter.FlyWheelEncoder.getRPM());
            turret.updateTurret(Localizer.getCurrentPosition());
            //RobotInitializers.Dashtelemetry.addData("IsStorageFull", storage.check3artefacts());
            shooter.UpdateTurretAndPitch(); //20-30
            shooter.updateFlyWheelSpeed(); //30-40
        }
        else
        {
            /*
            shooter.updateFlyWheelSpeed();
            shooter.SetTargetRPM(RPM);
            double newHoodWhileCompensating =  shooter.FormulaCalculator.CompensateRPMSagWitchHood(shooter.RPMError(),hood, ShooterCalculator.RPMErrorHoodCompensation);
            shooter.SetAngleToHood(newHoodWhileCompensating);
            turret.setAngle(1.08+Math.toRadians(OffsetTurretD2));

             */
        }


        //shooter.GenerateOffsetFromVelocity();
        Localizer.Update();
        //RobotInitializers.Dashtelemetry.addData("bearing",aprilTagHandler.GetTeamTagBearing());
        prevgm1.copy(gm1);
        prevgm2.copy(gm2);
    }
}
