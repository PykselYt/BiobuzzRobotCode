package org.firstinspires.ftc.teamcode.Biobuzz.TeleopLogic;

import static org.firstinspires.ftc.teamcode.Biobuzz.TeleopLogic.Commands.ShootAllSequence.ShootTasks;
import static org.firstinspires.ftc.teamcode.Biobuzz.Teleops.TeleopsStarter.gm1;
import static org.firstinspires.ftc.teamcode.Biobuzz.Teleops.TeleopsStarter.gm2;
import static org.firstinspires.ftc.teamcode.Biobuzz.Teleops.TeleopsStarter.prevgm1;
import static org.firstinspires.ftc.teamcode.Biobuzz.Teleops.TeleopsStarter.prevgm2;
import static org.firstinspires.ftc.teamcode.Biobuzz.Teleops.TeleopsStarter.shooter;
import static org.firstinspires.ftc.teamcode.Biobuzz.Teleops.TeleopsStarter.storage;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.teamcode.Biobuzz.ActionsCommandLineImplementation.Scheduler;
import org.firstinspires.ftc.teamcode.Biobuzz.RobotComponents.Chassis;
import org.firstinspires.ftc.teamcode.Biobuzz.RobotComponents.Localizer;
import org.firstinspires.ftc.teamcode.Biobuzz.RobotComponents.ShooterCalculator;
import org.firstinspires.ftc.teamcode.Biobuzz.RobotComponents.Storage;
import org.firstinspires.ftc.teamcode.Biobuzz.RobotComponents.Turret;
import org.firstinspires.ftc.teamcode.Biobuzz.Teleops.TeleopsStarter;

public class MainHandler {


    Scheduler tasks;

    public static boolean PinpointDown = false;
    public static double RPM = 3000, hood = 50;

    public static boolean TurnTurretOn = false;

    public static double OffsetTurretD2 = 0;
    public static double Multiplier = 10;
    public MainHandler()
    {
        tasks = new Scheduler();
    }
    public static boolean shouldUpdateStorage = false;

    public void ActionIntake()
    {
        if(gm1.circle && gm1.circle!=prevgm1.circle){
            storage.switchRamp();
        }
        if(gm1.right_bumper!= prevgm1.right_bumper || gm1.left_bumper!= prevgm1.left_bumper){
            if(gm1.right_bumper && gm1.left_bumper) {
                if(TeleopsStarter.prepareShoot) TeleopsStarter.speedMult = TeleopsStarter.rotMult = 1;
                storage.stop();
            }
            else if(gm1.right_bumper) {
                if(TeleopsStarter.prepareShoot) {
                    TeleopsStarter.speedMult = TeleopsStarter.rotMult = 0.6;
                    storage.takeIn(storage.transferSpeed);
                }else{
                    storage.takeIn();
                }

            }
            else if(gm1.left_bumper) {
                if(TeleopsStarter.prepareShoot) TeleopsStarter.speedMult = TeleopsStarter.rotMult = 1;
                storage.spit();
            }
            else {
                if(TeleopsStarter.prepareShoot) TeleopsStarter.speedMult = TeleopsStarter.rotMult = 1;
                storage.idle();

            }
        }

    }

    public void update()
    {

        ActionIntake();

        if(gm1.cross && gm1.cross != prevgm1.cross){
            Localizer.setPosition(new SparkFunOTOS.Pose2D(0,0,0));
            Chassis.kinematics.resetHoldHeading();
        }


        if(prevgm1.square != gm1.square && gm1.square){
            if(TeleopsStarter.prepareShoot){
                TeleopsStarter.speedMult = TeleopsStarter.rotMult = 1;
                storage.lockPuta();
                TeleopsStarter.prepareShoot = false;
                shooter.Pitch.setAngle(Storage.blockPos);
                shooter.setPowerEquilibrated(0.75);
            }else{
                storage.unlockPuta();
                TeleopsStarter.prepareShoot = true;
            }


        }


        //tasks.update();


    }


}
