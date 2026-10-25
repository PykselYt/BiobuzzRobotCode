package org.firstinspires.ftc.teamcode.Biobuzz.TeleopLogic;

import static org.firstinspires.ftc.teamcode.Biobuzz.TeleopLogic.Commands.ShootAllSequence.ShootTasks;
import static org.firstinspires.ftc.teamcode.Biobuzz.Teleops.TeleopsStarter.gm1;
import static org.firstinspires.ftc.teamcode.Biobuzz.Teleops.TeleopsStarter.gm2;
import static org.firstinspires.ftc.teamcode.Biobuzz.Teleops.TeleopsStarter.prevgm1;
import static org.firstinspires.ftc.teamcode.Biobuzz.Teleops.TeleopsStarter.prevgm2;
import static org.firstinspires.ftc.teamcode.Biobuzz.Teleops.TeleopsStarter.storage;

import org.firstinspires.ftc.teamcode.Biobuzz.ActionsCommandLineImplementation.Scheduler;
import org.firstinspires.ftc.teamcode.Biobuzz.RobotComponents.Localizer;
import org.firstinspires.ftc.teamcode.Biobuzz.RobotComponents.ShooterCalculator;
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
        if((gm1.right_bumper && gm1.left_bumper) || (!gm1.right_bumper && !gm1.left_bumper)) {
            storage.stop();
            //rot = 1;
        }
        if(gm1.right_bumper) {
            storage.takeIn();
            //rot = 0.6;
        }
        else if(gm1.left_bumper) {
            storage.spit();
            //rot = 1;
        }
    }

    public void update()
    {
        ActionIntake();
        if(prevgm1.square != gm1.square && gm1.square && tasks.IsSchedulerDone()){
            tasks.AddAnotherScheduler(ShootTasks);
        }

        if(prevgm1.triangle != gm1.triangle && gm1.triangle){
            Localizer.setHeading(Math.toRadians(TeleopsStarter.resetPosition.h));
            Turret.ManualOffset=0;
        }
        if(prevgm1.circle != gm1.circle && gm1.circle ){
            Localizer.setPosition(TeleopsStarter.resetPosition);
            Turret.ManualOffset=0;
            ShooterCalculator.manualHoodAddition=0;
            ShooterCalculator.manualRPMAddition=0;
        }
        if(prevgm1.cross != gm1.cross && gm1.cross ){
            Localizer.setHeading(Math.toRadians(TeleopsStarter.resetPosition.h+180));
            Turret.ManualOffset=0;
        }




        if(prevgm1.right_stick_button != gm1.right_stick_button && gm1.right_stick_button){
            ShooterCalculator.clampDistance = !ShooterCalculator.clampDistance;
        }



        if(prevgm2.square != gm2.square && gm2.square)
            PinpointDown = !PinpointDown;



        if(gm2.circle && prevgm2.dpad_up != gm2.dpad_up && gm2.dpad_up)
            ShooterCalculator.manualHoodAddition-=1;
        if(gm2.circle&& prevgm2.dpad_down != gm2.dpad_down && gm2.dpad_down)
            ShooterCalculator.manualHoodAddition+=1;
        if(gm2.circle && prevgm2.dpad_right != gm2.dpad_right && gm2.dpad_right)
            ShooterCalculator.manualRPMAddition+=50;
        if(gm2.circle&& prevgm2.dpad_left != gm2.dpad_left && gm2.dpad_left)
            ShooterCalculator.manualRPMAddition-=50;


        if(prevgm2.dpad_up != gm2.dpad_up && gm2.dpad_up && !gm2.circle)
            hood-=1;
        if(prevgm2.dpad_down != gm2.dpad_down && gm2.dpad_down && !gm2.circle)
            hood+=1;

        if(prevgm2.dpad_right != gm2.dpad_right && gm2.dpad_right && !gm2.circle)
            RPM+=100;
        if(prevgm2.dpad_left != gm2.dpad_left && gm2.dpad_left && !gm2.circle)
            RPM-=100;


        if(Math.abs(gm2.right_stick_x) > 0.05)
            OffsetTurretD2 = Math.toRadians(-gm2.right_stick_x * Multiplier);
        else
            OffsetTurretD2 = 0;

        if(prevgm2.left_bumper != gm2.left_bumper && gm2.left_bumper)
            Turret.ManualOffset += Math.toRadians(1);
        if(prevgm2.right_bumper != gm2.right_bumper && gm2.right_bumper)
            Turret.ManualOffset -= Math.toRadians(1);


        if(prevgm2.triangle != gm2.triangle && gm2.triangle)
            TurnTurretOn = true;

        if(prevgm2.right_stick_button != gm2.right_stick_button && gm2.right_stick_button)
            ShooterCalculator.clampDistance = !ShooterCalculator.clampDistance;



        tasks.update();
    }


}
