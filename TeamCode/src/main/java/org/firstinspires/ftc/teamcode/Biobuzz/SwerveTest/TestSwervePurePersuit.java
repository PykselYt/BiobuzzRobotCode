package org.firstinspires.ftc.teamcode.Biobuzz.SwerveTest;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.Biobuzz.ActionsCommandLineImplementation.Scheduler;
import org.firstinspires.ftc.teamcode.Biobuzz.ActionsCommandLineImplementation.Task;
import org.firstinspires.ftc.teamcode.Biobuzz.Pathing.PurePersuit;
import org.firstinspires.ftc.teamcode.Biobuzz.RobotComponents.Chassis;
import org.firstinspires.ftc.teamcode.Biobuzz.RobotComponents.Localizer;
import org.firstinspires.ftc.teamcode.Biobuzz.RobotComponents.RobotInitializers;

import java.util.ArrayList;
import java.util.Arrays;


@TeleOp
@Config
public class TestSwervePurePersuit extends LinearOpMode {

    public static SparkFunOTOS.Pose2D PoseOne = new SparkFunOTOS.Pose2D(0,0,0),
                                        PoseTwo = new SparkFunOTOS.Pose2D(0, 1000, Math.toRadians(-45)),
                                        PoseThree = new SparkFunOTOS.Pose2D(1000, 1000, Math.toRadians(-90));

    public static PIDCoefficients strafe= new PIDCoefficients(0,0,0);
    public static PIDCoefficients forward = new PIDCoefficients(0,0,0);
    public static PIDCoefficients heading = new PIDCoefficients(0,0,0);

    @Override
    public void runOpMode() throws InterruptedException{

        RobotInitializers.InitializeFull(hardwareMap);
        Localizer.setPosition(new SparkFunOTOS.Pose2D(0, 0, 0));
        strafe = Chassis.Strafe.pidCoefficients;
        forward = Chassis.Forward.pidCoefficients;
        heading = Chassis.Heading.pidCoefficients;

        ArrayList<PurePersuit.Point> purePersuitPathOne = new ArrayList<>(Arrays.asList(
                new PurePersuit.Point(PoseOne.x, PoseOne.y, PoseOne.h, 0.5),
                new PurePersuit.Point(PoseTwo.x, PoseTwo.y, PoseTwo.h, 0.5),
                new PurePersuit.Point(PoseThree.x, PoseThree.y, PoseThree.h, 0.5)
        ));
        ArrayList<PurePersuit.Point> purePersuitPathTwo = new ArrayList<>(Arrays.asList(
                new PurePersuit.Point(PoseThree.x, PoseThree.y, PoseThree.h, 0.5),
                new PurePersuit.Point(PoseTwo.x, PoseTwo.y, PoseTwo.h, 0.5),
                new PurePersuit.Point(PoseOne.x, PoseOne.y, PoseOne.h, 0.5)

        ));

        Scheduler tasks = new Scheduler()
                .StartPurePersuit(purePersuitPathOne, 0, 800)
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                    }
                    @Override
                    protected boolean Conditions() {
                        return Chassis.IsPositionDone(100);
                    }
                })
                .waitSeconds(5)
                .StartPurePersuit(purePersuitPathTwo, 0, 800)
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                    }

                    @Override
                    protected boolean Conditions() {
                        return Chassis.IsPositionDone(100);
                    }
                })
                .waitSeconds(5);

        Scheduler currenttasks = new Scheduler();
        currenttasks.AddAnotherScheduler(tasks);
        while(opModeInInit()){
            //Localizer.Reset();
        }


        while(opModeIsActive())
        {
            //Chassis.Strafe.setPidCoefficients(strafe);
            //Chassis.Forward.setPidCoefficients(forward);
            //Chassis.Heading.setPidCoefficients(heading);

            currenttasks.update();
            if(currenttasks.IsSchedulerDone())
                currenttasks.AddAnotherScheduler(tasks);

            Localizer.Update();
            Chassis.Update();
            Chassis.kinematics.updatePods();
            RobotInitializers.clearCache();
        }


    }

}
