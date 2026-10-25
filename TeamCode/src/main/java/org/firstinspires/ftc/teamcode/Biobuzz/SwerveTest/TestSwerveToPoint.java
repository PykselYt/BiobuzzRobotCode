package org.firstinspires.ftc.teamcode.Biobuzz.SwerveTest;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Biobuzz.ActionsCommandLineImplementation.Scheduler;
import org.firstinspires.ftc.teamcode.Biobuzz.ActionsCommandLineImplementation.Task;
import org.firstinspires.ftc.teamcode.Biobuzz.RobotComponents.Chassis;
import org.firstinspires.ftc.teamcode.Biobuzz.RobotComponents.Localizer;
import org.firstinspires.ftc.teamcode.Biobuzz.RobotComponents.RobotInitializers;

@TeleOp
@Config
public class TestSwerveToPoint extends LinearOpMode {

    public static double x = 0;
    public static double y = 0;


    @Override
    public void runOpMode() throws InterruptedException{

        RobotInitializers.InitializeFull(hardwareMap);
        Localizer.setPosition(new SparkFunOTOS.Pose2D(0, 0, 0));


        Scheduler tasks = new Scheduler()
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        Chassis.orientatePodsToPos(new SparkFunOTOS.Pose2D(0, 0, 0));
                    }

                    @Override
                    protected boolean Conditions() {
                        return true;
                    }
                })
                .waitSeconds(0.5)
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        Chassis.setTargetPosition(new SparkFunOTOS.Pose2D(0,0,0));
                        Chassis.usedTrajectory = Chassis.trajectoryStates.DIRECTPATH;
                    }
                    @Override
                    protected boolean Conditions() {
                        return true;
                    }
                })
                .waitSeconds(5)
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        Chassis.orientatePodsToPos(new SparkFunOTOS.Pose2D(x, y, 0));
                    }

                    @Override
                    protected boolean Conditions() {
                        return true;
                    }
                })
                .waitSeconds(0.5)
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        Chassis.setTargetPosition(new SparkFunOTOS.Pose2D(x,y,0));
                        Chassis.usedTrajectory = Chassis.trajectoryStates.DIRECTPATH;
                    }

                    @Override
                    protected boolean Conditions() {
                        return true;
                    }
                })
                .waitSeconds(5);

        Scheduler currenttasks = new Scheduler();
        currenttasks.AddAnotherScheduler(tasks);



        while(opModeInInit()){}

        while(opModeIsActive())
        {

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
