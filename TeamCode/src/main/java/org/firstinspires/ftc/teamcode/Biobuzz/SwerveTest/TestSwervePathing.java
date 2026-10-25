package org.firstinspires.ftc.teamcode.Biobuzz.SwerveTest;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.Biobuzz.ActionsCommandLineImplementation.Scheduler;
import org.firstinspires.ftc.teamcode.Biobuzz.ActionsCommandLineImplementation.Task;
import org.firstinspires.ftc.teamcode.Biobuzz.RobotComponents.Chassis;
import org.firstinspires.ftc.teamcode.Biobuzz.RobotComponents.Localizer;
import org.firstinspires.ftc.teamcode.Biobuzz.RobotComponents.RobotInitializers;


@TeleOp
@Config
public class TestSwervePathing extends LinearOpMode {



    public static double x = 0;
    public static double y = 0;
    public static double hdeg = 0;
    public static double h = Math.toRadians(hdeg);

    public static PIDCoefficients strafe= new PIDCoefficients(0,0,0);
    public static PIDCoefficients forward = new PIDCoefficients(0,0,0);
    public static PIDCoefficients heading = new PIDCoefficients(0,0,0);


    @Override
    public void runOpMode() throws InterruptedException {

        RobotInitializers.InitializeFull(hardwareMap);
        Localizer.setPosition(new SparkFunOTOS.Pose2D(0, 0, 0));
        Scheduler tasks = new Scheduler()
                .addTask(new Task() {
                    @Override
                    protected void Actions() {
                        Chassis.setTargetPosition(new SparkFunOTOS.Pose2D(0,0,0));
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
                        Chassis.setTargetPosition(new SparkFunOTOS.Pose2D(x,y,h));
                    }

                    @Override
                    protected boolean Conditions() {
                        return true;
                    }
                })
                .waitSeconds(5);

        Scheduler currenttasks = new Scheduler();
        currenttasks.AddAnotherScheduler(tasks);

        strafe = Chassis.Strafe.pidCoefficients;
        forward = Chassis.Forward.pidCoefficients;
        heading = Chassis.Heading.pidCoefficients;

        Chassis.usedTrajectory = Chassis.trajectoryStates.FREEWILL;

        while(opModeInInit()){
            Localizer.Reset();
        }



        while(opModeIsActive())
        {
            h = Math.toRadians(hdeg);
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
