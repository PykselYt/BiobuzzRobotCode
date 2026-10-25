package org.firstinspires.ftc.teamcode.Biobuzz.SwerveTest;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Biobuzz.MathHelpers.MathUtils;
import org.firstinspires.ftc.teamcode.Biobuzz.RobotComponents.Chassis;
import org.firstinspires.ftc.teamcode.Biobuzz.RobotComponents.Localizer;
import org.firstinspires.ftc.teamcode.Biobuzz.RobotComponents.RobotInitializers;


@TeleOp(name="sixsevenswerve")
@Config
public class TestSwerveKinematics extends LinearOpMode {

    public static boolean FC = true;

    @Override
    public void runOpMode() throws InterruptedException {

        RobotInitializers.InitializeFull(hardwareMap);
        MathUtils.InitializePrecalculation();

        SwerveKinematics kinematics = new SwerveKinematics();

        waitForStart();
        //Localizer.startThread();

        Chassis.usedTrajectory = Chassis.trajectoryStates.USENOTHING;

        while (opModeIsActive()){
            RobotInitializers.clearCache(false);

            kinematics.setFC(gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x * 0.6);

            Localizer.Update();
            kinematics.updatePods();

            //telemetry.addData("x", Localizer.getCurrentPosition().x);
            //telemetry.addData("y", Localizer.getCurrentPosition().y);
            //telemetry.addData("h", Localizer.getCurrentPosition().h);



            //telemetry.update();

            //RobotInitializers.Dashtelemetry.addData("forward", gamepad1.left_stick_y);
            //RobotInitializers.Dashtelemetry.addData("strafe", -gamepad1.left_stick_x);
            //RobotInitializers.Dashtelemetry.addData("rotation", gamepad1.right_stick_x);
        }
        //Localizer.stopThread();
    }
}
