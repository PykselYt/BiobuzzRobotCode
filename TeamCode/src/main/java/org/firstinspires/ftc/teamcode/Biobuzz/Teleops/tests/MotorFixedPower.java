package org.firstinspires.ftc.teamcode.Biobuzz.Teleops.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Biobuzz.RobotComponents.RobotInitializers;
import org.firstinspires.ftc.teamcode.Biobuzz.RobotComponents.Shooter;

@TeleOp
@Config
public class MotorFixedPower extends LinearOpMode {
    public static double power = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        RobotInitializers.InitializeFull(hardwareMap);

        Shooter shooter = new Shooter();

        waitForStart();

        while (opModeIsActive()){

            shooter.setPowerEquilibrated(power);

            RobotInitializers.Dashtelemetry.addData("AMP1", shooter.Launcher1.getCurrent(CurrentUnit.AMPS));
            RobotInitializers.Dashtelemetry.addData("AMP2", shooter.Launcher2.getCurrent(CurrentUnit.AMPS));


            RobotInitializers.clearCache();

            telemetry.update();
        }
    }
}
