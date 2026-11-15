package org.firstinspires.ftc.teamcode.Biobuzz.Teleops.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Biobuzz.RobotComponents.RobotInitializers;
import org.firstinspires.ftc.teamcode.Biobuzz.RobotComponents.ShooterCalculator;

@Config
@TeleOp
public class PitchServoCalib extends LinearOpMode {

    public static Telemetry Dashtelemetry;

    @Override
    public void runOpMode() throws InterruptedException {
        RobotInitializers.InitializeHubs(hardwareMap);
        waitForStart();

        while (opModeIsActive()){
            RobotInitializers.ServoHub1.setServoPosition(3, (0.0008724202 * pitch * pitch * pitch * pitch  - 0.2466545 * pitch * pitch * pitch + 25.82175 * pitch * pitch - 1179.99294 * pitch + 19761.1581 + ShooterCalculator.pitchOffset)/355);

            RobotInitializers.Dashtelemetry.addData("pitch calc", (0.0008724202 * pitch * pitch * pitch * pitch  - 0.2466545 * pitch * pitch * pitch + 25.82175 * pitch * pitch - 1179.99294 * pitch + 19761.1581 + ShooterCalculator.pitchOffset));
            RobotInitializers.Dashtelemetry.update();
            RobotInitializers.clearCache();

        }
    }

    public static double pitch = 0;



}
