package org.firstinspires.ftc.teamcode.Biobuzz.Teleops.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
            if(hub == ServoTest.Hubs.ControlHub){
                RobotInitializers.ControlHubServos.setServoPosition(port, pitch / 355.f);
            } else if(hub == ServoTest.Hubs.ServoHub1) {
                RobotInitializers.ServoHub1.setServoPosition(port, pitch / 355.f);
            } else if(hub == ServoTest.Hubs.ServoHub2){
                RobotInitializers.ServoHub2.setServoPosition(port, pitch / 355.f);
            }
            Dashtelemetry.addData("pitch calc", (1.57621 * Math.pow(10, -7)) * Math.pow(pitch, 4) - 0.0000630077 * Math.pow(pitch, 3) + 0.00725814 * pitch * pitch - 0.0445644 * pitch + 65.659);
            Dashtelemetry.update();
            RobotInitializers.clearCache();

        }
    }

    public enum Hubs{
        ControlHub,
        ServoHub1,
        ServoHub2
    }
    public static ServoTest.Hubs hub = ServoTest.Hubs.ControlHub;
    public static int port = 0;
    public static double pitch = 0;



}
