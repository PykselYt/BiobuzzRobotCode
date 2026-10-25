package org.firstinspires.ftc.teamcode.Biobuzz.Teleops.tests;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.Biobuzz.RobotComponents.RobotInitializers;

@Config
@TeleOp
public class ServoTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        RobotInitializers.InitializeHubs(hardwareMap);
        waitForStart();

        while (opModeIsActive()){
            if(hub == Hubs.ControlHub){
                RobotInitializers.ControlHubServos.setServoPosition(port, angle / 355.f);
            } else if(hub == Hubs.ServoHub) {
                //RobotInitializers.ServoHub.setServoPosition(port, angle / 355.f);
            }

            RobotInitializers.clearCache();

        }
    }

    public enum Hubs{
        ControlHub,
        ServoHub,
        ExpansionHub
    }
    public static Hubs hub = Hubs.ControlHub;
    public static int port = 0;
    public static double angle = 0;


}