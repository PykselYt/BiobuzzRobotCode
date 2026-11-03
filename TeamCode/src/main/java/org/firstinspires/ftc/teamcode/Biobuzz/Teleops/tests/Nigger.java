package org.firstinspires.ftc.teamcode.Biobuzz.Teleops.tests;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Biobuzz.RobotComponents.RobotInitializers;


@TeleOp
@Config
public class Nigger extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        RobotInitializers.InitializeHubs(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            RobotInitializers.ControlHubMotors.setMotorPower(1, 0.5);

        }
    }


}
