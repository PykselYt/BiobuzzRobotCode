package org.firstinspires.ftc.teamcode.Biobuzz.Teleops.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Biobuzz.RobotComponents.RobotInitializers;

@TeleOp
@Config
public class TestDigital extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotInitializers.InitializeHubs(hardwareMap);


        waitForStart();

        while(opModeIsActive()){

            RobotInitializers.clearCache();

            for(int i=0;i<8;i++)
            {
                RobotInitializers.Dashtelemetry.addData("c" + i + " ", RobotInitializers.ControlHubDigital.getDigitalChannelState(i));
                RobotInitializers.Dashtelemetry.addData("e" + i + " ", RobotInitializers.ExpansionHubDigital.getDigitalChannelState(i));
            }
        }
    }
}
