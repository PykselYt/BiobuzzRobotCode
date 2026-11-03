package org.firstinspires.ftc.teamcode.Biobuzz.Teleops.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Biobuzz.RobotComponents.Localizer;
import org.firstinspires.ftc.teamcode.Biobuzz.RobotComponents.RobotInitializers;

@TeleOp
@Config
public class TestLocalizer extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException{

        RobotInitializers.InitializeHubs(hardwareMap, true);
        Localizer.Initialize(hardwareMap);


        Localizer.Reset();

        waitForStart();

        while(opModeIsActive()){
            Localizer.Update();
            telemetry.addData("XPos", Localizer.getCurrentPosition().x);
            telemetry.addData("YPos", Localizer.getCurrentPosition().y);
            telemetry.addData("HPos", Localizer.getCurrentPosition().h);

            telemetry.update();
        }

    }

}
