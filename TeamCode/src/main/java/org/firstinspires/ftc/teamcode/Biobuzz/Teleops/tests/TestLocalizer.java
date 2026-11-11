package org.firstinspires.ftc.teamcode.Biobuzz.Teleops.tests;

import static org.firstinspires.ftc.teamcode.Biobuzz.Teleops.TeleopsStarter.DrivingFieldCentric;
import static org.firstinspires.ftc.teamcode.Biobuzz.Teleops.TeleopsStarter.rotMult;
import static org.firstinspires.ftc.teamcode.Biobuzz.Teleops.TeleopsStarter.speedMult;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Biobuzz.RobotComponents.Chassis;
import org.firstinspires.ftc.teamcode.Biobuzz.RobotComponents.Localizer;
import org.firstinspires.ftc.teamcode.Biobuzz.RobotComponents.RobotInitializers;

@TeleOp
@Config
public class TestLocalizer extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException{

        RobotInitializers.InitializeFull(hardwareMap);
        Localizer.Initialize(hardwareMap);


        Localizer.Reset();

        waitForStart();

        while(opModeIsActive()){
            RobotInitializers.clearCache();
            Localizer.Update();
            Chassis.drive(DrivingFieldCentric, -gamepad1.left_stick_x * speedMult, -gamepad1.left_stick_y * speedMult, (gamepad1.left_trigger - gamepad1.right_trigger) * rotMult);

            Chassis.Update();


            telemetry.addData("XPos", Localizer.getCurrentPosition().x);
            telemetry.addData("YPos", Localizer.getCurrentPosition().y);
            telemetry.addData("HPos", Math.toDegrees(Localizer.getCurrentPosition().h));

            telemetry.update();
        }

    }

}
