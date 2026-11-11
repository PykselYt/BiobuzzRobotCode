package org.firstinspires.ftc.teamcode.Biobuzz.Teleops.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.Biobuzz.RobotComponents.RobotInitializers;
import org.firstinspires.ftc.teamcode.Biobuzz.RobotComponents.Shooter;

@TeleOp
@Config
public class TuneRPMShooter extends LinearOpMode {


    public static double rpm = 0;

    public static PIDCoefficients coefs = new PIDCoefficients(0,0,0);

    @Override
    public void runOpMode() throws InterruptedException {

        RobotInitializers.InitializeFull(hardwareMap);

        Shooter shooter = new Shooter();

        while(opModeInInit()){

        }

        waitForStart();

        while(opModeIsActive()){
            RobotInitializers.clearCache();

            shooter.SetTargetRPM(rpm);
            shooter.LauncherSpeedController.setPidCoefficients(coefs);

            shooter.updateFlyWheelSpeed();
        }
    }

}
