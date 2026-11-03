package org.firstinspires.ftc.teamcode.Biobuzz.Teleops.tests;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Biobuzz.RobotComponents.RobotInitializers;
import org.firstinspires.ftc.teamcode.Biobuzz.RobotComponents.Turret;

@TeleOp
@Config
public class SetTurretServos extends LinearOpMode {

    public static double angle = 0;

    @Override
    public void runOpMode() throws InterruptedException{

        RobotInitializers.InitializeFull(hardwareMap);

        Turret turret = new Turret();

        waitForStart();

        while(opModeIsActive()){
            turret.setAngleToInit(angle);
        }

    }

}
