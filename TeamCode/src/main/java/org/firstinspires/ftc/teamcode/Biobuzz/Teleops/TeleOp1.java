package org.firstinspires.ftc.teamcode.Biobuzz.Teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Biobuzz.RobotComponents.RobotInitializers;
import org.firstinspires.ftc.teamcode.Biobuzz.RobotComponents.ShooterCalculator;

@TeleOp(name = ".one")
public class TeleOp1 extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        TeleopsStarter opmode = new TeleopsStarter(hardwareMap, TEAM.RED);
        TeleopsStarter.team = TEAM.RED;


        while(opModeInInit()){
            TeleopsStarter.gm1.copy(gamepad1);
            TeleopsStarter.gm2.copy(gamepad2);
            opmode.InitUpdate();
        }

        waitForStart();
        RobotInitializers.enable();


        while (opModeIsActive()){

            TeleopsStarter.gm1.copy(gamepad1);
            TeleopsStarter.gm2.copy(gamepad2);
            opmode.update();

        }
    }
}
