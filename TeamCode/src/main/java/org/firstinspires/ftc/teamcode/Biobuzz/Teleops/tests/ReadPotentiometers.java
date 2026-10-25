package org.firstinspires.ftc.teamcode.Biobuzz.Teleops.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.Biobuzz.RobotComponents.RobotInitializers;

@TeleOp
@Config
public class ReadPotentiometers extends LinearOpMode {

    public static int port = 0;

    public static int ServoPort = 0;

    public static double angle = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        RobotInitializers.InitializeHubs(hardwareMap);

        AnalogInput analog;


        waitForStart();

        while(opModeIsActive()){
            RobotInitializers.clearCache();

            analog = new AnalogInput(RobotInitializers.ControlHubAnalog,port);
            RobotInitializers.ControlHubServos.setServoPosition(ServoPort, angle / 355.f);

            RobotInitializers.Dashtelemetry.addData("analogRead",analog.getVoltage());

            RobotInitializers.Dashtelemetry.update();


        }
    }
}
