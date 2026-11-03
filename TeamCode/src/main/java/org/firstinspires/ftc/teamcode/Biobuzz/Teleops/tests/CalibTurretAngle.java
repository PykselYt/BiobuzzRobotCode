package org.firstinspires.ftc.teamcode.Biobuzz.Teleops.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Biobuzz.RobotComponents.Localizer;
import org.firstinspires.ftc.teamcode.Biobuzz.RobotComponents.RobotInitializers;
import org.firstinspires.ftc.teamcode.Biobuzz.RobotComponents.ShooterCalculator;
import org.firstinspires.ftc.teamcode.Biobuzz.RobotComponents.Turret;

@TeleOp
@Config
public class CalibTurretAngle extends LinearOpMode {


    public static double offset = 0;
    public static SparkFunOTOS.Pose2D goalPosition = new SparkFunOTOS.Pose2D(0, 100, 0);

    @Override
    public void runOpMode() throws InterruptedException{

        RobotInitializers.InitializeFull(hardwareMap);

        Turret turret = new Turret();

        Localizer.Reset();

        waitForStart();

        while(opModeIsActive()){

            SparkFunOTOS.Pose2D robotPose = Localizer.getCurrentPosition();

            robotPose = turret.TurretFieldRelativePosition(robotPose);


            //RobotInitializers.Dashtelemetry.addData("pos turret",robotPose);
            //RobotInitializers.Dashtelemetry.addData("OffsetFromMoving", ShooterCalculator.OffsetTurretAngle);

            //RobotInitializers.Dashtelemetry.addData("turretpos", TurretRawPosition.getVoltage() / TurretRawPosition.getMaxVoltage() * MaxAngle);
            double robotHeading = Localizer.getCurrentPosition().h;

            double dx = goalPosition.x - robotPose.x;
            double dy = goalPosition.y - robotPose.y;

            double distanceToGoal = Math.hypot(dx, dy);;



            double targetGlobalHeading = Math.atan2(dy, dx);

        /*double xAdd = radius * Math.cos(targetGlobalHeading);
        double yAdd = radius * Math.sin(targetGlobalHeading);

        targetGlobalHeading = Math.atan2(dy + yAdd, dx + xAdd);*/

            //RobotInitializers.Dashtelemetry.addData("targetGlobalHeading",targetGlobalHeading);
            double ShouldHaveTurretHeading = Localizer.normalizeTo0To2PI(targetGlobalHeading - robotHeading+ Math.toRadians(offset) /*+ (distanceToGoal < 2600?ShooterCalculator.TurretOffsetStatic:0) + RotationalVelocityTurretCompensator * (Localizer.getVelocity().h)*/);


            //RobotInitializers.Dashtelemetry.addData("OffsetFromMoving", ShooterCalculator.OffsetTurretAngle);
            //RobotInitializers.Dashtelemetry.addData("ShouldHaveTurretHeading",ShouldHaveTurretHeading);

            //ShouldHaveTurretHeading = Math.min(ShouldHaveTurretHeading,Math.toRadians(MaxAngle));
            //if(dy >= 2100) {
            //    if (ShouldHaveTurretHeading >= 2.2) {
            //        ShouldHaveTurretHeading += LinearFunction.getOutput(0, Math.toRadians(offsetEnd1), 2.2, 5.78, ShouldHaveTurretHeading);
            //    } else {
            //        ShouldHaveTurretHeading += LinearFunction.getOutput(0, Math.toRadians(offsetEnd2), 2.2, 0.48, ShouldHaveTurretHeading);
            //    }
            //}

            turret.setAngle(ShouldHaveTurretHeading);
        }

    }

}
