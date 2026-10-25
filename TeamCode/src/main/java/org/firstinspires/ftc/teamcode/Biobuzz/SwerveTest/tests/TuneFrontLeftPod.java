package org.firstinspires.ftc.teamcode.Biobuzz.SwerveTest.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.Biobuzz.RobotComponents.RobotInitializers;
import org.firstinspires.ftc.teamcode.Biobuzz.SwerveTest.SwerveKinematics;

@TeleOp
@Config
public class TuneFrontLeftPod extends LinearOpMode {
    public static double Offset = 0,P,D,F,BP,BD,BF,TargetAngle, powerMotor = 0;


    @Override
    public void runOpMode() throws InterruptedException {

        RobotInitializers.InitializeFull(hardwareMap);

        waitForStart();

        BP = SwerveKinematics.frontLeftModule.bigCoef.p;
        BD = SwerveKinematics.frontLeftModule.bigCoef.i;
        BF = SwerveKinematics.frontLeftModule.bigCoef.d;

        P = SwerveKinematics.frontLeftModule.smallCoef.p;
        D = SwerveKinematics.frontLeftModule.smallCoef.i;
        F = SwerveKinematics.frontLeftModule.smallCoef.d;

        while (opModeIsActive()){
            RobotInitializers.clearCache();

            SwerveKinematics.frontLeftModule.set(TargetAngle,powerMotor);
            SwerveKinematics.frontLeftModule.bigCoef = new PIDCoefficients(BP,BD,BF);
            SwerveKinematics.frontLeftModule.smallCoef = new PIDCoefficients(P,D,F);
            SwerveKinematics.frontLeftModule.SetEncoderOffset(Offset);


            SwerveKinematics.frontLeftModule.update();

            RobotInitializers.Dashtelemetry.addData("errorFromTarget",SwerveKinematics.frontLeftModule.error(SwerveKinematics.frontLeftModule.targetAngle,SwerveKinematics.frontLeftModule.getAngle()));
            RobotInitializers.Dashtelemetry.addData("currentAngle",SwerveKinematics.frontLeftModule.getAngle());

        }

    }
}
