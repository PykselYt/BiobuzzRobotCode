package org.firstinspires.ftc.teamcode.Biobuzz.SwerveTest.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.Biobuzz.RobotComponents.RobotInitializers;
import org.firstinspires.ftc.teamcode.Biobuzz.SwerveTest.SwerveKinematics;

@TeleOp
@Config
public class TuneFrontRightPod extends LinearOpMode {

    public static double Offset = 0,P,D,F,BP,BD,BF,TargetAngle, powerMotor = 0;


    @Override
    public void runOpMode() throws InterruptedException {

        RobotInitializers.InitializeFull(hardwareMap);

        waitForStart();

        BP = SwerveKinematics.frontRightModule.bigCoef.p;
        BD = SwerveKinematics.frontRightModule.bigCoef.i;
        BF = SwerveKinematics.frontRightModule.bigCoef.d;

        P = SwerveKinematics.frontRightModule.smallCoef.p;
        D = SwerveKinematics.frontRightModule.smallCoef.i;
        F = SwerveKinematics.frontRightModule.smallCoef.d;

        while (opModeIsActive()){
            RobotInitializers.clearCache();

            SwerveKinematics.frontRightModule.set(TargetAngle,powerMotor);
            SwerveKinematics.frontRightModule.bigCoef = new PIDCoefficients(BP,BD,BF);
            SwerveKinematics.frontRightModule.smallCoef = new PIDCoefficients(P,D,F);
            SwerveKinematics.frontRightModule.SetEncoderOffset(Offset);


            SwerveKinematics.frontRightModule.update();

            RobotInitializers.Dashtelemetry.addData("errorFromTarget",SwerveKinematics.frontRightModule.error(SwerveKinematics.frontRightModule.targetAngle,SwerveKinematics.frontRightModule.getAngle()));
            RobotInitializers.Dashtelemetry.addData("currentAngle",SwerveKinematics.frontRightModule.getAngle());

        }

    }
}
