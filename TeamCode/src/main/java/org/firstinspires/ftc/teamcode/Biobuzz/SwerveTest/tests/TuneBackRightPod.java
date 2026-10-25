package org.firstinspires.ftc.teamcode.Biobuzz.SwerveTest.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.Biobuzz.RobotComponents.RobotInitializers;
import org.firstinspires.ftc.teamcode.Biobuzz.SwerveTest.SwerveKinematics;

@TeleOp
@Config
public class TuneBackRightPod extends LinearOpMode {
    public static double Offset = 0,P,D,F,BP,BD,BF,TargetAngle, powerMotor = 0;


    @Override
    public void runOpMode() throws InterruptedException {

        RobotInitializers.InitializeFull(hardwareMap);

        waitForStart();

        BP = SwerveKinematics.backRightModule.bigCoef.p;
        BD = SwerveKinematics.backRightModule.bigCoef.i;
        BF = SwerveKinematics.backRightModule.bigCoef.d;

        P = SwerveKinematics.backRightModule.smallCoef.p;
        D = SwerveKinematics.backRightModule.smallCoef.i;
        F = SwerveKinematics.backRightModule.smallCoef.d;

        while (opModeIsActive()){
            RobotInitializers.clearCache();

            SwerveKinematics.backRightModule.set(TargetAngle,powerMotor);
            SwerveKinematics.backRightModule.bigCoef = new PIDCoefficients(BP,BD,BF);
            SwerveKinematics.backRightModule.smallCoef = new PIDCoefficients(P,D,F);
            SwerveKinematics.backRightModule.SetEncoderOffset(Offset);


            SwerveKinematics.backRightModule.update();

            RobotInitializers.Dashtelemetry.addData("errorFromTarget",SwerveKinematics.backRightModule.error(SwerveKinematics.backRightModule.targetAngle,SwerveKinematics.backRightModule.getAngle()));
            RobotInitializers.Dashtelemetry.addData("currentAngle",SwerveKinematics.backRightModule.getAngle());

        }

    }
}
