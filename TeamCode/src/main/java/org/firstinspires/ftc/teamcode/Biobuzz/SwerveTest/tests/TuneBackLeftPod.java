package org.firstinspires.ftc.teamcode.Biobuzz.SwerveTest.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.Biobuzz.RobotComponents.RobotInitializers;
import org.firstinspires.ftc.teamcode.Biobuzz.SwerveTest.SwerveKinematics;
import org.firstinspires.ftc.teamcode.Biobuzz.SwerveTest.SwervePod;

@TeleOp
@Config
public class TuneBackLeftPod extends LinearOpMode {
    public static double Offset = 0,P,D,F,BP,BD,BF,TargetAngle, powerMotor = 0;


    @Override
    public void runOpMode() throws InterruptedException {

        RobotInitializers.InitializeFull(hardwareMap);

        waitForStart();
/*
        BP = SwerveKinematics.backLeftModule.bigCoef.p;
        BD = SwerveKinematics.backLeftModule.bigCoef.i;
        BF = SwerveKinematics.backLeftModule.bigCoef.d;

        P = SwerveKinematics.backLeftModule.smallCoef.p;
        D = SwerveKinematics.backLeftModule.smallCoef.i;
        F = SwerveKinematics.backLeftModule.smallCoef.d;
*/
        while (opModeIsActive()){
            RobotInitializers.clearCache();



            SwerveKinematics.backLeftModule.set(TargetAngle,powerMotor);
            SwerveKinematics.backLeftModule.bigCoef = new PIDCoefficients(BP,BD,BF);
            SwerveKinematics.backLeftModule.smallCoef = new PIDCoefficients(P,D,F);
            SwerveKinematics.backLeftModule.SetEncoderOffset(Offset);


            SwerveKinematics.backLeftModule.update();

            RobotInitializers.Dashtelemetry.addData("errorFromTarget",SwerveKinematics.backLeftModule.error(SwerveKinematics.backLeftModule.targetAngle,SwerveKinematics.backLeftModule.getAngle()));
            RobotInitializers.Dashtelemetry.addData("currentAngle",SwerveKinematics.backLeftModule.getAngle());
            RobotInitializers.Dashtelemetry.addData("currentvoltage",SwerveKinematics.backLeftModule.encoder.getVoltage());
            RobotInitializers.Dashtelemetry.addData("currentmotorpower",SwerveKinematics.backLeftModule.motorPower);

        }

    }
}
