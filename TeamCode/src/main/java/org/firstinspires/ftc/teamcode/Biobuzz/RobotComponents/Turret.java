package org.firstinspires.ftc.teamcode.Biobuzz.RobotComponents;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class Turret {



    public static double AngleOffsetFrom0 = 83.5;
    public static double BreathRoomPos = 0.02;
    public static double OppositionPos = 0.02;

    public static double ManualOffset = 0;

    public static double RotationalVelocityTurretCompensator = -0.08;
//    public static SparkFunOTOS.Pose2D goalPosition = new SparkFunOTOS.Pose2D(1437,3200,0);

    public static SparkFunOTOS.Pose2D goalPosition;
    public static double MaxAngle = 355;

    public static ServoImplEx mk21;
    public static AnalogInput TurretRawPosition;
    private static double INV_MAX_ANGLE_RAD =
            1.0 / Math.toRadians(MaxAngle);

    public Turret(){
        //time = new ElapsedTime();

        ShooterCalculator.TurretOffsetStatic = 0;

        mk21.setPwmRange(new PwmControl.PwmRange(500 , 2500));
        //mk22.setPwmRange(new PwmControl.PwmRange(500 , 2500));

        mk21.setDirection(Servo.Direction.FORWARD);
        //mk22.setDirection(Servo.Direction.FORWARD);
    }
    public static double offsetTurretX = -0.323;
    public static double offsetTurretY = -79.136;

    public static double offsetH = 0;

    public SparkFunOTOS.Pose2D TurretFieldRelativePosition(SparkFunOTOS.Pose2D robotpose){

        double cos = Math.cos(robotpose.h + offsetH);
        double sin = Math.sin(robotpose.h + offsetH);

        double trueOutputBallx = robotpose.x + offsetTurretX * cos - offsetTurretY * sin;
        double trueOutputBally = robotpose.y + offsetTurretX * sin + offsetTurretY * cos;

        return new SparkFunOTOS.Pose2D(trueOutputBallx,trueOutputBally, robotpose.h);
    }

    public static double radius = 100;

    public double directRobotAngle(SparkFunOTOS.Pose2D robotPose){

        robotPose = TurretFieldRelativePosition(robotPose);

        //double robotHeading = Localizer.getCurrentPosition().h;

        double dx = goalPosition.x - robotPose.x;
        double dy = goalPosition.y - robotPose.y;

        //double distanceToGoal = Math.hypot(dx, dy);;



        double targetGlobalHeading = Math.atan2(dy, dx);

        return targetGlobalHeading - Math.PI/2;

    }

    public void updateTurret(SparkFunOTOS.Pose2D robotPose){



        /*if(time.seconds() < 1.0/Hz)
            return;
        time.reset();
        */
        robotPose = TurretFieldRelativePosition(robotPose);
        if(robotPose.x > ShooterCalculator.halfFieldDist && ShooterCalculator.usedPos1){
            ShooterCalculator.usedPos1 = false;
            goalPosition = ShooterCalculator.goalPosition2;
        }

        if(robotPose.x < ShooterCalculator.halfFieldDist && !ShooterCalculator.usedPos1){
            ShooterCalculator.usedPos1 = true;
            goalPosition = ShooterCalculator.goalPosition1;
        }

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
        double ShouldHaveTurretHeading = Localizer.normalizeTo0To2PI(targetGlobalHeading - robotHeading + ManualOffset + Math.toRadians(AngleOffsetFrom0) /*+ (distanceToGoal < 2600?ShooterCalculator.TurretOffsetStatic:0)*/ + RotationalVelocityTurretCompensator * (Localizer.getVelocity().h));


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

        setAngle(ShouldHaveTurretHeading);

    }
    private double angleCached = 1000;

    public void setAngle(double angle){
        //RobotInitializers.Dashtelemetry.addData("angle",angle);
        //RobotInitializers.Dashtelemetry.addData("m21",Math.max(BreathRoomPos,Math.min(1-BreathRoomPos,angle/MaxAngle+OppositionPos)));


        /*angle = ((int)(angle * 100) / 100.f);
        if(angle == angleCached) return; // Cacheing
        angleCached = angle;*/
        double anglenorm = angle * INV_MAX_ANGLE_RAD;
        mk21.setPosition(Math.max(BreathRoomPos, Math.min(1 - BreathRoomPos, (1 - anglenorm) + OppositionPos)));
        //mk22.setPosition(Math.max(BreathRoomPos, Math.min(1 - BreathRoomPos, (1 - anglenorm) - OppositionPos)));
    }

    public void setAngleToInit(double val){
        mk21.setPosition(val);
        //mk22.setPosition(val);
    }



}
