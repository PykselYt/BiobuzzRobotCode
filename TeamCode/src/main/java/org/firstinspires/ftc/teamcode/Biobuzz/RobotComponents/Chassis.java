package org.firstinspires.ftc.teamcode.Biobuzz.RobotComponents;

import static java.lang.Math.sqrt;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Biobuzz.MathHelpers.PIDController;
import org.firstinspires.ftc.teamcode.Biobuzz.Pathing.PurePersuit;
import org.firstinspires.ftc.teamcode.Biobuzz.SwerveTest.SwerveKinematics;
import org.firstinspires.ftc.teamcode.Biobuzz.SwerveTest.SwervePod;
import org.firstinspires.ftc.teamcode.Biobuzz.SwerveTest.TestSwerveKinematics;

@Config
public class Chassis {


    public static PurePersuit PurePersuitTrajectory;

    public static SwerveKinematics kinematics = new SwerveKinematics();

    public static SparkFunOTOS.Pose2D target = new SparkFunOTOS.Pose2D(0,0,0);

    public static double breakingConstant = 5;



    public static PIDController Strafe = new PIDController(-0.007,0,-0.00075),
            Forward = new PIDController(-0.007,0,-0.00075),
            Heading = new PIDController(-0.74,0,-0.07);

    public static PIDCoefficients NormalStrafe = new PIDCoefficients(-0.007, 0, -0.0075),
            NormalForward = new PIDCoefficients(-0.007, 0, -0.0075),
            NormalHeading = new PIDCoefficients(-0.007, 0, -0.0075),
            SlowStrafe = new PIDCoefficients(-0.004, 0, -0.0045),
            SlowForward = new PIDCoefficients(-0.004, 0, -0.0045),
            SlowHeading = new PIDCoefficients(-0.004, 0, -0.0045);

    static{
        Strafe.setFreq(30);
        Forward.setFreq(30);
        Heading.setFreq(30);

        Strafe.kS = -0.0;
        Forward.kS = 0.00;
        Heading.kS = -0.0; //-0.035

    }
    public enum trajectoryStates{
        FREEWILL,
        FOLLOWINGPUREPERSUIT,
        FOLLOWINGMOTIONPREDICTIVECONTROL,
        OFF,
        USENOTHING,
        DIRECTPATH,
    }

    public static trajectoryStates usedTrajectory = trajectoryStates.FREEWILL;

    public static void SlowPids(){
        Forward.pidCoefficients = SlowForward;
        Strafe.pidCoefficients = SlowStrafe;
        Heading.pidCoefficients = SlowHeading;
    }

    public static void NormalPids(){
        Forward.pidCoefficients = NormalForward;
        Strafe.pidCoefficients = NormalStrafe;
        Heading.pidCoefficients = NormalHeading;
    }

    public static void setTargetPosition(SparkFunOTOS.Pose2D pos){

        pos.h = Localizer.normalizeRadians(pos.h);

        Strafe.setTargetPosition(0);
        Forward.setTargetPosition(0);
        Heading.setTargetPosition(0);
        target = pos;
    }

    public static SparkFunOTOS.Pose2D getTargetPosition(){
        return target;
    }

    public static boolean IsPositionDone(double error_distance){
        switch (usedTrajectory){
            case FREEWILL:
                return Localizer.getDistanceFromTwoPoints(getTargetPosition(),Localizer.getCurrentPosition()) <= error_distance;
            case FOLLOWINGPUREPERSUIT:
                return PurePersuitTrajectory.TrajectoryDone(error_distance);
            case FOLLOWINGMOTIONPREDICTIVECONTROL:
                //maybe in the future
        }

        return false;
    }

    public static void orientatePodsToPos(SparkFunOTOS.Pose2D targetPose){
        double distance = Localizer.getDistanceFromTwoPoints(Localizer.getCurrentPosition(), targetPose);
        double unnormalX = targetPose.x - Localizer.getCurrentPosition().x;
        double unnormalY = targetPose.y - Localizer.getCurrentPosition().y;
        double normalX = unnormalX / distance;
        double normalY = unnormalY / distance;
        double heading = Localizer.getCurrentPosition().h;
        double directionalX = normalX * Math.cos(heading) - normalY * Math.sin(heading);
        double directionalY = normalX * Math.sin(heading) - normalY * Math.cos(heading);

        kinematics.setAngleOnly(directionalY, -directionalX, 0);
    }

    public static void drive(boolean fc, double x, double y, double z){
        if(fc){
            kinematics.setFC(x, y, z);
        }
        else{
            kinematics.set(x, y, z);
        }
    }

    public static void Update(){
        kinematics.updatePods();

        double headingRespectCoef = 1;
        switch (usedTrajectory){
            case FOLLOWINGPUREPERSUIT:
                RobotLog.ii("current purepersuit angle",String.valueOf(PurePersuitTrajectory.FromLinesGeneratePointToFollow()));
                setTargetPosition(PurePersuitTrajectory.FromLinesGeneratePointToFollow());
                headingRespectCoef = PurePersuitTrajectory.GetHeadingRespectCoef();
                break;
            case OFF:
                kinematics.set(0,0,0);
                return;
            case USENOTHING:
                return;
            case DIRECTPATH:
                double distance = Localizer.getDistanceFromTwoPoints(Localizer.getCurrentPosition(), getTargetPosition());
                if(distance<=20){
                    //SlowPids();
                    usedTrajectory = trajectoryStates.FREEWILL;
                }
                double unnormalX = getTargetPosition().x - Localizer.getCurrentPosition().x;
                double unnormalY = getTargetPosition().y - Localizer.getCurrentPosition().y;
                double normalX = unnormalX / distance;
                double normalY = unnormalY / distance;
                double robotVelocity = sqrt(Localizer.getVelocity().x * Localizer.getVelocity().x + Localizer.getVelocity().y * Localizer.getVelocity().y);
                double brakeDist = robotVelocity / breakingConstant;
                double heading = Localizer.getCurrentPosition().h;
                double directionalX = normalX * Math.cos(heading) - normalY * Math.sin(heading);
                double directionalY = normalX * Math.sin(heading) - normalY * Math.cos(heading);
                if(distance > brakeDist){
                    kinematics.set(directionalY, -directionalX, 0);
                }else kinematics.set(-directionalY, directionalX, 0);
                return;

        }

        SparkFunOTOS.Pose2D normal = new SparkFunOTOS.Pose2D(
                getTargetPosition().x - Localizer.getCurrentPosition().x,
                getTargetPosition().y - Localizer.getCurrentPosition().y,
                getTargetPosition().h - Localizer.getCurrentPosition().h);

        double Herror = Localizer.normalizeRadians(normal.h);
        double h = Localizer.getCurrentPosition().h;
        while(h < 0) h += Math.PI * 2;
        while(h > 2*Math.PI) h -= Math.PI * 2;
        SparkFunOTOS.Pose2D error = new SparkFunOTOS.Pose2D(
                Math.cos(h) * normal.x + Math.sin(h) * normal.y,
                Math.sin(h) * normal.x - Math.cos(h) * normal.y,
                Herror
        );

        if(Math.abs(error.x)<5 && Math.abs(error.y)<5 && Math.abs(error.h)<1){
            kinematics.set(0,0,0);
            return;
        }

        double yP = Forward.calculatePower(error.y);
        double xP = Strafe.calculatePower(error.x);
        double hP = Heading.calculatePower(error.h);

        //RobotInitializers.Dashtelemetry.addData("xError", error.x);
        //RobotInitializers.Dashtelemetry.addData("yError", error.y);

        RobotInitializers.Dashtelemetry.addData("DiffH",getTargetPosition().h - Localizer.getCurrentPosition().h);
        RobotInitializers.Dashtelemetry.addData("CurrH",Localizer.getCurrentPosition().h);
        RobotInitializers.Dashtelemetry.addData("targetH",getTargetPosition().h);
        RobotInitializers.Dashtelemetry.addData("hError", Math.toDegrees(error.h));
        double p = 1;
        /*if(RobotInitializers.VOLTAGE > 12.8){
            p *= 12.8 / RobotInitializers.VOLTAGE;
        }*/


        //RobotInitializers.Dashtelemetry.addData("x power", yP * p);
        //RobotInitializers.Dashtelemetry.addData("y power", -xP * p);
        RobotInitializers.Dashtelemetry.addData("h power", hP * p);

        double importanceOfX = 1;
        double importanceOfY = 1;

        if(usedTrajectory == trajectoryStates.FOLLOWINGPUREPERSUIT && headingRespectCoef != 1){
            double MAX_ANGLE = Math.toRadians(180.0);

            double headingError = Math.abs(
                    Localizer.getAngleDifference(
                            getTargetPosition().h,
                            Localizer.getCurrentPosition().h
                    )
            );

            //RobotInitializers.Dashtelemetry.addData("headingError",headingError);

            double procentile = 1 - headingError / MAX_ANGLE;
            procentile = Math.min(1.0, procentile);

            //RobotInitializers.Dashtelemetry.addData("procentile",procentile);

            double translationScale = 1.0 - headingRespectCoef;

            importanceOfX = procentile * translationScale;
            importanceOfY = procentile * translationScale;
        }


        //RobotInitializers.Dashtelemetry.addData("impotanceOfX",importanceOfX);
        //RobotInitializers.Dashtelemetry.addData("impotanceOfY",importanceOfY);

        kinematics.set(yP * p * importanceOfY, -xP * p * importanceOfX, hP * p);



    }


}
