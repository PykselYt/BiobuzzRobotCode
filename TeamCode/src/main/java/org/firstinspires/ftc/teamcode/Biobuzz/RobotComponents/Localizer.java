package org.firstinspires.ftc.teamcode.Biobuzz.RobotComponents;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.Biobuzz.Wrapers.PinPoint;



public class Localizer {


    public static double X = -191.839, Y = 180.213, yawScalar = 1;//de vazut
    public static PinPoint.EncoderDirection xPod = PinPoint.EncoderDirection.REVERSED, yPod = PinPoint.EncoderDirection.FORWARD;
    public static PinPoint pinPoint;
    private static SparkFunOTOS.Pose2D velocity = new SparkFunOTOS.Pose2D();
    public static SparkFunOTOS.Pose2D lastPose = new SparkFunOTOS.Pose2D();

    private static ElapsedTime time = new ElapsedTime();



    /*---angle functions---*/
    public static double normalizeTo0To2PI(double raw){
        return (raw < 0) ? raw + 2*Math.PI : raw;
    }
    public static double normalizeRadians(double raw){
        while(raw > Math.PI) raw -= Math.PI * 2;
        while(raw < -Math.PI) raw += Math.PI * 2;
        return raw;
    }
    public static double getAngleDifference(double h1, double h2){
        h1 = Localizer.normalizeRadians(h1);
        h2 = Localizer.normalizeRadians(h2);
        return Math.min(Math.abs(h1 - h2), Math.PI * 2 - Math.abs(h1 - h2));
    }

    /*---position functions---*/
    public static double getDistanceFromTwoPoints(SparkFunOTOS.Pose2D p1, SparkFunOTOS.Pose2D p2){
        if(p1 == null) p1 = new SparkFunOTOS.Pose2D();
        if(p2 == null) p2 = new SparkFunOTOS.Pose2D();
        return Math.sqrt(
                (p1.x - p2.x) * (p1.x - p2.x) +
                        (p1.y - p2.y) * (p1.y - p2.y)
        );
    }


    /*---object functions---*/
    public static void Initialize(HardwareMap hm){
        pinPoint = hm.get(PinPoint.class, "pinpoint");
        pinPoint.setOffsets(X, Y, DistanceUnit.MM);
        pinPoint.setEncoderResolution(PinPoint.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinPoint.setYawScalar(yawScalar);
        pinPoint.setEncoderDirections(xPod, yPod);
//        pinPoint.resetPosAndIMU();
    }
    public static void Update(){
        pinPoint.update();
        /*if((System.currentTimeMillis() - imuUpdate) >= 500) {
            //RobotLog.i("resetRotation");
            double h = RobotInitializers.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            if(
                    getAngleDifference(0, h) <= Math.toRadians(0.03) &&
                            getAngleDifference(Math.PI, h) <= Math.toRadians(0.03) &&
                            getAngleDifference(-Math.PI, h) <= Math.toRadians(0.03) &&
                            getAngleDifference(Math.PI / 2, h) <= Math.toRadians(0.03) &&
                            getAngleDifference(-Math.PI / 2, h) <= Math.toRadians(0.03)
            ){
                while (h < 0) h += 2 * Math.PI;
                while (h > 2 * Math.PI) h -= Math.PI * 2;

                pinPoint.setPosition(new Pose2D(DistanceUnit.MM, getCurrentPosition().x, getCurrentPosition().y, AngleUnit.RADIANS, h));
                imuUpdate = System.currentTimeMillis();
            }
        }*/



        // RobotInitializers.Dashtelemetry.addData("pinpoint frequency", pinPoint.getFrequency());


        velocity = new SparkFunOTOS.Pose2D(getCurrentPosition().x - lastPose.x, getCurrentPosition().y - lastPose.y, pinPoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS));
        lastPose = getCurrentPosition();   //        velocity = Div(velocity, time.seconds());
        velocity.x /= time.seconds();
        velocity.y /= time.seconds();

        RobotInitializers.Dashtelemetry.addData("pose", "(" + getCurrentPosition().x + ", " + getCurrentPosition().y + ", " + Math.toDegrees(getCurrentPosition().h) + "deg)");
        time.reset();

    }




    /*---pinpoint functions---*/
    public static void Reset(){
        pinPoint.recalibrateIMU();
        pinPoint.resetPosAndIMU();
    }
    public static SparkFunOTOS.Pose2D getCurrentPosition(){
        double h = pinPoint.getHeading();
        h = normalizeRadians(h);
        return new SparkFunOTOS.Pose2D(pinPoint.getPosX(),pinPoint.getPosY(), h);
    }
    public static SparkFunOTOS.Pose2D getVelocity(){
        return new SparkFunOTOS.Pose2D(pinPoint.getVelX(DistanceUnit.MM), pinPoint.getVelY(DistanceUnit.MM), pinPoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS));
    }


    public static void setPosition(SparkFunOTOS.Pose2D humanTake) {
        pinPoint.setPosition(new Pose2D(DistanceUnit.MM, humanTake.x, humanTake.y, AngleUnit.RADIANS, humanTake.h));
        pinPoint.update();
    }

    public static void setHeading(double heading){
        pinPoint.setHeading(heading,AngleUnit.RADIANS);
        pinPoint.update();
    }


    /*---vector descriptor---*/
    public static class VectorDescriptor{
        public double value,orientation;
        public VectorDescriptor(double value,double orientation){
            this.value = value;
            this.orientation = orientation;
        }
        SparkFunOTOS.Pose2D returnCartheianCoords(){
            return new SparkFunOTOS.Pose2D(this.value * Math.cos(orientation), this.value * Math.sin(orientation), 0);
        }
    }

    public static VectorDescriptor GetVelocityVectorOfRobot(){
        SparkFunOTOS.Pose2D velocity = Localizer.getVelocity();
        double x = velocity.x;
        double y = velocity.y;
        return new VectorDescriptor(Math.sqrt(x*x + y*y),Math.atan2(y,x));
    }


}
