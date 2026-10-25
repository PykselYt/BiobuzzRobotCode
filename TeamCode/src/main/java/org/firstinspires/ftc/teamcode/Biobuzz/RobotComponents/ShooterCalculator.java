package org.firstinspires.ftc.teamcode.Biobuzz.RobotComponents;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.teamcode.Biobuzz.MathHelpers.LinearFunction;

public class ShooterCalculator {



    public static double FlywheelDiameter = 76.2;

    public static double PosMaxPitch = 80, LowestPitch =
            53.9;
    public static double PosMinPitch = 320, HighestPitch = 28.6;

    public static double TurretOffsetStatic = 0;

    public static SparkFunOTOS.Pose2D goalPosition1 = new SparkFunOTOS.Pose2D(0,0,0), goalPosition2 = new SparkFunOTOS.Pose2D(0,0,0);
    public static double halfFieldDist = 3000;
    public static boolean usedPos1 = true;
    public static double usedHood = 0;

    public double NeededServoPosForPitchGiven(double pitch){
        double angle = LinearFunction.getOutput(PosMaxPitch,PosMinPitch,LowestPitch,HighestPitch,pitch);
        return Math.max(PosMaxPitch,Math.min(angle,PosMinPitch));
    }

    public double givePitch(double Angle){
        Angle = Math.max(LowestPitch,Math.min(Angle,HighestPitch));
        return NeededServoPosForPitchGiven(Angle);
    }

    public double FromBallVelocityToRPM(double velocity){
        return velocity * 60 / (Math.PI * FlywheelDiameter);
    }

    public double FromRPMToBallVelocity(double rpm){
        return rpm * Math.PI * FlywheelDiameter / 60.0;
    }

    public static double TurretOffsetMultiplier = -2.9;
    public static double CompensatedNewRPM = 0.4;
    public static double CompensatorNewHood = 0;

    public static double startangle = 41; // 56
    public static double endangle = 29;


    public static double startRPM = 2550;
    public static double endRPM = 4100;

    public static double startDist = 700;
    public static double endDist = 3622;


    public static double RPMErrorHoodCompensation = 0.01;



    public static double manualHoodAddition = 0;
    public static double manualRPMAddition = 0;


    public double CompensateRPMSagWitchHood(double ErrorRPM,double currHood,double modifier){
        return currHood + ErrorRPM * modifier;
    }


    public static double GetPrequisiteRPM(double dist){
        return LinearFunction.getOutput(startRPM,endRPM,startDist,endDist,dist);


        //return -0.0000461454 * Math.pow(dist,2) + 0.701809 * dist + 2285.76429;
    }

    public static double GetPrequisiteHood(double dist){
        return LinearFunction.getOutput(startangle,endangle,startDist,endDist,dist);
    }



    public static class ShootData{
        public double hoodAngleDegrees,YawOffset,wheelRPM;
        public ShootData(double hoodAngleDegrees,double YawOffset,double wheelRPM){
            this.hoodAngleDegrees = hoodAngleDegrees;
            this.YawOffset = YawOffset;
            this.wheelRPM = wheelRPM;
        }
    }

    /*
     *
     * x = V*cos(a)*t => t = x / (V * cos(a))
     * y = V*sin(a)*t - t^2 * g / 2
     * y = V * sin(a) * x / (V * cos(a)) - x^2 / (V^2 * cos^2(a)) / 2
     * y = x * tg(a) - x^2 / V^2 * (1 + tg^2(a))
     * ...
     * a = atan((V^2 - sqrt(V^4 - 2*V^2*g*y - g^2*x^2)) / (g * X))
     *
     *
     *
     * */
    private static final double g = 9.81;
    public double CalculateHoodBasedOnVelocity(double shooterVelocity, double y, double x){
        double delta = Math.pow(shooterVelocity, 4) - 2 * g * y * Math.pow(shooterVelocity, 2) - g * g * x * x;
        if(delta < 0) return 0; // low rpm
        double angle = Math.atan((shooterVelocity * shooterVelocity + Math.sqrt(delta)) / (g * x));

        return angle;

    }

    public static boolean clampDistance = true;

    public static double minX = 0, maxX = 3000;
    public static double minY = 0, maxY = 3000;

    public static boolean shouldCompensateHood = false;

    public ShootData CalculateShootData(double ErrorRPM){

        double clampedX = Localizer.getCurrentPosition().x;
        double clampedY = Localizer.getCurrentPosition().y;

        if(clampDistance){
            if(clampedX < minX) clampedX = minX;
            if(clampedX > maxX) clampedX = maxX;

            if(clampedY < minY) clampedY = minY;
            if(clampedY > maxY) clampedY = maxY;
        }


        double dx = Turret.goalPosition.x - clampedX;
        double dy = Turret.goalPosition.y - clampedY;

        double distanceToGoal = Math.hypot(dx, dy);

        RobotInitializers.Dashtelemetry.addData("calculatedDistanceToGoal", distanceToGoal);

        double hood_angle = Math.toRadians(GetPrequisiteHood(distanceToGoal) + manualHoodAddition);
        double flywheelRPM = GetPrequisiteRPM(distanceToGoal) + manualRPMAddition;

        /*if(distanceToGoal > DistanceToCancelCompensation){
            double newHoodWhileCompensating =  CompensateRPMSagWitchHood(ErrorRPM,Math.toDegrees(hood_angle),RPMErrorHoodCompensation);
            RobotInitializers.Dashtelemetry.addData("hood angle",newHoodWhileCompensating);
            return new ShootData(newHoodWhileCompensating,0,flywheelRPM);
        }*/

        double v_ball = FromRPMToBallVelocity(flywheelRPM);

        //RobotInitializers.Dashtelemetry.addData("flywheel normal",FromBallVelocityToRPM(v_ball));
        //RobotInitializers.Dashtelemetry.addData("hood_angle",Math.toDegrees(hood_angle));

        Localizer.VectorDescriptor vectorMotion = Localizer.GetVelocityVectorOfRobot();

// Goal direction

        double line = Math.atan2(dy, dx);

        RobotInitializers.Dashtelemetry.addData("line",line);
// Optional: Add an offset if needed (e.g., if robot forward is at 90 degrees)
// double robotAngleOffset = Math.PI/2; // Example offset
// double adjustedLine = line - robotAngleOffset;

// Calculate components (without negative sign)
        double delta = vectorMotion.orientation - line;

        RobotInitializers.Dashtelemetry.addData("delta",delta);


        double towardsComponent = -Math.cos(delta) * vectorMotion.value;
        double sidewaysComponent = Math.sin(delta) * vectorMotion.value;

        RobotInitializers.Dashtelemetry.addData("towardsComponent",towardsComponent);
        RobotInitializers.Dashtelemetry.addData("sidewaysComponent",sidewaysComponent);

        double vballcompensatedX = v_ball * Math.cos(hood_angle) + towardsComponent;

        /*double newHood = hood_angle + -towardsComponent * CompensatorNewHood;
        double newRPM = flywheelRPM + -towardsComponent * CompensatedNewRPM;

        //double hoodCompensated = CompensateRPMSagWitchHood(ErrorRPM,Math.toDegrees(newHood),RPMErrorHoodCompensationWhileMoving);

        /*double vballY = v_ball * Math.sin(hood_angle);

        RobotInitializers.Dashtelemetry.addData("vballY = ", vballY);
        RobotInitializers.Dashtelemetry.addData("vballcompensatedX = ", vballcompensatedX);

        double timeFlight = distanceToGoal / (v_ball * Math.cos(hood_angle));

        double hood_new = Math.atan( vballY / vballcompensatedX);

        double VxBallElimCompensation = distanceToGoal / timeFlight + towardsComponent * ElimCompensatiorMultiplier;
        double VyBall = v_ball * Math.sin(hood_new);

        double NeededSlowedBall = Math.sqrt(VxBallElimCompensation * VxBallElimCompensation + VyBall * VyBall);
        double newRPM = FromBallVelocityToRPM(NeededSlowedBall);*/

        double OffsetTurret = Math.atan2(sidewaysComponent, vballcompensatedX);
        double newHoodWhileCompensating = Math.toDegrees(hood_angle);

        if(shouldCompensateHood)newHoodWhileCompensating =  CompensateRPMSagWitchHood(ErrorRPM,Math.toDegrees(hood_angle),RPMErrorHoodCompensation);




        //RobotInitializers.Dashtelemetry.addData("Difference in RPM = ", newRPM - flywheelRPM);
        //RobotInitializers.Dashtelemetry.addData("Difference in degrees hood", Math.toDegrees(newHood - hood_angle));

        TurretOffsetStatic = TurretOffsetMultiplier * OffsetTurret;

        return new ShootData(newHoodWhileCompensating,0,flywheelRPM);
    }

    public SparkFunOTOS.Pose2D GetPredictedPosition(double distanceToGoal, double flywheelRPM, double hoodRad) {
        SparkFunOTOS.Pose2D currentPos = Localizer.getCurrentPosition();
        Localizer.VectorDescriptor velocity = Localizer.GetVelocityVectorOfRobot();

        RobotInitializers.Dashtelemetry.addData("orientation",velocity.orientation);
        RobotInitializers.Dashtelemetry.addData("value",velocity.value);
        // Calculate time of flight
        double v_ball = FromRPMToBallVelocity(flywheelRPM);

        RobotInitializers.Dashtelemetry.addData("v_ball",v_ball);
        double horizontalVelocity = v_ball * Math.cos(hoodRad);

        RobotInitializers.Dashtelemetry.addData("horizontalVelocity",horizontalVelocity);
        double timeOfFlight = distanceToGoal / horizontalVelocity;

        RobotInitializers.Dashtelemetry.addData("timeOfFlight",timeOfFlight);
        // Calculate predicted position
        return new SparkFunOTOS.Pose2D(
                currentPos.x + velocity.value * Math.cos(velocity.orientation) * timeOfFlight,
                currentPos.y + velocity.value * Math.sin(velocity.orientation) * timeOfFlight,
                0
        );
    }
    //public static double goalHeight = 750;

}
