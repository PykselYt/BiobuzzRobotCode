package org.firstinspires.ftc.teamcode.Biobuzz.RobotComponents;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.teamcode.Biobuzz.MathHelpers.LinearFunction;

@Config
public class ShooterCalculator {



    public static double FlywheelDiameter = 0.076;

    public static double PosMaxPitch = 80, LowestPitch = 65.659;
    public static double PosMinPitch = 146.7, HighestPitch = 79.4;
    public static double pitchOffset = 80;

    public static double TurretOffsetStatic = 0;

    public static SparkFunOTOS.Pose2D goalPosition1 = new SparkFunOTOS.Pose2D(1200,-500,0), goalPosition2 = new SparkFunOTOS.Pose2D(2000,-500,0);
    public static double halfFieldDist = 1600;
    public static boolean usedPos1 = true;

    public double NeededServoPosForPitchGiven(double pitch){
        //double angle = (1.57621 * Math.pow(10, -7)) * Math.pow(pitch, 4) - 0.0000630077 * Math.pow(pitch, 3) + 0.00725814 * pitch * pitch - 0.0445644 * pitch + 65.659;
        double angle = 0.0008724202 * pitch * pitch * pitch * pitch  - 0.2466545 * pitch * pitch * pitch + 25.82175 * pitch * pitch - 1179.99294 * pitch + 19761.1581;
        angle += pitchOffset;
        double lo = Math.min(PosMinPitch, PosMaxPitch);
        double hi = Math.max(PosMinPitch, PosMaxPitch);
        return Math.max(lo, Math.min(angle, hi));
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





    public double CompensateRPMSagWitchHood(double ErrorRPM,double currHood,double modifier){
        return currHood + ErrorRPM * modifier;
    }


    public static class ShootData{
        public double hoodAngleDegrees,YawOffset,wheelRPM;
        public boolean canShoot = true;
        public ShootData(double hoodAngleDegrees,double YawOffset,double wheelRPM){
            this.hoodAngleDegrees = hoodAngleDegrees;
            this.YawOffset = YawOffset;
            this.wheelRPM = wheelRPM;
        }
    }

    private static final double g = 9.81;
    public static boolean shouldCompensateHood = false;


    public static double h = 1.27; //0.243
    public static double alfa = 30;
    public static double e = 0.65;

    public static double lastValidRPM = 0;
    public static double MaxFlywheelRPM = 3000;

    public ShootData CalculateFormula(double ErrorRPM){
        SparkFunOTOS.Pose2D pos = Localizer.getCurrentPosition();

        double dx = Turret.goalPosition.x - pos.x;
        double dy = Turret.goalPosition.y - pos.y;
        double distanceToGoal = Math.hypot(dx, dy) / 1000;

        //unghi ideal
        double thetaIdeal = Math.atan((2 * h / distanceToGoal) + Math.tan(Math.toRadians(alfa)));

        //clamp hood
        double hood_angle = Math.max(Math.toRadians(LowestPitch),
                Math.min(thetaIdeal, Math.toRadians(HighestPitch)));
        boolean hoodAtLimit = hood_angle != thetaIdeal;

        //rpm
        double denom = distanceToGoal * Math.tan(hood_angle) - h;
        boolean canShoot = denom > 0;
        double flywheelRPM = lastValidRPM;

        if (canShoot) {
            double ball_speed = distanceToGoal / Math.cos(hood_angle) * Math.sqrt(g / (2 * denom));
            flywheelRPM = (ball_speed * 60) / (e * Math.PI * FlywheelDiameter);
            if (flywheelRPM > MaxFlywheelRPM) {
                flywheelRPM = MaxFlywheelRPM;
                canShoot = false;
            }
            lastValidRPM = flywheelRPM;
        }

        //compensare
        double newHood = Math.toDegrees(hood_angle);
        if (shouldCompensateHood && canShoot) {
            double measuredRPM = flywheelRPM - ErrorRPM;
            newHood = CompensateHoodForRPM(measuredRPM, flywheelRPM, distanceToGoal, newHood);
            newHood = Math.max(LowestPitch, Math.min(newHood, HighestPitch));
        }

        RobotInitializers.Dashtelemetry.addData("thetaIdeal", Math.toDegrees(thetaIdeal));
        RobotInitializers.Dashtelemetry.addData("hoodAtLimit", hoodAtLimit);
        RobotInitializers.Dashtelemetry.addData("canShoot", canShoot);
        RobotInitializers.Dashtelemetry.addData("compensatedCalculatedHood", newHood);

        ShootData data = new ShootData(newHood, 0, flywheelRPM);
        data.canShoot = canShoot;
        return data;
    }

    public static double CompDeadbandRPM = 100;
    public static double RPMFilterGain = 1;
    private double filteredRPM = 0;

    public double CompensateHoodForRPM(double measuredRPM, double targetRPM,
                                       double distM, double nominalHoodDeg) {
        filteredRPM += RPMFilterGain * (measuredRPM - filteredRPM);

        if (Math.abs(targetRPM - filteredRPM) < CompDeadbandRPM) return nominalHoodDeg;

        double v  = filteredRPM * e * Math.PI * FlywheelDiameter / 60.0;
        double v2 = v * v;
        double delta = v2 * v2 - g * (g * distM * distM + 2 * h * v2);

        if (delta < 0) {
            return Math.toDegrees(Math.PI / 4 + Math.atan2(h, distM) / 2);
        }

        double low  = Math.toDegrees(Math.atan((v2 - Math.sqrt(delta)) / (g * distM)));
        double high = Math.toDegrees(Math.atan((v2 + Math.sqrt(delta)) / (g * distM)));

        return Math.abs(low - nominalHoodDeg) < Math.abs(high - nominalHoodDeg) ? low : high;
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
