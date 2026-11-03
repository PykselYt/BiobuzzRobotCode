package org.firstinspires.ftc.teamcode.Biobuzz.SwerveTest;

import static org.firstinspires.ftc.teamcode.Biobuzz.MathHelpers.ExtendedMaths.fastAtan2;
import static org.firstinspires.ftc.teamcode.Biobuzz.MathHelpers.ExtendedMaths.fastCos;
import static org.firstinspires.ftc.teamcode.Biobuzz.MathHelpers.ExtendedMaths.fastSin;
import static java.lang.Math.PI;
import static java.lang.Math.atan2;
import static java.lang.Math.hypot;
import static java.lang.Math.max;
import static java.lang.Math.sqrt;
import static java.lang.Math.toDegrees;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Biobuzz.RobotComponents.Localizer;
import org.firstinspires.ftc.teamcode.Biobuzz.RobotComponents.RobotInitializers;
import org.firstinspires.ftc.teamcode.Biobuzz.Teleops.TeleopsStarter;

@Config
public class SwerveKinematics {
    public static SwervePod frontLeftModule, backLeftModule, backRightModule, frontRightModule;
    /*public SwerveModule[] modules;*/

    public static double TRACK_WIDTH = 260.222, WHEEL_BASE = 260.424;
    public static double TRACK_OVER_R, WHEEL_OVER_R;
    private final double R;

    public static boolean maintainHeading = false;

    double[] ws = new double[4];
    double[] wa = new double[4];

    private boolean locked = false;

    private static ElapsedTime time = new ElapsedTime();
    private static double targetHeading = 0;
    public static double holdDelay = 0.3;
    public static double holdP = 0.3;

    public SwerveKinematics() {

        /*modules = new SwerveModule[]{frontLeftModule, frontRightModule, backRightModule, backLeftModule};
        for (SwerveModule m : modules) m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);*/
        R = hypot(TRACK_WIDTH, WHEEL_BASE);
        TRACK_OVER_R = TRACK_WIDTH/R;
        WHEEL_OVER_R = WHEEL_BASE/R;

    }


    public void setAngleOnly(double x, double y, double r){

        double a = x - r * (WHEEL_OVER_R),
                b = x + r * (WHEEL_OVER_R),
                c = y - r * (TRACK_OVER_R),
                d = y + r * (TRACK_OVER_R);

        if (locked) {
            ws[0]=ws[1]=ws[2]=ws[3]=0;
            wa[0] = Math.PI / 4;
            wa[1] = -Math.PI / 4;
            wa[2] = Math.PI / 4;
            wa[3] = -Math.PI / 4;
        } else {
            double a_sqrt = a*a;
            double b_sqrt = b*b;
            double c_sqrt = c*c;
            double d_sqrt = d*d;
            ws[0] = Math.sqrt(b_sqrt+c_sqrt);
            ws[1] = Math.sqrt(b_sqrt+d_sqrt);
            ws[2] = Math.sqrt(a_sqrt+d_sqrt);
            ws[3] = Math.sqrt(a_sqrt+c_sqrt);
            //ws = new double[]{hypot(b, c), hypot(b, d), hypot(a, d), hypot(a, c)};
            if ((x != 0 || y != 0 || r != 0) && !maintainHeading) wa = new double[]{fastAtan2(b, c), fastAtan2(b, d), fastAtan2(a, d), fastAtan2(a, c)};
        }


        frontLeftModule.set(toDegrees(wa[1]),0);
        frontRightModule.set(toDegrees(wa[2]),0);
        backRightModule.set(toDegrees(wa[3]),0);
        backLeftModule.set(toDegrees(wa[0]),0);

        /*for(int i=0;i<4;i++)
            RobotInitializers.Dashtelemetry.addData("power " + i, ws[i]/diminuator);*/
        /*for(int i=0;i<4;i++)
            RobotInitializers.Dashtelemetry.addData("angle " + i, Math.toDegrees(wa[i]));*/
    }

    public void set(double x,double y,double r) {

        double a = x - r * (WHEEL_OVER_R),
                b = x + r * (WHEEL_OVER_R),
                c = y - r * (TRACK_OVER_R),
                d = y + r * (TRACK_OVER_R);

        if (locked) {
            ws[0]=ws[1]=ws[2]=ws[3]=0;
            wa[0] = Math.PI / 4;
            wa[1] = -Math.PI / 4;
            wa[2] = Math.PI / 4;
            wa[3] = -Math.PI / 4;
        } else {
            double a_sqrt = a*a;
            double b_sqrt = b*b;
            double c_sqrt = c*c;
            double d_sqrt = d*d;
            ws[0] = Math.sqrt(b_sqrt+c_sqrt);
            ws[1] = Math.sqrt(b_sqrt+d_sqrt);
            ws[2] = Math.sqrt(a_sqrt+d_sqrt);
            ws[3] = Math.sqrt(a_sqrt+c_sqrt);
            //ws = new double[]{hypot(b, c), hypot(b, d), hypot(a, d), hypot(a, c)};
            if ((x != 0 || y != 0 || r != 0) && !maintainHeading) wa = new double[]{fastAtan2(b, c), fastAtan2(b, d), fastAtan2(a, d), fastAtan2(a, c)};
        }

        /*
        double a_sqrt = a*a;
            double b_sqrt = b*b;
            double c_sqrt = c*c;
            double d_sqrt = d*d;
            ws[0] = Math.sqrt(b_sqrt+d_sqrt);
            ws[1] = Math.sqrt(a_sqrt+d_sqrt);
            ws[2] = Math.sqrt(a_sqrt+c_sqrt);
            ws[3] = Math.sqrt(b_sqrt+c_sqrt);
            //ws = new double[]{hypot(b, c), hypot(b, d), hypot(a, d), hypot(a, c)};
            if ((x != 0 || y != 0 || r != 0) && !maintainHeading) wa = new double[]{fastAtan2(b, d), fastAtan2(a, d), fastAtan2(a, c), fastAtan2(b, c)};
         */

        double diminuator = max(max(max(ws[2],ws[3]),max(ws[1],ws[0])),1);

        frontLeftModule.set(toDegrees(wa[0]),ws[0]/diminuator);
        frontRightModule.set(toDegrees(wa[1]),ws[1]/diminuator);
        backRightModule.set(toDegrees(wa[2]),ws[2]/diminuator);
        backLeftModule.set(toDegrees(wa[3]),ws[3]/diminuator);

        /*for(int i=0;i<4;i++)
            RobotInitializers.Dashtelemetry.addData("power " + i, ws[i]/diminuator);*/
        /*for(int i=0;i<4;i++)
            RobotInitializers.Dashtelemetry.addData("angle " + i, Math.toDegrees(wa[i]));*/
    }


    public void resetHoldHeading(){targetHeading = Localizer.getCurrentPosition().h;}
    public void setFC(double x, double y, double r){



        double heading = Localizer.getCurrentPosition().h;

        if(r != 0 ){
            time.reset();
            targetHeading = heading;
        }else if(time.seconds() < holdDelay){
            targetHeading = heading;
        }else{
            double err = targetHeading - heading;
            if(err > Math.PI) err -= 2* Math.PI;
            if(err < -Math.PI) err += 2 * Math.PI;
            r = Math.max(-0.5, Math.min(0.5, holdP * err));
        }

        heading += Math.toRadians(TeleopsStarter.AngleOffsetForDriving);

        RobotInitializers.Dashtelemetry.addData("heading", heading);
        RobotInitializers.Dashtelemetry.addData("targetHeading", targetHeading);
        RobotInitializers.Dashtelemetry.addData("holdR", r);

        //heading += TeleopsStarter.AngleOffsetForDriving;

        double nx = x * fastCos(heading) - y * fastSin(heading),ny = x * fastSin(heading) + y* fastCos(heading);


        double a = nx - r * (WHEEL_OVER_R),
                b = nx + r * (WHEEL_OVER_R),
                c = ny - r * (TRACK_OVER_R),
                d = ny + r * (TRACK_OVER_R);

        if (locked) {
            ws[0]=ws[1]=ws[2]=ws[3]=0;
            wa[0] = PI / 4;
            wa[1] = -PI / 4;
            wa[2] = PI / 4;
            wa[3] = -PI / 4;
        } else {
            double a_sqrt = a*a;
            double b_sqrt = b*b;
            double c_sqrt = c*c;
            double d_sqrt = d*d;
            ws[0] = sqrt(b_sqrt+c_sqrt);
            ws[1] = sqrt(b_sqrt+d_sqrt);
            ws[2] = sqrt(a_sqrt+d_sqrt);
            ws[3] = sqrt(a_sqrt+c_sqrt);
            //ws = new double[]{hypot(b, c), hypot(b, d), hypot(a, d), hypot(a, c)};

            if ((x != 0 || y != 0 || r != 0) && !maintainHeading) {
                wa[0] = fastAtan2(b, c);
                wa[1] = fastAtan2(b, d);
                wa[2] = fastAtan2(a, d);
                wa[3] = fastAtan2(a, c);
            }
        }

        double diminuator = max(max(max(ws[2],ws[3]), max(ws[1],ws[0])),1);

        frontLeftModule.set(toDegrees(wa[0]),ws[0]/diminuator);
        frontRightModule.set(toDegrees(wa[1]),ws[1]/diminuator);
        backRightModule.set(toDegrees(wa[2]),ws[2]/diminuator);
        backLeftModule.set(toDegrees(wa[3]),ws[3]/diminuator);
    }

    public void updatePods(){
        double FLError = frontLeftModule.getError();
        double FRError = frontRightModule.getError();
        double BLError = backLeftModule.getError();
        double BRError = backRightModule.getError();

        double mult = Math.max(0, Math.cos(Math.toRadians(Math.max(Math.max(FLError,FRError), Math.max(BLError, BRError)))));
        mult*=1.5;
        if(mult>0.9) mult = 1;

        frontLeftModule.update(mult);
        frontRightModule.update(mult);
        backLeftModule.update(mult);
        backRightModule.update(mult);

        /*
        RobotInitializers.Dashtelemetry.addData( "FL amp", frontLeftModule.motor.getCurrent(CurrentUnit.AMPS));
        RobotInitializers.Dashtelemetry.addData( "FR amp", frontRightModule.motor.getCurrent(CurrentUnit.AMPS));
        RobotInitializers.Dashtelemetry.addData( "BL amp", backLeftModule.motor.getCurrent(CurrentUnit.AMPS));
        RobotInitializers.Dashtelemetry.addData( "BR amp", backRightModule.motor.getCurrent(CurrentUnit.AMPS));
        */
    }


    public void setLocked(boolean locked){
        this.locked = locked;
    }

    public boolean isLocked(){
        return locked;
    }

}