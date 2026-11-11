package org.firstinspires.ftc.teamcode.Biobuzz.RobotComponents;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Biobuzz.MathHelpers.PIDController;
import org.firstinspires.ftc.teamcode.Biobuzz.Wrapers.CachedMotor;
import org.firstinspires.ftc.teamcode.Biobuzz.Wrapers.ServoPlus;

@Config
public class Shooter {


    public static CachedMotor Launcher1,Launcher2;

    public static CachedMotor FlyWheelEncoder;

    public static double ratio = 20.0/31.0;

    public static ServoPlus Pitch;

    public PIDController LauncherSpeedController = new PIDController(0.0006,0,0);

    public ShooterCalculator FormulaCalculator;

    public Shooter(){
        FormulaCalculator = new ShooterCalculator();
    }


    public void setPowerEquilibrated(double power){
        double p = 12.8 / RobotInitializers.VOLTAGE;
        double powercomp = power * p;
        //RobotInitializers.Dashtelemetry.addLine("equilibrated power" + power * p);
        Launcher1.setPower(powercomp);
        Launcher2.setPower(powercomp);
    }


    public static double Fs = 0.177;
    public static double Kv = 0.00035;

    private static final PIDCoefficients AGGR_PID =
            new PIDCoefficients(0.0015, 0, 0);

    private static final PIDCoefficients SOFT_PID =
            new PIDCoefficients(0.0006, 0, 0);

    public void updateFlyWheelSpeed(){
        //Launcher1.setPower(0.85);


        LauncherSpeedController.setPidCoefficients(
                RPMError(200) ? SOFT_PID : AGGR_PID);

        if(LauncherSpeedController.getTargetPosition() < 500)
        {
            Launcher1.setPower(Fs);
            Launcher2.setPower(Fs);
            return;
        }
        double rpm = FlyWheelEncoder.getRPM()*ratio;
        RobotInitializers.Dashtelemetry.addData("rpm shooter",rpm);
        RobotInitializers.Dashtelemetry.addData("AMP 1",Launcher1.getCurrent(CurrentUnit.AMPS));
        RobotInitializers.Dashtelemetry.addData("AMP 2",Launcher2.getCurrent(CurrentUnit.AMPS));

        double LaunchSpeed = LauncherSpeedController.calculatePower(rpm);
        LaunchSpeed += Kv * LauncherSpeedController.getTargetPosition() + Math.signum(LauncherSpeedController.getTargetPosition() - rpm) * Fs;
        setPowerEquilibrated(LaunchSpeed);


    }

    public void UpdateTurretAndPitch(){

        ShooterCalculator.ShootData data = FormulaCalculator.CalculateFormula(RPMError());

        SetTargetRPM(data.wheelRPM);
        RobotInitializers.Dashtelemetry.addData("rpm calc", data.wheelRPM);
        RobotInitializers.Dashtelemetry.addData("actualHood", FormulaCalculator.NeededServoPosForPitchGiven(data.hoodAngleDegrees));
        if(!Storage.putaBlock){
            Pitch.setAngle(FormulaCalculator.NeededServoPosForPitchGiven(data.hoodAngleDegrees));
            //Pitch.setAngle(150);
        }else{
            Pitch.setAngle(Storage.blockPos);
        }

    }


    public void SetAngleToHood(double hood){
        Pitch.setAngle(FormulaCalculator.NeededServoPosForPitchGiven(hood));
    }




    public void SetTargetRPM(double targetRPM){
        LauncherSpeedController.setTargetPosition(targetRPM);
    }

    public boolean RPMError(double RPMerror){
        return Math.abs(LauncherSpeedController.getTargetPosition() - FlyWheelEncoder.getRPM()*ratio) < RPMerror;
    }

    public double RPMError(){
        return LauncherSpeedController.getTargetPosition() - FlyWheelEncoder.getRPM()*ratio;
    }


}
