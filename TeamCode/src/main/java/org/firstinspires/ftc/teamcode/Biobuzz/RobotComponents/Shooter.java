package org.firstinspires.ftc.teamcode.Biobuzz.RobotComponents;

import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.Biobuzz.MathHelpers.PIDController;
import org.firstinspires.ftc.teamcode.Biobuzz.Wrapers.CachedMotor;
import org.firstinspires.ftc.teamcode.Biobuzz.Wrapers.ServoPlus;

public class Shooter {


    public static CachedMotor Launcher1,Launcher2;

    public static CachedMotor FlyWheelEncoder;

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


    public static double Fs = 0.03;
    public static double Kv = 0.000175;
    public static double Ka = 0;

    private static final PIDCoefficients AGGR_PID =
            new PIDCoefficients(0.0015, 0, 0);

    private static final PIDCoefficients SOFT_PID =
            new PIDCoefficients(0.0006, 0, 0);

    public void updateFlyWheelSpeed(){

        LauncherSpeedController.setPidCoefficients(
                RPMError(300) ? SOFT_PID : AGGR_PID);

        if(LauncherSpeedController.getTargetPosition() < 1000)
        {
            Launcher1.setPower(Fs);
            Launcher2.setPower(Fs);
            return;
        }
        double rpm = FlyWheelEncoder.getRPM();
        RobotInitializers.Dashtelemetry.addData("rpm shooter",rpm);
        double LaunchSpeed = LauncherSpeedController.calculatePower(rpm);
        LaunchSpeed += Kv * LauncherSpeedController.getTargetPosition() + Math.signum(LauncherSpeedController.getTargetPosition() - rpm) * Fs;
        setPowerEquilibrated(LaunchSpeed);
    }

    public double lastPos = 0.5;
    public static double procent = 1;

    public void UpdateTurretAndPitch(){

        ShooterCalculator.ShootData data = FormulaCalculator.CalculateShootData(RPMError());

        SetTargetRPM(data.wheelRPM);
        if(!Storage.putaBlock){
            Pitch.setAngle(FormulaCalculator.NeededServoPosForPitchGiven(data.hoodAngleDegrees));
        }else{
            Pitch.setAngle(Storage.blockPos);
        }

    }

    private double manualRPM = 0,manualHood = 0;

    public void SetManualData(double rpm,double hood){
        manualHood = hood;
        //ShooterCalculator.usedHood = hood;
        manualRPM = rpm;
    }
    public void UpdateManualData() {
        SetTargetRPM(manualRPM);
        double new_hood = manualHood;
        if (ShooterCalculator.shouldCompensateHood) {
            new_hood = FormulaCalculator.CompensateRPMSagWitchHood(
                    RPMError(),
                    manualHood,
                    ShooterCalculator.RPMErrorHoodCompensation
            );
        }

        // Set servo angle using degrees
        Pitch.setAngle(FormulaCalculator.NeededServoPosForPitchGiven(new_hood));
        RobotInitializers.Dashtelemetry.addData("compensatedHood", FormulaCalculator.NeededServoPosForPitchGiven(new_hood));
    }

    public void SetAngleToHood(double hood){
        Pitch.setAngle(FormulaCalculator.NeededServoPosForPitchGiven(hood));
    }




    public void SetTargetRPM(double targetRPM){
        LauncherSpeedController.setTargetPosition(targetRPM);
    }

    public boolean RPMError(double RPMerror){
        return Math.abs(LauncherSpeedController.getTargetPosition() - FlyWheelEncoder.getRPM()) < RPMerror;
    }

    public double RPMError(){
        return LauncherSpeedController.getTargetPosition() - FlyWheelEncoder.getRPM();
    }


}
