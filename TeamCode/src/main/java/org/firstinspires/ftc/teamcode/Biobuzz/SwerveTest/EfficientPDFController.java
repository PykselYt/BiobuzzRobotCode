package org.firstinspires.ftc.teamcode.Biobuzz.SwerveTest;

import com.qualcomm.robotcore.util.ElapsedTime;

public class EfficientPDFController {

    public double kP, kD, kF;
    public double freq = 100;

    private double target;
    private double lastError;
    private double lastOutput;

    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime updateTimer = new ElapsedTime();

    public EfficientPDFController(double kP, double kD, double kF) {
        this.kP = kP;
        this.kD = kD;
        this.kF = kF;
    }

    public double calculate(double current) {
        if (updateTimer.seconds() < 1.0 / freq)
            return lastOutput;

        updateTimer.reset();

        double dt = timer.seconds();
        timer.reset();

        double error = target - current;
        double derivative = dt > 0 ? (error - lastError) / dt : 0;

        lastError = error;

        lastOutput = kP * error
                + kD * derivative
                + kF * Math.signum(error);

        return lastOutput;
    }

    public void setTarget(double target) {
        this.target = target;
    }

    public void setFreq(double freq) {
        this.freq = freq;
    }

    public void setPDFCoefficients(double kP, double kD, double kF){
        this.kP = kP;
        this.kD = kD;
        this.kF = kF;
    }

    public void reset() {
        lastError = 0;
        lastOutput = 0;
        timer.reset();
        updateTimer.reset();
    }
}