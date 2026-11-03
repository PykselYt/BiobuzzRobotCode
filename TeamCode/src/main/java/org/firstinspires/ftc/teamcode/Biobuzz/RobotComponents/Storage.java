package org.firstinspires.ftc.teamcode.Biobuzz.RobotComponents;

import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.Biobuzz.Wrapers.CachedMotor;
import org.firstinspires.ftc.teamcode.Biobuzz.Wrapers.ServoPlus;

public class Storage {

    public static CachedMotor motorfata,motorspate;
    public static ServoPlus ramp;

    public static DigitalChannel bb1, bb2, bb3, bb4;

    public static double upPos=15, downPos=275;
    private static boolean rampDown = false;

    public static boolean putaBlock = true;
    public static double blockPos = 355;

    public void takeIn(double power){
        motorfata.setPower(power);
        motorspate.setPower(power);
    }

    public void senzorTakeIn(double power){
        if(!check4()) {
            motorfata.setPower(power);
            motorspate.setPower(power);
        }
        else {
            motorfata.setPower(0);
            motorspate.setPower(0);
        }

    }

    public void idle(){
        motorspate.setPower(0);
        if(check4()){
            motorfata.setPower(0);
        }else motorfata.setPower(0);
    }

    public void takeIn(){senzorTakeIn(1);}
    public void spit(){takeIn(-1);}
    public void stop(){takeIn(0);}
    public void transfer(){takeIn(1);}



    public void unlockPuta(){putaBlock = false;}
    public void lockPuta(){putaBlock = true;}


    public void switchRamp(){
        if(rampDown){
            rampDown = false;
            ramp.setAngle(upPos);
        }else{
            rampDown = true;
            ramp.setAngle(downPos);
        }
    }

    public boolean check4(){
        return !bb1.getState() && !bb2.getState() && !bb3.getState() && !bb4.getState();
    }

}
