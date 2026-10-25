package org.firstinspires.ftc.teamcode.Biobuzz.RobotComponents;

import org.firstinspires.ftc.teamcode.Biobuzz.Wrapers.CachedMotor;

public class Storage {

    public static CachedMotor motorfata,motorspate;

    public static boolean putaBlock = true;
    public static double blockPos = 0;

    public void takein(double power){
        motorfata.setPower(power);
        motorspate.setPower(power);
    }

    public void takeIn(){
        takein(1);
    }
    public void spit(){takein(-1);}
    public void stop(){takein(0);}

    public void unlockPuta(){putaBlock = false;}
    public void lockPuta(){putaBlock = true;}

}
