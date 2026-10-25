package org.firstinspires.ftc.teamcode.Biobuzz.MathHelpers;

import com.acmerobotics.dashboard.config.Config;

@Config
public class LinearFunction {
    public static double getOutput(double pstart,double pend,double estart, double eend,double t){
        if(eend - estart == 0)
            return 0;
        return pstart + (t - estart) * (pend - pstart) / (eend - estart);
    }
}