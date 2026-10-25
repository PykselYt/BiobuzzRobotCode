package org.firstinspires.ftc.teamcode.Biobuzz.MathHelpers;

import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.toDegrees;
import static java.lang.Math.toRadians;

public class ExtendedMaths {

    public static double[] precalculatedSin;
    public static double[] precalculatedCos;

    public static void InitializeMathematics(){

        precalculatedSin = new double[36001];
        precalculatedCos = new double[36001];

        for(int i = 0; i <= 36000; i++){
            precalculatedSin[i] = sin(toRadians(i/100.0));
            precalculatedCos[i] = cos(toRadians(i/100.0));
        }
    }

    public static double fastSin(double degrees){
        degrees = toDegrees(degrees);
        degrees = degrees % 360;
        if (degrees < 0) degrees += 360;
        return precalculatedSin[(int)(degrees*100)];
    }

    public static double fastCos(double degrees){
        degrees = toDegrees(degrees);
        degrees = degrees % 360;
        if (degrees < 0) degrees += 360;
        return precalculatedCos[(int)(degrees*100)];
    }

    public static double fastAtan2(double y, double x) {
        double yonx = y/x;
        double xony = x/y;

        if (x == 0.0)
            return y > 0.0 ? Math.PI / 2.0 :
                    y < 0.0 ? -Math.PI / 2.0 : 0.0;

        double angle;

        if (Math.abs(y) <= Math.abs(x)) {
            double ax = Math.abs(yonx);
            angle = Math.PI / 4.0 * (yonx)
                    - (yonx) * (ax - 1.0)
                    * (0.2447 + 0.0663 * ax);

            if (x < 0.0)
                angle += y >= 0.0 ? Math.PI : -Math.PI;
        } else {
            double ax = Math.abs(xony);
            angle = Math.PI / 2.0 -
                    (Math.PI / 4.0 * (xony)
                            - (xony) * (ax - 1.0)
                            * (0.2447 + 0.0663 * ax));

            if (y < 0.0)
                angle -= Math.PI;
        }

        return angle;
    }

}
