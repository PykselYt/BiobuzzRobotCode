package org.firstinspires.ftc.teamcode.Biobuzz.MathHelpers;

public class MathUtils {

    public static double[] PrecalSin;
    public static double[] PrecalCos;

    public static void PrecalcultateSinCos(){
        PrecalSin = new double[36001];
        PrecalCos = new double[36001];

        for (int i = 0; i <= 36000; i++) {
            double angle = Math.toRadians(i / 100.0);
            PrecalSin[i] = Math.sin(angle);
            PrecalCos[i] = Math.cos(angle);
        }
    }
    public static double fastAtan2(double y, double x) {
        if (x == 0.0)
            return y > 0.0 ? Math.PI / 2.0 :
                    y < 0.0 ? -Math.PI / 2.0 : 0.0;

        double angle;

        if (Math.abs(y) <= Math.abs(x)) {
            double ax = Math.abs(y / x);
            angle = Math.PI / 4.0 * (y / x)
                    - (y / x) * (ax - 1.0)
                    * (0.2447 + 0.0663 * ax);

            if (x < 0.0)
                angle += y >= 0.0 ? Math.PI : -Math.PI;
        } else {
            double ax = Math.abs(x / y);
            angle = Math.PI / 2.0 -
                    (Math.PI / 4.0 * (x / y)
                            - (x / y) * (ax - 1.0)
                            * (0.2447 + 0.0663 * ax));

            if (y < 0.0)
                angle -= Math.PI;
        }

        return angle;
    }



    public static void InitializePrecalculation(){
            PrecalcultateSinCos();
    }

}
