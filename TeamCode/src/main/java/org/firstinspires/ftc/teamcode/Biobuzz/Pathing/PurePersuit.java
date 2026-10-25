package org.firstinspires.ftc.teamcode.Biobuzz.Pathing;

import static org.firstinspires.ftc.teamcode.Biobuzz.RobotComponents.Chassis.getTargetPosition;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.teamcode.Biobuzz.RobotComponents.Localizer;

import java.util.ArrayList;
import java.util.List;

public class PurePersuit {


    List<Line> Path = new ArrayList<>();
    int indLine = 0;
    double ChosenRadius = 0;
    double LastLineHeading = 0;
    public double StartHeading;

    private class Circle {
        public double x, y, r;

        public Circle(double x, double y, double r) {
            this.x = x;
            this.y = y;
            this.r = r;
        }

        @NonNull
        @Override
        public String toString(){
            return "x: " + x + "y: " + y + "radius: " + r;
        }
    }

    public static class Point {
        public double x, y, HeadingRespectCoef,h;

        public Point(double x, double y) {
            this.x = x;
            this.y = y;
            this.HeadingRespectCoef = 0;
        }

        public Point(double x, double y,double HeadingRespectCoef){
            this.x = x;
            this.y = y;
            this.HeadingRespectCoef = HeadingRespectCoef;
        }

        public Point(double x, double y,double h,double HeadingRespectCoef) {
            this.x = x;
            this.y = y;
            this.h = h;
            this.HeadingRespectCoef = HeadingRespectCoef;
        }

        @NonNull
        @Override
        public String toString(){
            return "x: " + x + "y: " + y;
        }
    }

    private class Line {
        public Point pstart, pend;

        public Line(Point pstart, Point pend) {
            this.pend = pend;
            this.pstart = pstart;
        }
    }

    public PurePersuit(ArrayList<Point> Points,double startRadius){//in rads please
        BuildPath(Points,startRadius);
        this.StartHeading = Localizer.getCurrentPosition().h;
    }

    /*public SparkFunOTOS.Pose2D getTargetPosition(){
        Point end = Path.get(Path.size()-1).pend;
        return new SparkFunOTOS.Pose2D(end.x, end.y, 0);
    }*/

    public boolean TrajectoryDone(double errorDistance){
        double in_front_still_distance = 0;
        for(int i=indLine+1;i<Path.size();i++)
            in_front_still_distance+=distance(Path.get(i).pstart,Path.get(i).pend);
        in_front_still_distance+=distance(Path.get(indLine).pend,new Point(Localizer.getCurrentPosition().x,Localizer.getCurrentPosition().y));
        return in_front_still_distance <= errorDistance;
    }

    public int TrajectoryIsAt(){
        return indLine;
    }

    public boolean AngleDone(double errorAngle){
        if(indLine+1 == Path.size())
            return Localizer.getAngleDifference(getTargetPosition().h,Localizer.getCurrentPosition().h) <= Math.toRadians(errorAngle);
        return false;
    }

    private double distance(Point a, Point b) {
        return Math.sqrt(Math.pow(a.x - b.x, 2) + Math.pow(a.y - b.y, 2));
    }

    private boolean is_on(Point a, Point b, Point c) {
        return Math.abs(distance(a, c) + distance(c, b) - distance(a, b)) < 1;
    }

    private List<Point> CalculateIntersectionOfCircleAndLine(Circle cerc, Line line) {
        List<Point> result = new ArrayList<>();

        Point a = line.pstart;
        Point b = line.pend;

        double eDistAtoB = Math.sqrt(Math.pow(b.x - a.x, 2) + Math.pow(b.y - a.y, 2));

        // compute the direction vector d from a to b
        Point d = new Point(0,0);
        d.x = (b.x - a.x) / eDistAtoB;
        d.y = (b.y - a.y) / eDistAtoB;

        // Now the line equation is x = dx*t + ax, y = dy*t + ay with 0 <= t <= 1.

        // compute the value t of the closest point to the circle center (cx, cy)
        double t = (d.x * (cerc.x - a.x)) + (d.y * (cerc.y - a.y));

        // compute the coordinates of the point e on line and closest to c
        Point e = new Point(0,0);
        e.x = (t * d.x) + a.x;
        e.y = (t * d.y) + a.y;

        // Calculate the euclidean distance between c & e
        double eDistCtoE = Math.sqrt(Math.pow(e.x - cerc.x, 2) + Math.pow(e.y - cerc.y, 2));

        // test if the line intersects the circle
        if (eDistCtoE <= cerc.r) {
            // compute distance from t to circle intersection point
            double dt = Math.sqrt(Math.pow(cerc.r, 2) - Math.pow(eDistCtoE, 2));

            // compute first intersection point
            Point f = new Point(0, 0);
            f.x = ((t - dt) * d.x) + a.x;
            f.y = ((t - dt) * d.y) + a.y;
            // check if f lies on the line
            if (is_on(a, b, f))
                result.add(f);

            // compute second intersection point
            Point g = new Point(0, 0);
            g.x = ((t + dt) * d.x) + a.x;
            g.y = ((t + dt) * d.y) + a.y;
            // check if g lies on the line
            if (is_on(a, b, g))
                result.add(g);
        }

        return result;
    }

    private void BuildPath(List<Point> pointsToFollow, double RobotRadius) {
        ChosenRadius = RobotRadius;
        for (int i = 0; i < pointsToFollow.size() - 1; i++) {
            Point p1 = pointsToFollow.get(i);
            Point p2 = pointsToFollow.get(i + 1);
            Path.add(new Line(p1, p2));
        }
    }

    public double GetHeadingRespectCoef(){
        Line CurrentLine = Path.get(indLine);
        return CurrentLine.pstart.HeadingRespectCoef;
    }

    private double GenerateBestHeading(Point robotPos, Line currentLine) {// to be remade
        Point normRobot = new Point(-robotPos.x, -robotPos.y);
        Point normendLine = new Point(-currentLine.pend.x, -currentLine.pend.y);
        double dxend = normendLine.x - normRobot.x;
        double dyend = normendLine.y - normRobot.y;
        double angleend = Math.atan2(dyend, dxend);// returns [-π, π]
        if (angleend < 0) {
            angleend += 2 * Math.PI;
        }
        double oppositeangle = Localizer.normalizeRadians(Localizer.getCurrentPosition().h + Math.PI);
        if(Localizer.getAngleDifference(Localizer.getCurrentPosition().h,angleend) > Localizer.getAngleDifference(oppositeangle,angleend))
            return Localizer.normalizeRadians(angleend + Math.PI);
        return angleend;
    }

    private Point GenerateBestPoint(List<Point> IntersectionsWithTheNextLine, Line line, SparkFunOTOS.Pose2D RobotPos) {
        Point BestPoint = null;
        double minDist = 1e9;
        for (Point point : IntersectionsWithTheNextLine) {
            double dist = Localizer.getDistanceFromTwoPoints(new SparkFunOTOS.Pose2D(point.x, point.y, 0), new SparkFunOTOS.Pose2D(line.pend.x, line.pend.y, 0));
            if (dist < minDist) {
                BestPoint = point;
                minDist = dist;
            }
        }
        if(Localizer.getDistanceFromTwoPoints(RobotPos, new SparkFunOTOS.Pose2D(line.pend.x, line.pend.y, 0)) < minDist)
            BestPoint = line.pend;
        return BestPoint;
    }

    public boolean HasPassedEndSegment(SparkFunOTOS.Pose2D pos,Line CurrentLine){
        return distance(
                new Point(pos.x, pos.y),
                CurrentLine.pend
        ) < ChosenRadius * 0.5;
    }

    public SparkFunOTOS.Pose2D FromLinesGeneratePointToFollow() {
        SparkFunOTOS.Pose2D pos = Localizer.getCurrentPosition();

        Line FutureLine = new Line(new Point(0, 0), new Point(0, 0));
        List<Point> IntersectionsWithTheNextLine = new ArrayList<>();

        if (indLine + 1 < Path.size()) {
            FutureLine = Path.get(indLine + 1);
            IntersectionsWithTheNextLine = CalculateIntersectionOfCircleAndLine(new Circle(pos.x, pos.y, ChosenRadius), FutureLine);
        }
        if (!IntersectionsWithTheNextLine.isEmpty()) {
            indLine++;
            Point BestPoint = GenerateBestPoint(IntersectionsWithTheNextLine, FutureLine,pos);
            //return new SparkFunOTOS.Pose2D(BestPoint.x, BestPoint.y, GenerateBestHeading(new Point(pos.x, pos.y), Path.get(indLine)));
            return new SparkFunOTOS.Pose2D(BestPoint.x, BestPoint.y, FutureLine.pend.h);
        }

        Line CurrentLine = Path.get(indLine);
        List<Point> IntersectionsWithTheCurrentLine = CalculateIntersectionOfCircleAndLine(new Circle(pos.x, pos.y, ChosenRadius), CurrentLine);
        if (!IntersectionsWithTheNextLine.isEmpty()) {
            Point BestPoint = GenerateBestPoint(IntersectionsWithTheCurrentLine, CurrentLine,pos);
            if (indLine + 1 < Path.size())
                //return new SparkFunOTOS.Pose2D(BestPoint.x, BestPoint.y, GenerateBestHeading(new Point(pos.x, pos.y), CurrentLine));
                return new SparkFunOTOS.Pose2D(BestPoint.x, BestPoint.y, CurrentLine.pend.h);
            return new SparkFunOTOS.Pose2D(BestPoint.x, BestPoint.y, CurrentLine.pend.h);
        }
        //am belit pula cineva s-a ciocnit in noi si suntem prea departe de path incearca sa merge la ultima pozitie
        Point BestPoint = new Point(CurrentLine.pend.x, CurrentLine.pend.y);
        if (indLine + 1 < Path.size())
            return new SparkFunOTOS.Pose2D(BestPoint.x, BestPoint.y, CurrentLine.pend.h);
        return new SparkFunOTOS.Pose2D(BestPoint.x, BestPoint.y, CurrentLine.pend.h);
    }
    public void ChangeRadius(double radius){
        this.ChosenRadius = radius;
    }

}
