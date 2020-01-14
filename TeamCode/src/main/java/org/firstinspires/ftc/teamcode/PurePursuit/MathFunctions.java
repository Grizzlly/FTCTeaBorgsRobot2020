package org.firstinspires.ftc.teamcode.PurePursuit;

//import android.graphics.Point;

import org.opencv.core.Point;
import static java.lang.Math.*;
import java.util.ArrayList;


public class MathFunctions
{
    public static double AngleWrap(double angle)
    {
        while(angle < -Math.PI)
        {
            angle += 2*Math.PI;
        }
        while(angle > Math.PI)
        {
            angle -= 2*Math.PI;
        }
        return angle;
    }

    public static ArrayList<Point> lineCircleIntersection(Point circleCenter, double radius,
                                                          Point linePoin1, Point linePoint2)
    {
        if(Math.abs(linePoin1.y - linePoint2.y) < 0.003)
        {
            linePoin1.y = linePoint2.y + 0.003;
        }
        if(Math.abs(linePoin1.x - linePoint2.x) < 0.003)
        {
            linePoin1.x = linePoint2.x + 0.003;
        }

        double m1 = (linePoint2.y - linePoin1.y) / (linePoint2.x - linePoin1.x);

        double quadraticA = 1.0 + Math.pow(m1, 2);

        double x1 = linePoin1.x - circleCenter.x;
        double y1 = linePoin1.y - circleCenter.y;

        double quadraticB = (2.0 * m1 * y1) - (2.0 * Math.pow(m1, 2) * x1);

        double quadraticC = ((Math.pow(m1, 2) * Math.pow(x1, 2))) - (2.0 * y1 * m1 * x1) + Math.pow(y1, 2) - Math.pow(radius, 2);

        ArrayList<Point> allPoints = new ArrayList<>();

        try
        {
            double XRoot1 = (-quadraticB + sqrt(pow(quadraticB, 2) - 4.0 * quadraticA * quadraticC)) / (2.0 * quadraticA);

            double YRoot1 = m1 * (XRoot1 - x1) + y1;

            XRoot1 += circleCenter.x;
            YRoot1 += circleCenter.y;

            double minX = linePoin1.x < linePoint2.x ? linePoin1.x : linePoint2.x;
            double maxX = linePoin1.x > linePoint2.x ? linePoin1.x : linePoint2.x;
            //double minY = linePoin1.y < linePoint2.y ? linePoin1.y : linePoint2.y;


            if(XRoot1 > minX && XRoot1 < maxX)
            {
                allPoints.add(new Point(XRoot1, YRoot1));
            }

            double XRoot2 = (-quadraticB - sqrt(pow(quadraticB, 2) - 4.0 * quadraticA * quadraticC)) / (2.0 * quadraticA);

            double YRoot2 = m1 * (XRoot2 - x1) + y1;

            XRoot2 += circleCenter.x;
            YRoot2 += circleCenter.y;

            if(XRoot2 > minX && XRoot2 < maxX)
            {
                allPoints.add(new Point(XRoot2, YRoot2));
            }
        }catch(Exception e)
        {

        }

        return allPoints;
    }
}
