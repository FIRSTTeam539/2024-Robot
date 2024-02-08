package frc.robot.utils;

import java.lang.Math;

import pabeles.concurrency.IntOperatorTask.Min;

public class utils {
    public static double getR(double x, double y){
        return Math.sqrt(Math.pow(x, 2)+Math.pow(y,2));
    }
    public static double rConversionFactor(double x, double y){
        return getR(x,y)/Math.min(Math.abs(x), Math.abs(y));
    }
}
