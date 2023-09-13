package delauney.model;

import delauney.interf.*;
public class Point implements IPoint {
    private double X;
    private double Y;

    public Point(double x, double y) {
        X = x;
        Y = y;
    }

    @Override
    public double getX() {
        return X;
    }

    @Override
    public void setX(double x) {
        X = x;
    }

    @Override
    public double getY() {
        return Y;
    }

    @Override
    public void setY(double y) {
        Y = y;
    }

    @Override
    public String toString() {
        return X + "," + Y;
    }
}