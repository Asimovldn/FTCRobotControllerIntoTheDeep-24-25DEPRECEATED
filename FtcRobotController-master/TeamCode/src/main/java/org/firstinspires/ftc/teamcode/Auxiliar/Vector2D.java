package org.firstinspires.ftc.teamcode.Auxiliar;

public class Vector2D
{
    public double x;
    public double y;

    double magnitude;

    public Vector2D(double x, double y)
    {
        this.x = x;
        this.y = y;

        magnitude = Math.hypot(x,y);
    }

    public static Vector2D rotateVector(Vector2D vector, double angle)
    {
        double rotX = Math.cos(angle) * vector.x - Math.sin(angle) * vector.y;
        double rotY = Math.sin(angle) * vector.x + Math.cos(angle) * vector.y;

        return new Vector2D(rotX,rotY);
    }

    public static Vector2D normalizeVector(Vector2D vector)
    {
        return new Vector2D(vector.x / vector.magnitude, vector.y / vector.magnitude);
    }

    public static Vector2D scalarMultiplication(Vector2D vector, double scalar)
    {
        return new Vector2D(vector.x * scalar, vector.y * scalar);
    }

}

