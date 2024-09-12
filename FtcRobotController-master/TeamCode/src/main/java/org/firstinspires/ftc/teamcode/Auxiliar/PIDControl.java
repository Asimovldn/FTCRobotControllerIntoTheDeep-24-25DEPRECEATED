package org.firstinspires.ftc.teamcode.Auxiliar;

public class PIDControl
{
    double kp;
    public PIDControl(double kp)
    {
        this.kp = kp;
    }
    public double calculatePID (double setPoint, double reference)
    {
        double erro = setPoint - reference;

        double proporcional = erro * kp;

        return proporcional;
    }
}
