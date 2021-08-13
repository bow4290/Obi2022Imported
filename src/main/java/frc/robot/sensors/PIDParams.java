
package frc.robot.sensors;

public class PIDParams {
    private double m_kp;
    private double m_ki;
    private double m_kd;

    private double m_posTolerance;
    private double m_velTolerance;

    private double m_setpoint;

    public PIDParams(double kp, double ki, double kd, double posTolerance, double velTolerance, double setpoint){
        m_kp = kp;
        m_ki = ki;
        m_kd = kd;
        m_posTolerance = posTolerance;
        m_velTolerance = velTolerance;
        m_setpoint = setpoint;
    }

    public double getKp() {
        return m_kp;
    }

    public void setKp(double kp) {
        m_kp = kp;
    }

    public double getKi() {
        return m_ki;
    }

    public void setKi(double ki) {
        m_ki = ki;
    }

    public double getKd() {
        return m_kd;
    }

    public void setKd(double kd) {
        m_kd = kd;
    }

    public double getPosTolerance() {
        return m_posTolerance;
    }

    public void setPosTolerance(double posTolerance) {
        m_posTolerance = posTolerance;
    }

    public double getVelTolerance() {
        return m_velTolerance;
    }

    public void setVelTolerance(double velTolerance) {
        m_velTolerance = velTolerance;
    }

    public double getSetpoint() {
        return m_setpoint;
    }

    public void setSetpoint(double setpoint) {
        m_setpoint = setpoint;
    }
}
