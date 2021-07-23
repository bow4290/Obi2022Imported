
package frc.robot.sensors;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.LimelightConstants;

public class Limelight {
    private static NetworkTable table = null;

    public static enum LedMode {
        ledPipeline, ledOff, ledBlink, ledOn
    }
    
    public static enum CamMode {
        vision, driving
    }

    public Limelight(){
        table = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public void setLedMode(LedMode mode){                       // Set the state of the LED
        table.getEntry("ledMode").setNumber(mode.ordinal());
    }

    public LedMode getLedMode(){
        int mode_num = table.getEntry("ledMode").getNumber(0).intValue();

        if (mode_num == 0) {
            return LedMode.ledPipeline;
        }
        else if (mode_num == 1) {
            return LedMode.ledOff;
        }
        else if (mode_num == 1) {
            return LedMode.ledBlink;
        }
        else if (mode_num == 2) {
            return LedMode.ledOn;
        }

        return LedMode.ledOff;
    }

    public void setCamMode(CamMode mode){                       // Set the camera mode (vision processing or driving camera)
        table.getEntry("camMode").setNumber(mode.ordinal());
    }

    public void setPipeline(int pipeline){                      // Set the pipeline number (0-9)
        table.getEntry("pipeline").setNumber(pipeline);
    }

    public double getTarget(){                          // Whether the limelight has any valid targets (0 or 1)
        return table.getEntry("tv").getDouble(0);
    }

    public boolean isTarget(){                          // Whether the limelight has any valid targets (0 or 1)
        return table.getEntry("tv").getDouble(0) == 1;
    }
    
    public double getXError(){                          // Horizontal offset from limelight crosshair to target crosshair
        return table.getEntry("tx").getDouble(0.00);
    }

    public double getYError(){                          // Vertical offset from limelight crosshair to target crosshair
        return table.getEntry("ty").getDouble(0.00);
    }

    public double getArea(){                            // Target area (0% to 100% of image)
        return table.getEntry("ta").getDouble(0.00);
    }

    public double getSkew(){                            // Skew or rotation (-90 deg to 0 deg)
        return table.getEntry("ts").getDouble(0.00);
    }

    public double getLatency(){                         // Pipeline latency contribution (ms)
        return table.getEntry("tl").getDouble(0.00);
    }

    public double getShortLength(){                     // Sidelength of shortest side of fitted box (pixels)
        return table.getEntry("tshort").getDouble(0);
    }

    public double getLongLength(){                      // Sidelength of longest side of fitted box (pixels)
        return table.getEntry("tlong").getDouble(0);
    }

    public double getHorizontalLength(){                // Horizontal sidelength of rough bouding box (0-320 pixels)
        return table.getEntry("thor").getDouble(0);
    }

    public double getVerticalLength(){                  // Vertical sidelength of rough bouding box (0-320 pixels)
        return table.getEntry("tvert").getDouble(0);
    }

    public double getPipeline(){                        // True active pipeline of the camera (0-9)
        return table.getEntry("getpipe").getDouble(0);
    }

    public double getDistance(){                        // Horizontal distance from limelight to target
        return (LimelightConstants.h2 - LimelightConstants.h1) / Math.tan(Math.toRadians(LimelightConstants.a1) + Math.toRadians(getYError()));
    }

}
