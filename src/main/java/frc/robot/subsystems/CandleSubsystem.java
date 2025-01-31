package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CandleSubsystem extends SubsystemBase {
    private final CANdle candle;

    public CandleSubsystem(int deviceID) {
        candle = new CANdle(deviceID);

        CANdleConfiguration configAll = new CANdleConfiguration();

        configAll.statusLedOffWhenActive = false;
        configAll.disableWhenLOS = false;
        configAll.stripType = LEDStripType.RGB;
        configAll.brightnessScalar = 1;
        configAll.vBatOutputMode = VBatOutputMode.Modulated;

        System.out.println("Candle Configuration Complete");
    }

    public void setAllLEDToColor(int r, int g, int b) {
        candle.setLEDs(r, g, b);
        setModulatedOutput(0.9);
    }

    public void setOneLedToColor(int[] rgb, int ledNumber) {
        candle.setLEDs(rgb[0], rgb[1], rgb[2], 0, ledNumber, 1);
        setModulatedOutput(0.9);
    }

    public void doAnimate(Animation mouse) {
        candle.animate(mouse);
    }

    public void dontAnimate() {
        candle.clearAnimation(0);
    }

    public void setLEDOff() {
        candle.setLEDs(0, 0, 0);
        setModulatedOutput(0);
    }

    private void setModulatedOutput(double modulateVBatOutput) {
        candle.modulateVBatOutput(modulateVBatOutput);
    }
}
