package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.util.PIDF;
import org.firstinspires.ftc.teamcode.utility.CrServoCaching;
import org.firstinspires.ftc.teamcode.util.wrappers.Sensorange;

public class Spindexer {

    /* ================= ENUMS ================= */

    private enum Pixel {
        EMPTY, GREEN, PURPLE
    }

    /* ================= HARDWARE ================= */

    private final CrServoCaching leftServo;
    private final CrServoCaching rightServo;

    private final ColorSensor colorA;
    private final ColorSensor colorB;
    private final ColorSensor colorC;

    private final Sensorange encoder;

    /* ================= STATE ================= */

    private final Pixel[] stored = new Pixel[3];
    public int greenMotif = 0;

    private boolean sortEnabled = false;
    private boolean sorted = false;

    /* ================= PID ================= */

    public static double P = 0.0015;
    public static double D = 0.0005;

    private final PIDF pid = new PIDF(P, D);
    private double target = -44;
    private double targetTwo = -44;
    private final double maxServoSpeed = 0.3;

    /* ================= TIMING ================= */

    private long lastScanTime = 0;
    private static final long SCAN_INTERVAL_MS = 40;

    /* ================= CACHE ================= */

    private double lastServoPower = 0;
    private int emptyCount = 3;

    /* ================= CONSTRUCTOR ================= */

    public Spindexer(HardwareMap map) {

        leftServo  = CrServoCaching(map.get(CRServo.class, "IndexServoL"));
        rightServo = CrServoCaching(map.get(CRServo.class, "IndexServoR"));

        encoder = new Sensorange("encoder", map);

        colorA = map.get(ColorSensor.class, "color1");
        colorB = map.get(ColorSensor.class, "color2");
        colorC = map.get(ColorSensor.class, "color3");

        pid.setTolerance(5);

        for (int i = 0; i < 3; i++) {
            stored[i] = Pixel.EMPTY;
        }
    }

    /* ================= UPDATE ================= */

    public void update() {

        encoder.calculateValue();

        if (!runPID()) {
            if (sortEnabled && !sorted) {
                scanDexer();
                runSortLogic();
            } else {
                moveEmptySlot();
            }
        } else {target = targetTwo;}
    }

    /* ================= SORT LOGIC ================= */

    private void runSortLogic() {

        int greenCount = 0;
        int purpleCount = 0;

        int greenIndex = -1;
        int emptyIndex = -1;

        for (int i = 0; i < 3; i++) {
            if (stored[i] == Pixel.GREEN) {
                greenCount++;
                greenIndex = i;
            } else if (stored[i] == Pixel.PURPLE) {
                purpleCount++;
            } else {
                emptyIndex = i;
            }
        }

        int total = greenCount + purpleCount;
        sorted = (total == 3);

        if (sorted) return;

        if (total < 3 && emptyIndex != -1) {
            //moveEmptySlot();
            sorted = true;
            return;
        }

        // Example logic: rotate to align GREEN
        if (greenCount == 1 && greenIndex != 0) {
            double toSort = 120 * ((greenIndex - greenMotif + 3) % 3);
            target += 360 + toSort;
            targetTwo += toSort;
        }

        // screw slopdexer it doesnt sort 2g1p ill do that later
    }

    /* ================= SENSOR SCAN ================= */

    private void scanDexer() {
        long now = System.currentTimeMillis();
        if (now - lastScanTime < SCAN_INTERVAL_MS) return;
        lastScanTime = now;

        emptyCount = 0;

        stored[0] = detectColor(colorA);
        stored[1] = detectColor(colorB);
        stored[2] = detectColor(colorC);

        for (Pixel p : stored) {
            if (p == Pixel.EMPTY) emptyCount++;
        }
    }

    private Pixel detectColor(ColorSensor sensor) {

        int r = sensor.red();
        int g = sensor.green();
        int b = sensor.blue();

        if (g > r && g > b && g - b > 100) {
            return Pixel.GREEN;
        }

        if (b > g && b - g > 200 && r > 500) {
            return Pixel.PURPLE;
        }

        return Pixel.EMPTY;
    }

    /* ================= PID ================= */

    private boolean runPID() {

        if (pid.atSetPoint()) {
            setPower(0);
            return true;
        }

        setPower(pid.calculate(encoder.getPosition(), target));
        return false;
    }

    /* ================= ACTUATION ================= */

    private void setPower(double power) {

        double clipped = Math.max(-maxServoSpeed, Math.min(power, maxServoSpeed));

        if (clipped != lastServoPower) {
            leftServo.setPower(clipped);
            rightServo.setPower(clipped);
            lastServoPower = clipped;
        }
    }

    private void moveEmptySlot() {
        // Implement if needed — left intentionally lightweight
    }

    /* ================= COMMANDS ================= */

    public void shoot() {
        sorted = false;
        target -= 360;
        targetTwo -= 360;
    }

    public void enableSort() {
        sortEnabled = true;
    }

    public void disableSort() {
        sortEnabled = false;
    }

    public boolean isIdle() {
        return pid.atSetPoint();
    }

    public boolean isEmpty() {
        return emptyCount == 3;
    }

    public boolean isFull() {
        return emptyCount == 0;
    }

    public void pleasekillmeiwannadie(){setPower(0.5);} 
    public void youbetterflymeouttoworlds(){setPower(-0.5);} 
    public void iamsacrificingmyfutureforthis(){setPower(0);} 
    
    @Override 
    public String toString(){ 
        return "Spindexer {" + 
            // "sensorA =" + colora.red() + " " + colora.green() + " " + colora.blue() + stored.get(0) + 
            // "sensorB =" + colorb.red() + " " + colorb.green() + " " + colorb.blue() + stored.get(1) + 
            // "sensorC =" + colorc.red() + " " + colorc.green() + " " + colorc.blue() + stored.get(2) + 
            "ENCODER!! ENCODER!! " + encoder.getPosition() + 
            "TARGET " + target 
            + "}";
    }
}
