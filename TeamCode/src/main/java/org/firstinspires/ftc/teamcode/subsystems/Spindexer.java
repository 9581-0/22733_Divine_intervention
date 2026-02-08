package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.teleop.SwerveTeleOpConfig;
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

    public static double P = SwerveTeleOpConfig.P;
    public static double D = 0.00007;

    public static double P2 = 0.0001;

    public static double D2 = 0.000006;

    private final PIDF pid = new PIDF(P, D);
    private final PIDF pid2 = new PIDF(P,D);
    private double target = 165;
    private double targetTwo = 165;
    private final double maxServoSpeed = 0.3;
    private String beufbrubf = "YOU KILLED A CAT";

    /* ================= TIMING ================= */

    private long lastScanTime = 0;
    private static final long SCAN_INTERVAL_MS = 10;

    /* ================= CACHE ================= */

    private double lastServoPower = 0;
    private int emptyCount = 3;

    /* ================= CONSTRUCTOR ================= */

    public Spindexer(HardwareMap map) {

        leftServo  = new CrServoCaching(map.get(CRServo.class, "IndexServoL"));
        rightServo = new CrServoCaching(map.get(CRServo.class, "IndexServoR"));

        encoder = new Sensorange("encoder", map);

        colorA = map.get(ColorSensor.class, "color1");
        colorB = map.get(ColorSensor.class, "color2");
        colorC = map.get(ColorSensor.class, "color3");

        pid.setTolerance(5);
        pid.setSetPoint(-44);

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
        if (sorted) return;
        beufbrubf = "SORTING AT LEAST TRIES";

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

        if (total < 3 && (greenCount > 0 || purpleCount > 0)) {
            //moveEmptySlot();
            //sorted = true;
        }
        beufbrubf = "SORTING GOT " + greenCount + "GREENS AND " + purpleCount + "PURPLES AND" + emptyCount + "EMPTIES WITH GREEN AT" + greenIndex;

        // Example logic: rotate to align GREEN
        if (sorted && greenCount == 1 && greenIndex == greenMotif) {target = targetTwo;}
        if (sorted && greenCount == 1 && greenIndex != greenMotif) {
            double toSort = 120 * ((greenIndex - greenMotif + 3) % 3);
            target += 70 + toSort;
            targetTwo += toSort;
            beufbrubf = "SORTING TRIED ITS BEST " + toSort;
        }

        // screw slopdexer it doesnt sort 2g1p ill do that later
    }

    /* ================= SENSOR SCAN ================= */

    private void scanDexer() {
        if (sorted) return;
        long now = System.currentTimeMillis();
        if (now - lastScanTime < SCAN_INTERVAL_MS) return;
        lastScanTime = now;

        emptyCount = 0;

        stored[0] = detectColor(colorC);
        stored[1] = detectColor(colorB);
        stored[2] = detectColor(colorA);

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
        pid.setSetPoint(target);
        if(Math.abs(encoder.getPosition()) > Math.abs(pid.getSetPoint()) + 4) {
            double potato = pid.calculate(encoder.getPosition());
            setPower(potato);
            return sorted && Math.abs(potato) < 0.05 || pid.atSetPoint();
        }
        else  {
            double potato2 = pid2.calculate(encoder.getPosition());
            setPower(potato2);
            return sorted && Math.abs(potato2) < 0.03 || pid.atSetPoint();
        }
        // if (potato < 0.05) {
        //     return true;
        // }

        // setPower(potato);
        // return false;
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
        target -= sortEnabled ? 70 : 720;
        targetTwo -= 720;
    }

    public void enableSort() {
        sortEnabled = true;
    }

    public void disableSort() {
        sortEnabled = false;
    }

    public boolean isIdle() {
        return true;
    }

    public boolean isEmpty() {
        return pid.atSetPoint();
    }

    public boolean isFull() {
        return emptyCount == 0;
    }

    public void pleasekillmeiwannadie(){setPower(0.5);} 
    public void youbetterflymeouttoworlds(){setPower(-0.5);} 
    public void iamsacrificingmyfutureforthis(){setPower(0);} 
    
    @Override 
    public String toString(){ 
        return "Spindexer {" + stored[0] + stored[1] + stored[2] +
            // "sensorA =" + colora.red() + " " + colora.green() + " " + colora.blue() + stored.get(0) + 
            // "sensorB =" + colorb.red() + " " + colorb.green() + " " + colorb.blue() + stored.get(1) + 
            // "sensorC =" + colorc.red() + " " + colorc.green() + " " + colorc.blue() + stored.get(2) + 
            "ENCODER!! ENCODER!! " + encoder.getPosition() + 
            "TARGET " + target +
            "PID says: " + pid.calculate() + pid.getSetPoint() + beufbrubf
            + "}";
    }
}
