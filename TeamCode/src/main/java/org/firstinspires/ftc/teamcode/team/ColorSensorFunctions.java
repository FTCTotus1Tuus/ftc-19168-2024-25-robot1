package org.firstinspires.ftc.teamcode.team;

public final class ColorSensorFunctions {

    public String whatColor(int r, int g, int b) {
        // return a string with the name of the color as either "Y", "R" or "B".
        String color = "";
        if (isYellow(r, g, b)) {
            color = "Y";
        } else if (isRed(r, g, b)) {
            color = "R";
        } else if (isBlue(r, g, b)) {
            color = "B";
        }
        return color;
    }

    public int yellowColorIntensity(int r, int g, int b) {
        int colorIntensityValue = 0;

        if (r >= 620 && g >= 870 && b >= 180) {
            colorIntensityValue = 80;
        }

        return colorIntensityValue;
    }


    public boolean isYellow(int r, int g, int b) {
        // RATIOS
        float r2g = (float) r / g;
        float b2g = (float) b / g;
        float r2b = (float) r / b;

        return .55 <= r2g && r2g <= .75
                && .15 <= b2g && b2g <= .55
                && 1.00 <= r2b && r2b <= 4.0;

    }

    public boolean isRed(int r, int g, int b) {
        // RATIOS
        float r2g = (float) r / g;
        float b2g = (float) b / g;
        float r2b = (float) r / b;

        return .65 <= r2g && r2g <= 1.90
                && .35 <= b2g && b2g <= .70
                && 0.95 <= r2b && r2b <= 4.85;

    }

    public boolean isBlue(int r, int g, int b) {
        // RATIOS
        float r2g = (float) r / g;
        float b2g = (float) b / g;
        float r2b = (float) r / b;

        return .35 <= r2g && r2g <= .55
                && .80 <= b2g && b2g <= 2.30
                && 0.15 <= r2b && r2b <= .70;

    }
}

