package frc.robot;

import java.util.function.Supplier;

public final class MathUtil {
    public static double scale(double input, double minX, double maxX, double minY, double maxY, double power) {
        return minY + Math.pow((input - minX) / (maxX - minX) , power) * (maxY - minY);
    }

    public static class DecimalChange {
        Supplier<Double> valGetter;
        double value;
        public DecimalChange(Supplier<Double> valGetter) {
            this.valGetter = valGetter;
            value = valGetter.get();
        }

        public double get() {
            double newVal = valGetter.get();
            double change = newVal - value;
            value = newVal;
            return change;
        }
    }

    public static class IntegerChange {
        Supplier<Long> valGetter;
        long value;
        public IntegerChange(Supplier<Long> valGetter) {
            this.valGetter = valGetter;
            value = valGetter.get();
        }

        public double get() {
            long newVal = valGetter.get();
            long change = newVal - value;
            value = newVal;
            return change;
        }
    }

    public static class SpeedGetter {
        Supplier<Double> posGetter;
        DecimalChange positionChange;
        IntegerChange timeChange;
        public SpeedGetter(Supplier<Double> posGetter) {
            this.posGetter = posGetter;
            positionChange = new DecimalChange(posGetter);
            timeChange = new IntegerChange(() -> {return System.currentTimeMillis();});
        }

        public double get() {
            return positionChange.get() /
                timeChange.get();
        }
    };
}
