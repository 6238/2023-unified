package frc.robot;

import java.util.function.Function;
import java.util.function.Supplier;

public final class MathUtil {
    public static Function<Double, Double> scaleMagnitude(double minIn, double maxIn, double minOut, double maxOut, double power) {
        return (input) ->
            input < minIn ? 0 :
            Math.signum(input) * edu.wpi.first.math.MathUtil.clamp(
                minOut + Math.pow((Math.abs(input) - minIn) / (maxIn - minIn) , power) * (maxOut - minOut),
                minOut, maxOut);
    }

    public static class Timer {
        long setPoint;
        public Timer(long timeMS) { this.setPoint = System.currentTimeMillis() + timeMS; }
        public boolean isFinished() { return System.currentTimeMillis() >= setPoint; }
    };

    public static class DecimalChange {
        Supplier<Double> valGetter;
        double value;
        public DecimalChange(Supplier<Double> valGetter) {
            this.valGetter = valGetter;
            value = valGetter.get();
        }

        public double get() {
            double newVal = valGetter.get();
            System.out.println("Old pitch: " + value + ", New Pitch: " + newVal);
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
            System.out.println("Old time: " + value + ", New time: " + newVal);
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
