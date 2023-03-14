package frc.robot;

public final class MathUtil {
    public static double scale(double input, double minX, double maxX, double minY, double maxY, double power) {
        return minY + Math.pow((input - minX) / (maxX - minX) , power) * (maxY - minY);
    }

    public static class DecimalChange {
        double value;
        public DecimalChange(double init) {
            value = init;
        }

        public void reset(double init) {
            value = init;
        }

        public double get(double newValue) {
            double change = newValue - value;
            value = newValue;
            return change;
        }
    }

    public static class IntegerChange {
        long value;
        public IntegerChange(long init) {
            value = init;
        }

        public void reset(long init) {
            value = init;
        }

        public double get(long newValue) {
            long change = newValue - value;
            value = newValue;
            return change;
        }
    }

    public static class SpeedGetter {
        DecimalChange positionChange;
        IntegerChange timeChange;
        public SpeedGetter(double initPosition) {
            positionChange = new DecimalChange(initPosition);
            timeChange = new IntegerChange(System.currentTimeMillis());
        }

        public void reset(double position) {
            positionChange.reset(position);
            timeChange.reset(System.currentTimeMillis());
        }

        public double get(double newPosition) {
            return positionChange.get(newPosition) /
                timeChange.get(System.currentTimeMillis());
        }
    };
}
