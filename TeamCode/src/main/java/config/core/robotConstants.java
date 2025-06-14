package config.core;

/// This class stores all values such as positions for bucket, transfer, and claws.
/// allowing the dashboard to config it allows us to modify it
/// All these variables need to be public and static
public class robotConstants {


    /// Extend Constants
    public static String outmotorName = "out";
    public static double ep = 0.01, ei = 0, ed = 0.00000000000005, f = 0.00; // Extend pid constants

    public static int zero = 0;

    public static int max = 600;

    public static int half = max/2;

    public static int one_third  = max/3;

    public static int two_thirds = (max/3) * 2;





}
