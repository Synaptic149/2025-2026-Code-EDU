package config.core;

import com.acmerobotics.dashboard.config.Config;

@Config
public class robotConstants {

/// This class stores all values such as positions for bucket, transfer, and claws.
/// allowing the dashboard to config it allows us to modify it
/// All these variables need to be public and static

    /// Extend Constants
    public static String outmotorName = "out";
    public static double ep = 0.005, ei = 0, ed = 0.00000000000005, f = 0.00; // Extend pid constants

    public static int zero = 10;

    public static int max = 450;

    public static int half = max/2;

    public static int one_third  = max/3;

    public static int two_thirds = (max/3) * 2;





}
