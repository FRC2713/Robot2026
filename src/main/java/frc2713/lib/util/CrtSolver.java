package frc2713.lib.util;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;

public class CrtSolver {
    
    /**
     * Calculates the absolute continuous turns of the turret motor.
     *
     * @param thetaMotor Normalized absolute position of the motor encoder [0.0, 1.0)
     * @param thetaEncoder Normalized absolute position of the 26T driven encoder [0.0, 1.0)
     * @return Continuous absolute turns of the motor [0.0, 26.0)
     */
    public static double calculateAbsoluteMotorTurns(Angle encoder1Rotations, Angle encoder2Rotations, int encoder1GearTeeth, int encoder2GearTeeth) {
        double normalizedEncoder1Rotations = encoder1Rotations.in(Rotations) % 1.0; // Normalize to [0.0, 1.0)
        double normalizedEncoder2Rotations = encoder2Rotations.in(Rotations) % 1.0; // Normalize to [0.0, 1.0)

        int modularInverse = modInverse(encoder1GearTeeth, encoder2GearTeeth);
        // Calculate the difference scaled by the gear ratios
        double diff = (encoder2GearTeeth * normalizedEncoder2Rotations) - (encoder1GearTeeth * normalizedEncoder1Rotations);
        
        // Round to nearest integer to eliminate sensor noise and mechanical backlash
        long D = Math.round(diff);
        
        // Apply the modular inverse to solve for the integer motor turns
        long kmRaw = modularInverse * D;
        
        // True Modulo 26 calculation (handles negative values safely in Java)
        long km = ((kmRaw % encoder2GearTeeth) + encoder2GearTeeth) % encoder2GearTeeth;
        
        // Combine the integer turns with the current motor phase for the exact absolute position
        return km + normalizedEncoder1Rotations;
    }    

    /**
     * Calculates the modular multiplicative inverse of 'a' modulo 'm'.
     * Uses the iterative Extended Euclidean Algorithm.
     *
     * @param a The number to find the inverse for (e.g., motor gear teeth)
     * @param m The modulo base (e.g., driven gear teeth)
     * @return The modular inverse
     * @throws ArithmeticException if 'a' and 'm' are not coprime (GCD != 1)
     */
    public static int modInverse(int a, int m) {
        // Save the original modulo for wrapping negative results at the end
        int m0 = m; 
        
        // Coefficients for the extended Euclidean algorithm
        int y = 0;
        int x = 1;

        // Base case: modulo 1 has an inverse of 0
        if (m == 1) {
            return 0;
        }

        int originalA = a;

        // Run the standard Euclidean algorithm while tracking coefficients
        while (a > 1) {
            // If m reaches 0 before a reaches 1, they share a common factor
            if (m == 0) {
                throw new ArithmeticException(
                    "Inverse does not exist: " + originalA + " and " + m0 + " are not coprime."
                );
            }

            // q is the quotient
            int q = a / m;
            
            // t is a temporary variable to hold the old 'm'
            int t = m;

            // m becomes the remainder (Euclid's step)
            m = a % m;
            a = t;
            
            // Update the coefficients y and x
            t = y;
            y = x - q * y;
            x = t;
        }

        // The algorithm can produce a negative inverse. 
        // If it does, we wrap it into the positive modulo space.
        if (x < 0) {
            x += m0;
        }

        return x;
    }
}
