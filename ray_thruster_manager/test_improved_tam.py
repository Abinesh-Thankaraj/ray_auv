#!/usr/bin/env python3

import numpy as np
import sys
import os

def test_improved_tam():
    """
    Test the improved TAM configuration based on actual thruster geometry.
    """
    
    # Improved TAM matrix based on actual thruster positions from URDF
    # Order: [surge, sway, heave, roll, pitch, yaw]
    improved_tam = np.array([
        [0, 1, 1, 0, 0, 0],      # Surge: Thruster 2, 3
        [-1, 0, 0, 0, 0, 0],     # Sway: Thruster 1 (negative because pointing left)
        [0, 0, 0, -1, -1, -1],   # Heave: Thruster 4, 5, 6 (negative because pointing down)
        [0, 0, 0, 0, 0.0684, -0.0684], # Roll: Thruster 5, 6 (moment arms)
        [0, 0, 0, -0.1815, -0.0684, 0.0684], # Pitch: Thruster 4, 5, 6 (moment arms)
        [-0.0549, 0.0747, -0.0747, 0, 0, 0]  # Yaw: Thruster 1, 2, 3 (moment arms)
    ])
    
    print("Improved TAM Matrix (based on actual thruster geometry):")
    print(improved_tam)
    print()
    
    # Test different wrench inputs
    test_wrenches = [
        np.array([1, 0, 0, 0, 0, 0]),  # Pure surge
        np.array([0, 1, 0, 0, 0, 0]),  # Pure sway
        np.array([0, 0, 1, 0, 0, 0]),  # Pure heave
        np.array([0, 0, 0, 1, 0, 0]),  # Pure roll
        np.array([0, 0, 0, 0, 1, 0]),  # Pure pitch
        np.array([0, 0, 0, 0, 0, 1]),  # Pure yaw
    ]
    
    wrench_names = ["Surge", "Sway", "Heave", "Roll", "Pitch", "Yaw"]
    
    print("Testing improved thruster allocation for different wrench inputs:")
    print("=" * 70)
    
    for i, wrench in enumerate(test_wrenches):
        print(f"\n{wrench_names[i]} motion:")
        print(f"Wrench: {wrench}")
        
        # Solve for thruster forces using pseudo-inverse
        tam_inv = np.linalg.pinv(improved_tam)
        thrust = tam_inv @ wrench
        
        print(f"Thruster forces: {thrust}")
        
        # Verify the solution
        reconstructed_wrench = improved_tam @ thrust
        error = np.linalg.norm(wrench - reconstructed_wrench)
        print(f"Reconstruction error: {error:.6f}")
        
        # Show which thrusters are primarily responsible
        print("Primary thrusters:")
        for j, force in enumerate(thrust):
            if abs(force) > 0.1:
                print(f"  Thruster {j+1}: {force:.3f}")
    
    print("\n" + "=" * 70)
    print("Improved TAM Configuration Summary:")
    print("- Thruster 1: Side thruster (sway + yaw) - pointing left")
    print("- Thruster 2,3: Forward thrusters (surge + yaw) - pointing forward")
    print("- Thruster 4,5,6: Vertical thrusters (heave + pitch + roll) - pointing down")
    print("\nKey improvements:")
    print("- Uses actual thruster positions and orientations from URDF")
    print("- Correct moment arm calculations for roll, pitch, yaw")
    print("- Proper sign conventions based on thruster directions")
    print("- Reduced deadzone for smoother control")
    print("- Thrust limiting to prevent excessive forces")
    print("\nThis should provide much more stable control!")

if __name__ == "__main__":
    test_improved_tam()
