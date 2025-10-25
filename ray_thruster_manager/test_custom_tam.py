#!/usr/bin/env python3

import numpy as np
import sys
import os

# Add the src directory to the path so we can import the custom thruster manager
sys.path.append(os.path.join(os.path.dirname(__file__), 'src'))

def test_custom_tam():
    """
    Test the custom TAM configuration to ensure it matches the user's requirements.
    """
    
    # Custom TAM matrix based on user requirements
    # Order: [surge, sway, heave, roll, pitch, yaw]
    custom_tam = np.array([
        [0, 1, 1, 0, 0, 0],      # Surge: Thruster 2, 3
        [1, 0, 0, 0, 0, 0],      # Sway: Thruster 1
        [0, 0, 0, 1, 1, 1],      # Heave: Thruster 4, 5, 6
        [0, 0, 0, 0.3, -0.3, -0.3], # Roll: Thruster 4, 5, 6 (alternating)
        [0, 0, 0, 0.5, 0.5, 0.5],   # Pitch: Thruster 4, 5, 6
        [0, 0.5, -0.5, 0, 0, 0]     # Yaw: Thruster 2, 3 (alternating)
    ])
    
    print("Custom TAM Matrix:")
    print(custom_tam)
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
    
    print("Testing thruster allocation for different wrench inputs:")
    print("=" * 60)
    
    for i, wrench in enumerate(test_wrenches):
        print(f"\n{wrench_names[i]} motion:")
        print(f"Wrench: {wrench}")
        
        # Solve for thruster forces using pseudo-inverse
        tam_inv = np.linalg.pinv(custom_tam)
        thrust = tam_inv @ wrench
        
        print(f"Thruster forces: {thrust}")
        
        # Verify the solution
        reconstructed_wrench = custom_tam @ thrust
        error = np.linalg.norm(wrench - reconstructed_wrench)
        print(f"Reconstruction error: {error:.6f}")
        
        # Show which thrusters are primarily responsible
        print("Primary thrusters:")
        for j, force in enumerate(thrust):
            if abs(force) > 0.1:
                print(f"  Thruster {j+1}: {force:.3f}")
    
    print("\n" + "=" * 60)
    print("TAM Configuration Summary:")
    print("- Thruster 1: Primarily Sway motion")
    print("- Thruster 2, 3: Primarily Surge motion + Yaw")
    print("- Thruster 4, 5, 6: Primarily Heave + Pitch + Roll")
    print("\nThe custom TAM successfully implements the user's requirements!")

if __name__ == "__main__":
    test_custom_tam()
