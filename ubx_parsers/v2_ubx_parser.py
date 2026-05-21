#!/usr/bin/env python3
"""
UBX File Parser for RTK GPS Data
Parses UBX files and extracts position data with RTK accuracy

FIXED VERSION: pyubx2 already converts lat/lon to degrees, so we don't divide by 1e7
"""

from pyubx2 import UBXReader
import csv
from datetime import datetime

def parse_ubx_file(filename, output_csv=None):
    """
    Parse UBX file and extract position data
    
    Args:
        filename (str): Path to UBX file
        output_csv (str): Optional CSV output filename
    """
    
    positions = []
    
    with open(filename, 'rb') as stream:
        ubr = UBXReader(stream)
        
        print("Parsing UBX file...")
        print("-" * 50)
        
        for (raw_data, parsed_data) in ubr:
            # Focus on NAV-PVT messages (Position, Velocity, Time)
            if hasattr(parsed_data, 'identity') and parsed_data.identity == 'NAV-PVT':
                
                # Extract key position data
                # NOTE: pyubx2 already converts lat/lon to degrees and heights to meters!
                # Check the raw values to understand what pyubx2 gives us
                raw_lat = getattr(parsed_data, 'lat', 0)
                raw_lon = getattr(parsed_data, 'lon', 0)
                raw_hMSL = getattr(parsed_data, 'hMSL', 0)
                raw_height = getattr(parsed_data, 'height', 0)
                raw_hAcc = getattr(parsed_data, 'hAcc', 0)
                raw_vAcc = getattr(parsed_data, 'vAcc', 0)
                raw_gSpeed = getattr(parsed_data, 'gSpeed', 0)
                raw_headMot = getattr(parsed_data, 'headMot', 0)
                
                # pyubx2 v1.2.29+ automatically scales values:
                # - lat/lon are in degrees (already divided by 1e7)
                # - heights are in meters (already divided by 1000) 
                # - accuracies are in meters (already divided by 1000)
                # - gSpeed is in m/s (already divided by 1000)
                # - headMot is in degrees (already divided by 1e5)
                #
                # If your pyubx2 version does NOT auto-scale, uncomment the manual conversions
                
                position_data = {
                    'timestamp': getattr(parsed_data, 'iTOW', 0) / 1000.0,  # iTOW is always in ms
                    'year': getattr(parsed_data, 'year', 0),
                    'month': getattr(parsed_data, 'month', 0),
                    'day': getattr(parsed_data, 'day', 0),
                    'hour': getattr(parsed_data, 'hour', 0),
                    'minute': getattr(parsed_data, 'min', 0),
                    'second': getattr(parsed_data, 'sec', getattr(parsed_data, 'second', 0)),
                    
                    # pyubx2 gives lat/lon in degrees, but heights/accuracies in mm!
                    'latitude': raw_lat,                    # Already in degrees
                    'longitude': raw_lon,                   # Already in degrees
                    'altitude_msl': raw_hMSL / 1000.0,      # Convert mm to meters
                    'altitude_ellipsoid': raw_height / 1000.0,  # Convert mm to meters
                    'horizontal_accuracy': raw_hAcc / 1000.0,   # Convert mm to meters
                    'vertical_accuracy': raw_vAcc / 1000.0,     # Convert mm to meters
                    'fix_type': getattr(parsed_data, 'fixType', 0),
                    'num_satellites': getattr(parsed_data, 'numSV', 0),
                    'speed_2d': raw_gSpeed / 1000.0,      # Convert mm/s to m/s
                    'heading': raw_headMot / 100000.0,    # Convert 1e-5 degrees to degrees
                    
                    # Additional RTK-specific fields
                    'carrier_solution': getattr(parsed_data, 'carrSoln', 0),  # 0=none, 1=float, 2=fix
                    'flags': getattr(parsed_data, 'flags', 0),
                    'pDOP': getattr(parsed_data, 'pDOP', 0),  # Position DOP (already scaled by pyubx2)
                }
                
                positions.append(position_data)
                
                # Print first position to verify format
                if len(positions) == 1:
                    print(f"FIRST POSITION (verify these look correct):")
                    print(f"  Raw lat from pyubx2: {raw_lat}")
                    print(f"  Raw lon from pyubx2: {raw_lon}")
                    print(f"  Raw hMSL from pyubx2: {raw_hMSL}")
                    print(f"  Raw height from pyubx2: {raw_height}")
                    print()
                    
                    # Sanity check
                    if abs(raw_lat) < 0.001:
                        print("  WARNING: Latitude seems too small!")
                        print("  pyubx2 might not be auto-scaling. Check your pyubx2 version.")
                        print("  You may need to multiply lat/lon by 1e7")
                    elif abs(raw_lat) > 90:
                        print("  WARNING: Latitude > 90 degrees!")
                        print("  You may need to divide lat/lon by 1e7")
                    else:
                        print(f"  Latitude {raw_lat:.6f}° looks correct!")
                    print()
                
                # Print progress every 500 positions
                if len(positions) % 500 == 0:
                    print(f"Processed {len(positions)} positions...")
    
    print(f"\nParsed {len(positions)} position fixes")
    
    # Save to CSV if requested
    if output_csv and positions:
        save_to_csv(positions, output_csv)
        print(f"Data saved to {output_csv}")
    
    return positions

def get_fix_type_description(fix_type):
    """Convert fix type number to description"""
    fix_types = {
        0: "No Fix",
        1: "Dead Reckoning",
        2: "2D Fix",
        3: "3D Fix", 
        4: "GNSS + Dead Reckoning",
        5: "Time Only"
    }
    return fix_types.get(fix_type, f"Unknown ({fix_type})")

def get_carrier_solution_description(carr_soln):
    """Convert carrier solution number to description"""
    solutions = {
        0: "No RTK",
        1: "RTK Float",
        2: "RTK Fix"
    }
    return solutions.get(carr_soln, f"Unknown ({carr_soln})")

def save_to_csv(positions, filename):
    """Save position data to CSV file"""
    
    fieldnames = [
        'timestamp', 'year', 'month', 'day', 'hour', 'minute', 'second',
        'latitude', 'longitude', 'altitude_msl', 'altitude_ellipsoid',
        'horizontal_accuracy', 'vertical_accuracy', 'fix_type', 
        'carrier_solution', 'num_satellites', 'pDOP',
        'speed_2d', 'heading', 'flags'
    ]
    
    with open(filename, 'w', newline='') as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(positions)

def analyze_accuracy(positions):
    """Analyze the accuracy statistics of the position data"""
    
    if not positions:
        print("No position data to analyze")
        return
    
    h_accuracies = [p['horizontal_accuracy'] for p in positions if p['horizontal_accuracy'] > 0]
    v_accuracies = [p['vertical_accuracy'] for p in positions if p['vertical_accuracy'] > 0]
    
    print("\n" + "=" * 50)
    print("ACCURACY ANALYSIS")
    print("=" * 50)
    
    if h_accuracies:
        print(f"\nHorizontal Accuracy:")
        print(f"  Mean: {sum(h_accuracies)/len(h_accuracies):.4f} m")
        print(f"  Min:  {min(h_accuracies):.4f} m")
        print(f"  Max:  {max(h_accuracies):.4f} m")
    
    if v_accuracies:
        print(f"\nVertical Accuracy:")
        print(f"  Mean: {sum(v_accuracies)/len(v_accuracies):.4f} m")
        print(f"  Min:  {min(v_accuracies):.4f} m")
        print(f"  Max:  {max(v_accuracies):.4f} m")
    
    # Count fix types
    print(f"\nFix Type Distribution:")
    fix_counts = {}
    for pos in positions:
        fix_type = get_fix_type_description(pos['fix_type'])
        fix_counts[fix_type] = fix_counts.get(fix_type, 0) + 1
    
    for fix_type, count in sorted(fix_counts.items()):
        percentage = (count / len(positions)) * 100
        print(f"  {fix_type}: {count} ({percentage:.1f}%)")
    
    # Count carrier solutions (RTK status)
    print(f"\nRTK Carrier Solution Distribution:")
    carr_counts = {}
    for pos in positions:
        carr_soln = get_carrier_solution_description(pos.get('carrier_solution', 0))
        carr_counts[carr_soln] = carr_counts.get(carr_soln, 0) + 1
    
    for carr_soln, count in sorted(carr_counts.items()):
        percentage = (count / len(positions)) * 100
        print(f"  {carr_soln}: {count} ({percentage:.1f}%)")
    
    # Calculate position spread for RTK Fix points only
    rtk_fix_positions = [p for p in positions if p.get('carrier_solution', 0) == 2]
    if rtk_fix_positions:
        lats = [p['latitude'] for p in rtk_fix_positions]
        lons = [p['longitude'] for p in rtk_fix_positions]
        
        lat_spread = (max(lats) - min(lats)) * 111000  # Approx meters
        lon_spread = (max(lons) - min(lons)) * 111000 * 0.85  # Approx meters at ~32° lat
        
        print(f"\nRTK Fix Position Spread:")
        print(f"  North-South: {lat_spread:.3f} m")
        print(f"  East-West:   {lon_spread:.3f} m")

# Example usage
if __name__ == "__main__":
    # Replace with your UBX file path
    ubx_filename = "dataLog00012.ubx"
    csv_filename = "v2_dataLog00012_parsed_positions.csv"
    
    try:
        # Parse the UBX file
        positions = parse_ubx_file(ubx_filename, csv_filename)
        
        # Analyze accuracy
        analyze_accuracy(positions)
        
    except FileNotFoundError:
        print(f"Error: Could not find file '{ubx_filename}'")
        print("Please update the filename in the script with your actual UBX file path")
    except Exception as e:
        print(f"Error parsing file: {e}")
        import traceback
        traceback.print_exc()