#!/usr/bin/env python3
"""
UBX File Parser for RTK GPS Data
Parses UBX files and extracts position data with RTK accuracy
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
                
                # Extract key position data with error handling for different UBX versions
                position_data = {
                    'timestamp': getattr(parsed_data, 'iTOW', 0) / 1000.0,  # GPS time of week in seconds
                    'year': getattr(parsed_data, 'year', 0),
                    'month': getattr(parsed_data, 'month', 0),
                    'day': getattr(parsed_data, 'day', 0),
                    'hour': getattr(parsed_data, 'hour', 0),
                    'minute': getattr(parsed_data, 'min', 0),
                    'second': getattr(parsed_data, 'sec', getattr(parsed_data, 'second', 0)),  # Handle both 'sec' and 'second'
                    'longitude': getattr(parsed_data, 'lon', 0) / 1e7,  # Convert to degrees
                    'latitude': getattr(parsed_data, 'lat', 0) / 1e7,   # Convert to degrees
                    'altitude_msl': getattr(parsed_data, 'hMSL', 0) / 1000.0,  # Convert to meters
                    'altitude_ellipsoid': getattr(parsed_data, 'height', 0) / 1000.0,  # Convert to meters
                    'horizontal_accuracy': getattr(parsed_data, 'hAcc', 0) / 1000.0,  # Convert to meters
                    'vertical_accuracy': getattr(parsed_data, 'vAcc', 0) / 1000.0,    # Convert to meters
                    'fix_type': getattr(parsed_data, 'fixType', 0),
                    'num_satellites': getattr(parsed_data, 'numSV', 0),
                    'speed_2d': getattr(parsed_data, 'gSpeed', 0) / 1000.0,  # Convert to m/s
                    'heading': getattr(parsed_data, 'headMot', 0) / 1e5,     # Convert to degrees
                }
                
                positions.append(position_data)
                
                # Print some positions for verification
                if len(positions) % 100 == 1:  # Print every 100th position
                    print(f"Position {len(positions)}:")
                    print(f"  Time: {position_data['year']}-{position_data['month']:02d}-{position_data['day']:02d} "
                          f"{position_data['hour']:02d}:{position_data['minute']:02d}:{position_data['second']:02d}")
                    print(f"  Lat: {position_data['latitude']:.8f}°")
                    print(f"  Lon: {position_data['longitude']:.8f}°")
                    print(f"  Alt: {position_data['altitude_msl']:.3f} m")
                    print(f"  Accuracy: H={position_data['horizontal_accuracy']:.3f}m, V={position_data['vertical_accuracy']:.3f}m")
                    print(f"  Fix Type: {get_fix_type_description(position_data['fix_type'])}")
                    print(f"  Satellites: {position_data['num_satellites']}")
                    print()
    
    print(f"Parsed {len(positions)} position fixes")
    
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

def save_to_csv(positions, filename):
    """Save position data to CSV file"""
    
    fieldnames = [
        'timestamp', 'year', 'month', 'day', 'hour', 'minute', 'second',
        'latitude', 'longitude', 'altitude_msl', 'altitude_ellipsoid',
        'horizontal_accuracy', 'vertical_accuracy', 'fix_type', 'num_satellites',
        'speed_2d', 'heading'
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
    
    print("Accuracy Analysis:")
    print("-" * 30)
    
    if h_accuracies:
        print(f"Horizontal Accuracy:")
        print(f"  Mean: {sum(h_accuracies)/len(h_accuracies):.4f} m")
        print(f"  Min:  {min(h_accuracies):.4f} m")
        print(f"  Max:  {max(h_accuracies):.4f} m")
    
    if v_accuracies:
        print(f"Vertical Accuracy:")
        print(f"  Mean: {sum(v_accuracies)/len(v_accuracies):.4f} m")
        print(f"  Min:  {min(v_accuracies):.4f} m")
        print(f"  Max:  {max(v_accuracies):.4f} m")
    
    # Count fix types
    fix_counts = {}
    for pos in positions:
        fix_type = get_fix_type_description(pos['fix_type'])
        fix_counts[fix_type] = fix_counts.get(fix_type, 0) + 1
    
    print(f"\nFix Type Distribution:")
    for fix_type, count in fix_counts.items():
        percentage = (count / len(positions)) * 100
        print(f"  {fix_type}: {count} ({percentage:.1f}%)")

# Example usage
if __name__ == "__main__":
    # Replace with your UBX file path
    ubx_filename = "dataLog00005.ubx"
    csv_filename = "dataLog00005_parsed_positions.csv"
    
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