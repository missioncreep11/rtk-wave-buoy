#!/usr/bin/env python3
"""
UBX parser - uses HPPOSLLH for better precision
"""

from pyubx2 import UBXReader
import csv
import sys
import statistics

def parse_ubx_file(filename, output_csv=None):
    
    # store messages by timestamp so we can match them up later
    pvt_msgs = {}
    hppos_msgs = {}
    
    with open(filename, 'rb') as f:
        ubr = UBXReader(f)
        
        print("Parsing...")
        counts = {'NAV-PVT': 0, 'NAV-HPPOSLLH': 0}
        
        for (raw, parsed) in ubr:
            if not hasattr(parsed, 'identity'):
                continue
                
            ident = parsed.identity
            iTOW = getattr(parsed, 'iTOW', None)
            if iTOW is None:
                continue
            
            if ident == 'NAV-PVT':
                counts['NAV-PVT'] += 1
                pvt_msgs[iTOW] = {
                    'iTOW': iTOW,
                    'year': getattr(parsed, 'year', 0),
                    'month': getattr(parsed, 'month', 0),
                    'day': getattr(parsed, 'day', 0),
                    'hour': getattr(parsed, 'hour', 0),
                    'minute': getattr(parsed, 'min', 0),
                    'second': getattr(parsed, 'sec', 0),
                    'fix_type': getattr(parsed, 'fixType', 0),
                    'carrier_solution': getattr(parsed, 'carrSoln', 0),
                    'num_satellites': getattr(parsed, 'numSV', 0),
                    'pDOP': getattr(parsed, 'pDOP', 0) / 100.0,
                    'flags': getattr(parsed, 'flags', 0),
                    'gSpeed': getattr(parsed, 'gSpeed', 0) / 1000.0,
                    'headMot': getattr(parsed, 'headMot', 0) / 100000.0,
                    'velN': getattr(parsed, 'velN', 0) / 1000.0,
                    'velE': getattr(parsed, 'velE', 0) / 1000.0,
                    'velD': getattr(parsed, 'velD', 0) / 1000.0,
                    # fallback position if no HPPOSLLH
                    'lat': getattr(parsed, 'lat', 0),
                    'lon': getattr(parsed, 'lon', 0),
                    'hMSL': getattr(parsed, 'hMSL', 0) / 1000.0,
                    'height': getattr(parsed, 'height', 0) / 1000.0,
                    'hAcc': getattr(parsed, 'hAcc', 0) / 1000.0,
                    'vAcc': getattr(parsed, 'vAcc', 0) / 1000.0,
                }
            
            elif ident == 'NAV-HPPOSLLH':
                counts['NAV-HPPOSLLH'] += 1
                
                lat = getattr(parsed, 'lat', 0)
                lon = getattr(parsed, 'lon', 0)
                height = getattr(parsed, 'height', 0)
                hMSL = getattr(parsed, 'hMSL', 0)
                
                # hp components - try both naming conventions pyubx2 might use
                hpLat = getattr(parsed, 'latHp', getattr(parsed, 'hpLat', 0))
                hpLon = getattr(parsed, 'lonHp', getattr(parsed, 'hpLon', 0))
                hpHeight = getattr(parsed, 'heightHp', getattr(parsed, 'hpHeight', 0))
                hpHMSL = getattr(parsed, 'hMSLHp', getattr(parsed, 'hpHMSL', 0))
                
                # combine base + hp: base is deg, hp adds 1e-9 deg
                lat_combined = lat + (hpLat * 1e-9)
                lon_combined = lon + (hpLon * 1e-9)
                # heights: base in mm, hp in 0.1mm
                height_combined = (height + hpHeight * 0.1) / 1000.0
                hMSL_combined = (hMSL + hpHMSL * 0.1) / 1000.0
                
                hppos_msgs[iTOW] = {
                    'lat': lat_combined,
                    'lon': lon_combined,
                    'height': height_combined,
                    'hMSL': hMSL_combined,
                    'hAcc': getattr(parsed, 'hAcc', 0) / 10000.0,
                    'vAcc': getattr(parsed, 'vAcc', 0) / 10000.0,
                }
                
                if counts['NAV-HPPOSLLH'] == 1:
                    print(f"First HPPOSLLH: lat={lat_combined:.9f}, hAcc={hppos_msgs[iTOW]['hAcc']*100:.2f}cm")
            
            total = sum(counts.values())
            if total % 2000 == 0:
                print(f"  {total} messages...")
    
    print(f"\nFound: {counts}")
    
    # combine into position records
    positions = []
    use_hp = len(hppos_msgs) > 0
    timestamps = sorted(hppos_msgs.keys() if use_hp else pvt_msgs.keys())
    
    print(f"Using {'HPPOSLLH' if use_hp else 'PVT'} for positions")
    
    for iTOW in timestamps:
        pos = {'timestamp': iTOW / 1000.0}
        
        pvt = pvt_msgs.get(iTOW, {})
        hp = hppos_msgs.get(iTOW, {})
        
        # time and fix info from PVT
        pos['year'] = pvt.get('year', 0)
        pos['month'] = pvt.get('month', 0)
        pos['day'] = pvt.get('day', 0)
        pos['hour'] = pvt.get('hour', 0)
        pos['minute'] = pvt.get('minute', 0)
        pos['second'] = pvt.get('second', 0)
        pos['fix_type'] = pvt.get('fix_type', 0)
        pos['carrier_solution'] = pvt.get('carrier_solution', 0)
        pos['num_satellites'] = pvt.get('num_satellites', 0)
        pos['pDOP'] = pvt.get('pDOP', 0)
        pos['flags'] = pvt.get('flags', 0)
        pos['speed_2d'] = pvt.get('gSpeed', 0)
        pos['heading'] = pvt.get('headMot', 0)
        pos['vel_north'] = pvt.get('velN', 0)
        pos['vel_east'] = pvt.get('velE', 0)
        pos['vel_down'] = pvt.get('velD', 0)
        
        # position - prefer HP when available
        if hp:
            pos['latitude'] = hp['lat']
            pos['longitude'] = hp['lon']
            pos['altitude_msl'] = hp['hMSL']
            pos['altitude_ellipsoid'] = hp['height']
            pos['horizontal_accuracy'] = hp['hAcc']
            pos['vertical_accuracy'] = hp['vAcc']
            pos['source'] = 'HP'
        else:
            pos['latitude'] = pvt.get('lat', 0)
            pos['longitude'] = pvt.get('lon', 0)
            pos['altitude_msl'] = pvt.get('hMSL', 0)
            pos['altitude_ellipsoid'] = pvt.get('height', 0)
            pos['horizontal_accuracy'] = pvt.get('hAcc', 0)
            pos['vertical_accuracy'] = pvt.get('vAcc', 0)
            pos['source'] = 'PVT'
        
        positions.append(pos)
    
    print(f"Combined {len(positions)} positions")
    
    if output_csv and positions:
        save_csv(positions, output_csv)
        print(f"Saved to {output_csv}")
    
    return positions


def save_csv(positions, filename):
    fields = [
        'timestamp', 'year', 'month', 'day', 'hour', 'minute', 'second',
        'latitude', 'longitude', 'altitude_msl', 'altitude_ellipsoid',
        'horizontal_accuracy', 'vertical_accuracy', 'fix_type', 
        'carrier_solution', 'num_satellites', 'pDOP',
        'speed_2d', 'heading', 'vel_north', 'vel_east', 'vel_down',
        'flags', 'source'
    ]
    
    with open(filename, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(positions)


def analyze(positions):
    if not positions:
        print("No data")
        return
    
    print("\n" + "="*50)
    print("ANALYSIS")
    print("="*50)
    
    # fix type breakdown
    print("\nFix types:")
    fix_names = {0: "No Fix", 1: "DR", 2: "2D", 3: "3D", 4: "GNSS+DR", 5: "Time"}
    fix_counts = {}
    for p in positions:
        ft = fix_names.get(p['fix_type'], str(p['fix_type']))
        fix_counts[ft] = fix_counts.get(ft, 0) + 1
    for ft, cnt in sorted(fix_counts.items()):
        print(f"  {ft}: {cnt} ({cnt/len(positions)*100:.1f}%)")
    
    # RTK status
    print("\nRTK status:")
    rtk_names = {0: "None", 1: "Float", 2: "Fix"}
    rtk_counts = {}
    for p in positions:
        rs = rtk_names.get(p['carrier_solution'], str(p['carrier_solution']))
        rtk_counts[rs] = rtk_counts.get(rs, 0) + 1
    for rs, cnt in sorted(rtk_counts.items()):
        print(f"  {rs}: {cnt} ({cnt/len(positions)*100:.1f}%)")
    
    # position stats for RTK fix only
    rtk_fix = [p for p in positions if p['carrier_solution'] == 2]
    if len(rtk_fix) > 1:
        lats = [p['latitude'] for p in rtk_fix]
        lons = [p['longitude'] for p in rtk_fix]
        alts = [p['altitude_msl'] for p in rtk_fix]
        
        lat_std = statistics.stdev(lats) * 111000
        lon_std = statistics.stdev(lons) * 111000 * 0.85  # cos correction
        alt_std = statistics.stdev(alts)
        horiz_std = (lat_std**2 + lon_std**2)**0.5
        
        print(f"\nRTK Fix precision ({len(rtk_fix)} pts):")
        print(f"  Horizontal: {horiz_std*100:.2f} cm")
        print(f"  Vertical:   {alt_std*100:.2f} cm")
    
    hp_cnt = sum(1 for p in positions if p.get('source') == 'HP')
    print(f"\nSource: {hp_cnt} HPPOSLLH, {len(positions)-hp_cnt} PVT")


if __name__ == "__main__":
    if len(sys.argv) > 1:
        infile = sys.argv[1]
        outfile = infile.replace('.ubx', '_parsed.csv')
    else:
        infile = "dataLog00029.ubx"
        outfile = "dataLog00029_parsed.csv"
    
    print(f"Input:  {infile}")
    print(f"Output: {outfile}\n")
    
    try:
        positions = parse_ubx_file(infile, outfile)
        analyze(positions)
    except FileNotFoundError:
        print(f"File not found: {infile}")
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()