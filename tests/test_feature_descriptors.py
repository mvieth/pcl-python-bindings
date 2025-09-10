#!/usr/bin/env python3
"""
Test script for PCL feature descriptor types.
"""

import pcl.common as pcl_common

def test_fpfh():
    """Test FPFHSignature33 functionality."""
    print("Testing FPFHSignature33...")
    
    # Create instance
    fpfh = pcl_common.FPFHSignature33()
    print(f"  Created: {repr(fpfh)}")
    
    # Test histogram access
    hist = fpfh.histogram
    print(f"  Histogram length: {len(hist)} (expected: 33)")
    assert len(hist) == 33, f"Expected 33 elements, got {len(hist)}"
    
    # Test setting values
    new_hist = [float(i) for i in range(33)]
    fpfh.histogram = new_hist
    retrieved = fpfh.histogram
    print(f"  Set values [0:5]: {new_hist[:5]}")
    print(f"  Got values [0:5]: {retrieved[:5]}")
    assert retrieved[:5] == new_hist[:5], "Values don't match"
    
    # Test point cloud
    cloud = pcl_common.PointcloudFPFHSignature33()
    cloud.append(fpfh)
    print(f"  Point cloud length after adding: {len(cloud)}")
    assert len(cloud) == 1, "Point cloud should have 1 element"
    
    print("  ✓ FPFHSignature33 tests passed!\n")

def test_shot352():
    """Test SHOT352 functionality."""
    print("Testing SHOT352...")
    
    # Create instance
    shot = pcl_common.SHOT352()
    print(f"  Created: {repr(shot)}")
    
    # Test descriptor access
    desc = shot.descriptor
    print(f"  Descriptor length: {len(desc)} (expected: 352)")
    assert len(desc) == 352, f"Expected 352 elements, got {len(desc)}"
    
    # Test reference frame access
    rf = shot.rf
    print(f"  Reference frame length: {len(rf)} (expected: 9)")
    assert len(rf) == 9, f"Expected 9 elements, got {len(rf)}"
    
    # Test setting values
    new_desc = [float(i % 100) for i in range(352)]
    new_rf = [float(i) for i in range(9)]
    shot.descriptor = new_desc
    shot.rf = new_rf
    
    retrieved_desc = shot.descriptor
    retrieved_rf = shot.rf
    
    print(f"  Set descriptor [0:5]: {new_desc[:5]}")
    print(f"  Got descriptor [0:5]: {retrieved_desc[:5]}")
    print(f"  Set RF: {new_rf}")
    print(f"  Got RF: {retrieved_rf}")
    
    assert retrieved_desc[:5] == new_desc[:5], "Descriptor values don't match"
    assert retrieved_rf == new_rf, "RF values don't match"
    
    # Test point cloud
    cloud = pcl_common.PointcloudSHOT352()
    cloud.append(shot)
    print(f"  Point cloud length after adding: {len(cloud)}")
    assert len(cloud) == 1, "Point cloud should have 1 element"
    
    print("  ✓ SHOT352 tests passed!\n")

def test_vfh():
    """Test VFHSignature308 functionality."""
    print("Testing VFHSignature308...")
    
    # Create instance
    vfh = pcl_common.VFHSignature308()
    print(f"  Created: {repr(vfh)}")
    
    # Test histogram access
    hist = vfh.histogram
    print(f"  Histogram length: {len(hist)} (expected: 308)")
    assert len(hist) == 308, f"Expected 308 elements, got {len(hist)}"
    
    # Test setting values
    new_hist = [float(i % 50) for i in range(308)]
    vfh.histogram = new_hist
    retrieved = vfh.histogram
    
    print(f"  Set values [0:5]: {new_hist[:5]}")
    print(f"  Got values [0:5]: {retrieved[:5]}")
    assert retrieved[:5] == new_hist[:5], "Values don't match"
    
    # Test point cloud
    cloud = pcl_common.PointcloudVFHSignature308()
    cloud.append(vfh)
    print(f"  Point cloud length after adding: {len(cloud)}")
    assert len(cloud) == 1, "Point cloud should have 1 element"
    
    print("  ✓ VFHSignature308 tests passed!\n")

def test_all_feature_types():
    """Test all feature descriptor types are available."""
    print("Testing all feature descriptor types availability...")
    
    feature_types = [
        'FPFHSignature33',
        'PFHSignature125', 
        'SHOT352',
        'SHOT1344',
        'VFHSignature308',
        'ESFSignature640'
    ]
    
    cloud_types = [
        'PointcloudFPFHSignature33',
        'PointcloudPFHSignature125',
        'PointcloudSHOT352', 
        'PointcloudSHOT1344',
        'PointcloudVFHSignature308',
        'PointcloudESFSignature640'
    ]
    
    # Test feature types exist
    for typename in feature_types:
        assert hasattr(pcl_common, typename), f"Missing type: {typename}"
        cls = getattr(pcl_common, typename)
        instance = cls()
        print(f"  ✓ {typename} available and instantiable")
    
    # Test cloud types exist  
    for typename in cloud_types:
        assert hasattr(pcl_common, typename), f"Missing cloud type: {typename}"
        cls = getattr(pcl_common, typename)
        instance = cls()
        print(f"  ✓ {typename} available and instantiable")
    
    print("  ✓ All feature types availability tests passed!\n")

def main():
    """Run all tests."""
    print("PCL Feature Descriptor Tests")
    print("============================\n")
    
    test_all_feature_types()
    test_fpfh()
    test_shot352()
    test_vfh()
    
    print("🎉 All feature descriptor tests passed!")

if __name__ == "__main__":
    main()
