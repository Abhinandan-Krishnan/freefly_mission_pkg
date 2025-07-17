#!/usr/bin/env python3

import requests
import json
import time

def test_web_app():
    """Test the web app API endpoints"""
    base_url = "http://localhost:5000"
    
    print("🧪 Testing Drone Web App API...")
    
    # Test 1: Check if main page loads
    try:
        response = requests.get(f"{base_url}/")
        if response.status_code == 200:
            print("✅ Main page loads successfully")
        else:
            print(f"❌ Main page failed: {response.status_code}")
    except Exception as e:
        print(f"❌ Main page error: {e}")
    
    # Test 2: Check current data endpoint
    try:
        response = requests.get(f"{base_url}/api/current_data")
        if response.status_code == 200:
            data = response.json()
            print("✅ Current data endpoint works")
            print(f"   Position: {data.get('position', 'N/A')}")
            print(f"   Status: {data.get('status', 'N/A')}")
        else:
            print(f"❌ Current data failed: {response.status_code}")
    except Exception as e:
        print(f"❌ Current data error: {e}")
    
    # Test 3: Test mission service (will fail without drone node)
    try:
        response = requests.post(f"{base_url}/api/start_mission")
        if response.status_code == 200:
            result = response.json()
            print(f"✅ Mission service endpoint works: {result.get('message', 'N/A')}")
        else:
            print(f"❌ Mission service failed: {response.status_code}")
    except Exception as e:
        print(f"❌ Mission service error: {e}")
    
    # Test 4: Test land service (will fail without drone node)
    try:
        response = requests.post(f"{base_url}/api/emergency_land")
        if response.status_code == 200:
            result = response.json()
            print(f"✅ Land service endpoint works: {result.get('message', 'N/A')}")
        else:
            print(f"❌ Land service failed: {response.status_code}")
    except Exception as e:
        print(f"❌ Land service error: {e}")
    
    print("\n🎉 Web app testing complete!")
    print("📝 To fully test drone control:")
    print("   1. Start drone nav node: ros2 run freefly_mission_pkg drone_nav_node")
    print("   2. Start web app: python3 web_app.py")
    print("   3. Open browser: http://localhost:5000")

if __name__ == "__main__":
    test_web_app() 