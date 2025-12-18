import sys, os
from pathlib import Path

# Add parent directory to path to allow importing 'backend' package
sys.path.append('..')
sys.path.append('.')

try:
    from backend.pairing import pairing_manager
except ImportError:
    # Fallback if running directly inside backend folder
    from pairing import pairing_manager

print(f"Service CWD: {os.getcwd()}")
print(f"Storage file: {pairing_manager.storage_file}")
print(f"State file: {pairing_manager.pairing_state_file}")
print(f"State file exists: {pairing_manager.pairing_state_file.exists()}")

if pairing_manager.pairing_state_file.exists():
    with open(pairing_manager.pairing_state_file, 'r') as f:
        print(f"State file content: {f.read()}")
