import os
import subprocess
from ament_index_python.packages import get_package_share_directory

def test_xacro_parsing():
    # Get the path to the xacro file
    pkg_path = os.path.join(os.getcwd(), 'src', 'my_bot')
    xacro_file = os.path.join(pkg_path, 'urdf', 'robot.urdf.xacro')
    
    assert os.path.exists(xacro_file), f"Xacro file not found at {xacro_file}"
    
    # Try to process the xacro file
    # Note: In a real test environment, we might need to mock dependencies or ensure they are installed
    # For now, we just check if the command runs without error
    try:
        result = subprocess.run(['xacro', xacro_file], capture_output=True, text=True, check=True)
        assert result.returncode == 0
        assert 'robot' in result.stdout
    except subprocess.CalledProcessError as e:
        # If it fails due to missing dependencies in the CI environment, we might want to handle it
        # but for a CI that installs everything, this should pass.
        assert False, f"Xacro parsing failed: {e.stderr}"
    except FileNotFoundError:
        # xacro command not found
        assert False, "xacro command not found"

if __name__ == '__main__':
    test_xacro_parsing()
