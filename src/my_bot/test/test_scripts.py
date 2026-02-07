import os
import stat

def test_scripts_executable():
    pkg_path = os.path.join(os.getcwd(), 'src', 'my_bot')
    scripts_path = os.path.join(pkg_path, 'scripts')
    
    scripts = [
        'ball_chaser.py',
        'camera_test.py',
        'patrol.py',
        'security_guard.py'
    ]
    
    for script in scripts:
        script_full_path = os.path.join(scripts_path, script)
        assert os.path.exists(script_full_path), f"{script} not found"
        
        # Check if executable
        st = os.stat(script_full_path)
        assert bool(st.st_mode & stat.S_IXUSR), f"{script} is not executable"
        
        # Check shebang
        with open(script_full_path, 'r') as f:
            first_line = f.readline()
            assert first_line.startswith('#!'), f"{script} missing shebang"
            assert 'python3' in first_line, f"{script} shebang should use python3"

if __name__ == '__main__':
    test_scripts_executable()
