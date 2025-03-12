import subprocess
import signal
import os
import threading
import time
import atexit
from mvp_gui.models import db, RosLaunchList, RosThreadList, RosNodeList, RosNodeKeywords, RosTopicList, RosTopicKeywords

class ServerManager:
    """Manages the Flask server process lifecycle and environment"""
    
    _instance = None
    
    @classmethod
    def get_instance(cls):
        """Singleton pattern to ensure only one manager exists"""
        if cls._instance is None:
            cls._instance = cls()
        return cls._instance
    
    def __init__(self):
        """Initialize the server manager with process tracking and environment setup"""
        self.flask_process = None
        self.lock = threading.Lock()
        self.project_path = os.getcwd()
        # Create environment once at initialization
        self.env = os.environ.copy()
        self.env['PYTHONPATH'] = self.project_path
        # Register cleanup on exit
        atexit.register(self.stop_server)
    
    def start_server(self):
        """Start the Flask server process if not already running"""
        with self.lock:
            if self.flask_process is None:
                self.flask_process = subprocess.Popen(
                    ['python3', './mvp_gui/server.py'],
                    env=self.env, 
                    start_new_session=True
                )
                print(f"Flask server started with PID: {self.flask_process.pid}")
                return True
            else:
                print("Flask server is already running")
                return False
    
    def stop_server(self):
        """Safely terminate the Flask server"""
        with self.lock:
            if self.flask_process:
                try:
                    os.killpg(os.getpgid(self.flask_process.pid), signal.SIGTERM)
                    print("Flask server stopped gracefully")
                except (ProcessLookupError, OSError) as e:
                    print(f"Error stopping Flask server: {e}")
                finally:
                    self.flask_process = None
    
    def is_running(self):
        """Check if the server is currently running"""
        with self.lock:
            return self.flask_process is not None and self.flask_process.poll() is None


class DatabaseManager:
    """Manages database operations"""
    
    def __init__(self):
        """Initialize the database manager"""
        self.ros_tables = {
            'RosLaunchList': RosLaunchList,
            'RosThreadList': RosThreadList,
            'RosNodeList': RosNodeList,
            'RosNodeKeywords': RosNodeKeywords,
            'RosTopicList': RosTopicList,
            'RosTopicKeywords': RosTopicKeywords
        }
    
    def clear_ros_tables(self):
        """Clear all ROS-related tables in the database"""
        try:
            for table_name, table_class in self.ros_tables.items():
                db.session.query(table_class).delete()
            
            db.session.commit()
            print("Database cleared successfully")
            return True
        except Exception as e:
            db.session.rollback()
            print(f"Error clearing database: {e}")
            return False


def main():
    """Main entry point for the application"""
    # Get server manager instance
    server_manager = ServerManager.get_instance()
    
    # Create database manager
    db_manager = DatabaseManager()
    
    try:
        # Start the Flask server
        server_manager.start_server()
        
        # Clear database entries
        db_manager.clear_ros_tables()
        
        print("Server running. Press Ctrl+C to exit...")
        while True:
            # Check if server is still running
            if not server_manager.is_running():
                print("Server stopped unexpectedly, restarting...")
                server_manager.start_server()
            
            time.sleep(5)  # Reduced polling frequency to lower CPU usage
            
    except KeyboardInterrupt:
        print("\nShutting down gracefully...")
    finally:
        # Cleanup will be handled by the atexit handler
        pass


if __name__ == "__main__":
    main()