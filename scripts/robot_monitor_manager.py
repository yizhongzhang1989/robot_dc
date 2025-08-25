#!/usr/bin/env python3
"""
Robot Monitor Manager

This script provides all-in-one management for robot monitor data.
Combines configuration viewing, data management, and basic analysis.
"""

import os
import json
import argparse
import subprocess
import yaml
import sqlite3
from datetime import datetime


class RobotMonitorManager:
    def __init__(self, data_dir=None):
        """Initialize manager with data directory"""
        if data_dir is None:
            data_dir = os.environ.get('ROBOT_DATA_DIR', os.path.expanduser('~/robot_data'))
        self.data_dir = data_dir
    
    def format_timestamp(self, timestamp_ns):
        """Convert nanosecond timestamp to readable format"""
        timestamp_sec = timestamp_ns / 1_000_000_000
        dt = datetime.fromtimestamp(timestamp_sec)
        return dt.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]  # Remove last 3 digits for ms precision
    
    def show_config(self):
        """Show current configuration and usage"""
        print("🤖 Robot Monitor Configuration")
        print("=" * 50)
        
        if os.environ.get('ROBOT_DATA_DIR'):
            print(f"📂 Data directory: {self.data_dir} (from environment)")
        else:
            print(f"📂 Data directory: {self.data_dir} (default)")
        
        print(f"📊 Directory exists: {'✅ Yes' if os.path.exists(self.data_dir) else '❌ No'}")
        
        if os.path.exists(self.data_dir):
            try:
                size = subprocess.check_output(['du', '-sh', self.data_dir]).decode().split()[0]
                print(f"💾 Total size: {size}")
            except Exception:
                print("💾 Total size: Unknown")
        
        print("\n💡 Usage:")
        print("  Default:     ros2 launch monitor monitor.launch.py")
        print("  Custom path: ros2 launch monitor monitor.launch.py data_dir:=/path")
        print("  Set env:     export ROBOT_DATA_DIR=/path")
        print()
    
    def list_sessions(self, date=None, detailed=False):
        """List recording sessions"""
        print(f"📊 Robot Data Sessions in {self.data_dir}")
        print("=" * 60)
        
        if not os.path.exists(self.data_dir):
            print("❌ No data directory found. Start monitoring to create data.")
            return
        
        # Get date directories
        if date:
            date_dirs = [date] if os.path.exists(os.path.join(self.data_dir, date)) else []
        else:
            date_dirs = sorted([d for d in os.listdir(self.data_dir) 
                              if os.path.isdir(os.path.join(self.data_dir, d)) and d.count('-') == 2])
        
        if not date_dirs:
            print("❌ No sessions found.")
            return
        
        total_sessions = 0
        total_size = 0
        
        for date_dir in date_dirs:
            date_path = os.path.join(self.data_dir, date_dir)
            print(f"\n📅 {date_dir}")
            print("-" * 40)
            
            # Find sessions
            sessions = []
            for item in os.listdir(date_path):
                item_path = os.path.join(date_path, item)
                if os.path.isdir(item_path) and item.startswith('robot_monitor_'):
                    sessions.append(item)
            
            if not sessions:
                print("   No sessions")
                continue
            
            sessions.sort()
            for session in sessions:
                total_sessions += 1
                size_mb = self._show_session(date_path, session, detailed)
                total_size += size_mb
        
        print(f"\n📈 Summary: {total_sessions} sessions, {total_size:.1f} MB total")
    
    def _show_session(self, date_path, session_name, detailed=False):
        """Show single session info"""
        session_path = os.path.join(date_path, session_name)
        info_file = os.path.join(date_path, f"{session_name}_info.json")
        
        # Extract time from name
        session_time = session_name.replace('robot_monitor_', '')
        print(f"   🕐 {session_time}")
        
        # Get file size - check both compressed and uncompressed formats
        db_file = os.path.join(session_path, f"{session_name}_0.db3")
        compressed_db_file = os.path.join(session_path, f"{session_name}_0.db3.zstd")
        
        size_mb = 0
        actual_file = None
        
        if os.path.exists(db_file):
            actual_file = db_file
            size_mb = os.path.getsize(db_file) / (1024 * 1024)
        elif os.path.exists(compressed_db_file):
            actual_file = compressed_db_file
            size_mb = os.path.getsize(compressed_db_file) / (1024 * 1024)
        
        if actual_file:
            compression_info = " (compressed)" if actual_file.endswith('.zstd') else ""
            print(f"      Size: {size_mb:.2f} MB{compression_info}")
            
            # Show message count if we can get it from metadata
            metadata_file = os.path.join(session_path, "metadata.yaml")
            if os.path.exists(metadata_file):
                try:
                    with open(metadata_file, 'r') as f:
                        metadata = yaml.safe_load(f)
                    msg_count = metadata['rosbag2_bagfile_information']['message_count']
                    duration = metadata['rosbag2_bagfile_information']['duration']['nanoseconds'] / 1_000_000_000
                    print(f"      Messages: {msg_count:,} ({duration:.2f}s)")
                except Exception:
                    pass
        else:
            print("      Status: ❌ No data")
        
        # Show session info if detailed
        if detailed and os.path.exists(info_file):
            try:
                with open(info_file, 'r') as f:
                    info = json.load(f)
                    print(f"      Start: {info.get('session_start', 'Unknown')}")
                    print(f"      Topics: {', '.join(info.get('topics', []))}")
            except Exception:
                pass
        
        return size_mb
    
    def cleanup(self, days=30, dry_run=True):
        """Clean up old data"""
        from datetime import datetime, timedelta
        
        cutoff = datetime.now() - timedelta(days=days)
        print(f"🧹 {'[DRY RUN] ' if dry_run else ''}Cleaning data older than {days} days")
        print("=" * 50)
        
        if not os.path.exists(self.data_dir):
            print("❌ No data directory found.")
            return
        
        deleted_count = 0
        freed_size = 0
        
        for date_dir in os.listdir(self.data_dir):
            try:
                date_obj = datetime.strptime(date_dir, '%Y-%m-%d')
                if date_obj < cutoff:
                    date_path = os.path.join(self.data_dir, date_dir)
                    if os.path.isdir(date_path):
                        try:
                            size = subprocess.check_output(['du', '-s', date_path]).decode().split()[0]
                            size_mb = int(size) / 1024
                            freed_size += size_mb
                        except Exception:
                            size_mb = 0
                        
                        print(f"   📁 {date_dir} ({size_mb:.1f} MB)")
                        if not dry_run:
                            import shutil
                            shutil.rmtree(date_path)
                        deleted_count += 1
            except ValueError:
                continue
        
        if deleted_count == 0:
            print("✅ No old data found.")
        else:
            action = "Would delete" if dry_run else "Deleted"
            print(f"\n{action} {deleted_count} directories, {freed_size:.1f} MB")
            if dry_run:
                print("Use --cleanup-confirm to actually delete")
    
    def test_connection(self):
        """Test UDP connection"""
        print("🔌 Testing UDP connection...")
        
        # Run test script
        test_script = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'test_udp_send.py')
        if os.path.exists(test_script):
            print(f"Running: {test_script}")
            subprocess.run(['python3', test_script])
        else:
            print("❌ test_udp_send.py not found")
    
    def analyze_session(self, session_path):
        """Analyze a specific session (replaces analyze_bag.py functionality)"""
        if not os.path.exists(session_path):
            print(f"❌ Session path not found: {session_path}")
            return
        
        if not os.path.isdir(session_path):
            print(f"❌ Not a directory: {session_path}")
            return
        
        session_name = os.path.basename(session_path)
        print(f"🔍 Analyzing: {session_name}")
        print("=" * 50)
        
        # Look for session info file
        info_file = os.path.join(os.path.dirname(session_path), f"{session_name}_info.json")
        if os.path.exists(info_file):
            self._show_session_info(info_file)
            print()
        
        # Analyze bag file from metadata
        self._show_bag_stats(session_path)
    
    def _show_session_info(self, info_file):
        """Show session information from JSON file"""
        try:
            with open(info_file, 'r') as f:
                info = json.load(f)
            
            print("📋 Session Information:")
            print(f"   Start Time: {info.get('session_start', 'Unknown')}")
            print(f"   Bag Name: {info.get('bag_name', 'Unknown')}")
            print(f"   Data Sources: {info.get('data_sources', {})}")
            print(f"   Topics: {', '.join(info.get('topics', []))}")
        except Exception as e:
            print(f"❌ Error reading session info: {e}")
    
    def _show_bag_stats(self, bag_path):
        """Show bag statistics from metadata.yaml"""
        metadata_file = os.path.join(bag_path, "metadata.yaml")
        
        if not os.path.exists(metadata_file):
            print(f"❌ Metadata file not found: {metadata_file}")
            return
        
        try:
            with open(metadata_file, 'r') as f:
                metadata = yaml.safe_load(f)
            
            bag_info = metadata['rosbag2_bagfile_information']
            
            print("📊 Bag Statistics:")
            print(f"   Message Count: {bag_info['message_count']:,}")
            print(f"   Duration: {bag_info['duration']['nanoseconds'] / 1_000_000_000:.2f} seconds")
            print(f"   Compression: {bag_info.get('compression_format', 'none')}")
            
            # Show file info
            for file_info in bag_info.get('files', []):
                file_path = os.path.join(bag_path, file_info['path'])
                # Check for compressed version
                if not os.path.exists(file_path):
                    compressed_path = file_path + '.zstd'
                    if os.path.exists(compressed_path):
                        file_path = compressed_path
                
                if os.path.exists(file_path):
                    size_mb = os.path.getsize(file_path) / (1024 * 1024)
                    print(f"   File Size: {size_mb:.2f} MB")
                    print(f"   File Path: {file_path}")
            
            # Show topic info
            print("\n📝 Recorded Topics:")
            for topic in bag_info.get('topics_with_message_count', []):
                topic_meta = topic['topic_metadata']
                msg_count = topic['message_count']
                print(f"   {topic_meta['name']}: {msg_count:,} messages ({topic_meta['type']})")
            
        except Exception as e:
            print(f"❌ Error analyzing bag metadata: {e}")
    
    def view_data(self, session_path, topic="/robot_data", limit=10, show_full=False, show_raw=False):
        """View data from ROS2 bag file"""
        
        # Find the database file
        db_file = None
        compressed_file = None
        temp_dir = None
        
        if os.path.isdir(session_path):
            # Look for .db3 or .db3.zstd files
            for file in os.listdir(session_path):
                if file.endswith('.db3'):
                    db_file = os.path.join(session_path, file)
                    break
                elif file.endswith('.db3.zstd'):
                    compressed_file = os.path.join(session_path, file)
        else:
            print(f"❌ Path not found: {session_path}")
            return
        
        # If we have compressed file but no uncompressed, decompress it temporarily
        if not db_file and compressed_file:
            print(f"📦 Found compressed file, decompressing: {compressed_file}")
            import tempfile
            import subprocess
            
            try:
                # Create temporary file for decompressed data
                temp_dir = tempfile.mkdtemp()
                temp_db_file = os.path.join(temp_dir, "temp.db3")
                
                # Decompress using zstd
                result = subprocess.run(['zstd', '-d', compressed_file, '-o', temp_db_file], 
                                      capture_output=True, text=True)
                if result.returncode == 0:
                    db_file = temp_db_file
                    print(f"✅ Decompressed to: {temp_db_file}")
                else:
                    print(f"❌ Failed to decompress: {result.stderr}")
                    return
            except Exception as e:
                print(f"❌ Error decompressing file: {e}")
                return
        
        if not db_file:
            print(f"❌ No .db3 or .db3.zstd file found in: {session_path}")
            return
        
        print(f"📖 Reading data from: {db_file}")
        print(f"🎯 Topic: {topic}")
        print(f"📊 Limit: {limit if limit > 0 else 'All'} messages")
        print("=" * 80)
        
        try:
            # Connect to SQLite database
            conn = sqlite3.connect(db_file)
            cursor = conn.cursor()
            
            # Get topic ID
            cursor.execute("SELECT id FROM topics WHERE name = ?", (topic,))
            topic_result = cursor.fetchone()
            if not topic_result:
                print(f"❌ Topic '{topic}' not found in bag file")
                cursor.execute("SELECT name FROM topics")
                available_topics = cursor.fetchall()
                print("Available topics:")
                for t in available_topics:
                    print(f"  - {t[0]}")
                return
            
            topic_id = topic_result[0]
            
            # Get messages with additional metadata
            if limit > 0:
                cursor.execute("""
                    SELECT m.id, m.timestamp, m.data, LENGTH(m.data) as data_size
                    FROM messages m
                    WHERE m.topic_id = ? 
                    ORDER BY m.timestamp 
                    LIMIT ?
                """, (topic_id, limit))
            else:
                cursor.execute("""
                    SELECT m.id, m.timestamp, m.data, LENGTH(m.data) as data_size
                    FROM messages m
                    WHERE m.topic_id = ? 
                    ORDER BY m.timestamp
                """, (topic_id,))
            
            messages = cursor.fetchall()
            
            if not messages:
                print(f"❌ No messages found for topic '{topic}'")
                return
            
            print(f"📝 Found {len(messages)} messages:")
            print()
            
            for i, (msg_id, timestamp, data_bytes, data_size) in enumerate(messages, 1):
                # Convert timestamp to readable format
                readable_time = self.format_timestamp(timestamp)
                
                print(f"📨 Message {i:3d} (DB ID: {msg_id})")
                print(f"   🕐 ROS2接收时间: {readable_time}")
                print(f"   📡 原始时间戳: {timestamp} 纳秒")
                print(f"   💾 数据大小: {data_size} 字节")
                
                if show_raw:
                    # Show raw binary data format
                    print(f"   📄 原始二进制数据 ({data_size} 字节):")
                    if isinstance(data_bytes, bytes):
                        # Show hex dump
                        hex_data = data_bytes.hex()
                        for j in range(0, len(hex_data), 32):
                            print(f"      {hex_data[j:j+32]}")
                        
                        # Show ASCII representation
                        ascii_repr = ""
                        for byte in data_bytes:
                            if 32 <= byte <= 126:  # Printable ASCII
                                ascii_repr += chr(byte)
                            else:
                                ascii_repr += "."
                        print("   📄 ASCII表示:")
                        for j in range(0, len(ascii_repr), 64):
                            print(f"      {ascii_repr[j:j+64]}")
                    else:
                        print(f"      {data_bytes}")
                    
                    print("   📝 存储说明:")
                    print("      - 时间戳存储在messages表的timestamp字段（INTEGER类型，纳秒精度）")
                    print("      - 数据存储在messages表的data字段（BLOB类型，CDR序列化格式）")
                    print("      - 时间戳 = ROS2节点接收UDP数据包的时间")
                    print("-" * 60)
                    continue
                
                # Parse the message data
                try:
                    # ROS2 messages are stored in CDR (Common Data Representation) format
                    # We need to skip the CDR header and extract the JSON string
                    
                    if isinstance(data_bytes, bytes) and len(data_bytes) > 8:
                        # Try to find JSON data in the binary message
                        # Look for the start of JSON data after the CDR header
                        json_start = -1
                        json_end = -1
                        
                        # Search for JSON pattern in bytes
                        for i in range(len(data_bytes) - 1):
                            if data_bytes[i:i+2] == b'{"':
                                json_start = i
                                break
                        
                        if json_start >= 0:
                            # Find the end of JSON - look for closing brace followed by null bytes or end
                            brace_count = 0
                            in_string = False
                            escape_next = False
                            
                            for i in range(json_start, len(data_bytes)):
                                char = chr(data_bytes[i]) if data_bytes[i] < 128 else '?'
                                
                                if escape_next:
                                    escape_next = False
                                    continue
                                    
                                if char == '\\':
                                    escape_next = True
                                    continue
                                    
                                if char == '"':
                                    in_string = not in_string
                                elif not in_string:
                                    if char == '{':
                                        brace_count += 1
                                    elif char == '}':
                                        brace_count -= 1
                                        if brace_count == 0:
                                            json_end = i + 1
                                            break
                            
                            if json_end > json_start:
                                # Extract and decode the JSON portion
                                json_bytes = data_bytes[json_start:json_end]
                                data_str = json_bytes.decode('utf-8', errors='replace')
                                
                                # Parse as JSON
                                if data_str.startswith('{"') and data_str.endswith('"}'):
                                    data_json = json.loads(data_str)
                                    print("   📊 Data (JSON):")
                                    for key, value in data_json.items():
                                        if key == "raw_message" and isinstance(value, str):
                                            if show_full:
                                                # Parse the raw_message if it's JSON
                                                try:
                                                    raw_json = json.loads(value)
                                                    print("      🤖 Robot Data:")
                                                    for rkey, rvalue in raw_json.items():
                                                        print(f"         {rkey}: {rvalue}")
                                                except Exception:
                                                    print(f"      {key}: {value}")
                                            else:
                                                preview = value[:100] + "..." if len(value) > 100 else value
                                                print(f"      {key}: {preview}")
                                        elif key == "message":
                                            # This is log data
                                            print(f"      📝 Log: {value}")
                                        else:
                                            print(f"      {key}: {value}")
                                else:
                                    print(f"   📄 Extracted text: {data_str}")
                            else:
                                # Fallback: show hex representation of data
                                hex_preview = data_bytes[:50].hex() + "..." if len(data_bytes) > 50 else data_bytes.hex()
                                print(f"   📄 Binary data (hex): {hex_preview}")
                                
                                # Try to find any readable text
                                readable_text = ""
                                for byte in data_bytes:
                                    if 32 <= byte <= 126:  # Printable ASCII
                                        readable_text += chr(byte)
                                    elif byte == 0:
                                        readable_text += "\\0"
                                    else:
                                        readable_text += f"\\x{byte:02x}"
                                
                                if len(readable_text) > 200 and not show_full:
                                    readable_text = readable_text[:200] + "..."
                                print(f"   📄 Readable text: {readable_text}")
                        else:
                            # No JSON found, show as hex
                            hex_preview = data_bytes[:50].hex() + "..." if len(data_bytes) > 50 else data_bytes.hex()
                            print(f"   📄 Binary data (hex): {hex_preview}")
                    else:
                        # Handle string data or short byte arrays
                        data_str = str(data_bytes)
                        print(f"   📄 Data: {data_str}")
                            
                except Exception as e:
                    print(f"   ❌ Error parsing data: {e}")
                    if show_full and isinstance(data_bytes, bytes):
                        hex_data = data_bytes.hex()
                        print(f"   📄 Raw data (hex): {hex_data}")
                    else:
                        print(f"   📄 Raw data: {data_bytes}")
                
                print("-" * 60)
            
            conn.close()
            
            # Clean up temporary file if we decompressed
            if compressed_file and temp_dir:
                import shutil
                try:
                    shutil.rmtree(temp_dir)
                    print("🧹 Cleaned up temporary files")
                except Exception:
                    pass
            
        except Exception as e:
            print(f"❌ Error reading bag file: {e}")
            # Clean up temporary file if we decompressed
            if compressed_file and temp_dir:
                import shutil
                try:
                    shutil.rmtree(temp_dir)
                except Exception:
                    pass
    
    def show_db_info(self, session_path):
        """Show database structure and detailed information"""
        
        # Find the database file
        db_file = None
        compressed_file = None
        temp_dir = None
        
        if os.path.isdir(session_path):
            # Look for .db3 or .db3.zstd files
            for file in os.listdir(session_path):
                if file.endswith('.db3'):
                    db_file = os.path.join(session_path, file)
                    break
                elif file.endswith('.db3.zstd'):
                    compressed_file = os.path.join(session_path, file)
        else:
            print(f"❌ Path not found: {session_path}")
            return
        
        # If we have compressed file but no uncompressed, decompress it temporarily
        if not db_file and compressed_file:
            print(f"📦 Found compressed file, decompressing: {compressed_file}")
            import tempfile
            import subprocess
            
            try:
                # Create temporary file for decompressed data
                temp_dir = tempfile.mkdtemp()
                temp_db_file = os.path.join(temp_dir, "temp.db3")
                
                # Decompress using zstd
                result = subprocess.run(['zstd', '-d', compressed_file, '-o', temp_db_file], 
                                      capture_output=True, text=True)
                if result.returncode == 0:
                    db_file = temp_db_file
                    print(f"✅ Decompressed to: {temp_db_file}")
                else:
                    print(f"❌ Failed to decompress: {result.stderr}")
                    return
            except Exception as e:
                print(f"❌ Error decompressing file: {e}")
                return
        
        if not db_file:
            print(f"❌ No .db3 or .db3.zstd file found in: {session_path}")
            return

        print("🗄️  数据库结构分析")
        print("=" * 80)
        print(f"📁 数据库文件: {db_file}")
        
        try:
            # Connect to SQLite database
            conn = sqlite3.connect(db_file)
            cursor = conn.cursor()
            
            # Show schema
            print("\n📋 数据库模式:")
            cursor.execute("SELECT sql FROM sqlite_master WHERE type='table'")
            schemas = cursor.fetchall()
            for schema in schemas:
                print(f"   {schema[0]}")
            
            # Show topics
            print("\n📡 话题信息:")
            cursor.execute("SELECT id, name, type, serialization_format FROM topics")
            topics = cursor.fetchall()
            for topic_id, name, msg_type, format_type in topics:
                print(f"   ID {topic_id}: {name} ({msg_type}, {format_type})")
                
                # Count messages for each topic
                cursor.execute("SELECT COUNT(*), MIN(timestamp), MAX(timestamp) FROM messages WHERE topic_id = ?", (topic_id,))
                count, min_ts, max_ts = cursor.fetchone()
                
                if count > 0:
                    min_time = self.format_timestamp(min_ts)
                    max_time = self.format_timestamp(max_ts)
                    duration = (max_ts - min_ts) / 1_000_000_000  # Convert to seconds
                    print(f"      消息数量: {count}")
                    print(f"      时间范围: {min_time} 至 {max_time}")
                    print(f"      录制时长: {duration:.2f} 秒")
                    if count > 1:
                        avg_rate = count / duration if duration > 0 else 0
                        print(f"      平均频率: {avg_rate:.2f} Hz")
                else:
                    print("      消息数量: 0")
            
            # Show message storage details
            print("\n💾 存储详情:")
            cursor.execute("SELECT COUNT(*) FROM messages")
            total_messages = cursor.fetchone()[0]
            print(f"   总消息数: {total_messages}")
            
            cursor.execute("SELECT AVG(LENGTH(data)), MIN(LENGTH(data)), MAX(LENGTH(data)) FROM messages")
            avg_size, min_size, max_size = cursor.fetchone()
            if avg_size:
                print(f"   消息大小: 平均 {avg_size:.1f} 字节, 范围 {min_size}-{max_size} 字节")
            
            # Show timestamp details
            print("\n⏰ 时间戳详情:")
            print("   存储位置: messages表的timestamp字段")
            print("   数据类型: INTEGER (64位)")
            print("   精度: 纳秒级")
            print("   来源: ROS2节点接收UDP数据包的系统时间")
            
            # Sample timestamps
            cursor.execute("SELECT timestamp FROM messages ORDER BY timestamp LIMIT 3")
            sample_timestamps = cursor.fetchall()
            print("   示例时间戳:")
            for ts_tuple in sample_timestamps:
                ts = ts_tuple[0]
                readable = self.format_timestamp(ts)
                print(f"     {ts} -> {readable}")
            
            conn.close()
            
            # Clean up temporary file if we decompressed
            if compressed_file and temp_dir:
                import shutil
                try:
                    shutil.rmtree(temp_dir)
                    print("\n🧹 已清理临时文件")
                except Exception:
                    pass
            
        except Exception as e:
            print(f"❌ Error reading database: {e}")
            # Clean up temporary file if we decompressed
            if compressed_file and temp_dir:
                import shutil
                try:
                    shutil.rmtree(temp_dir)
                except Exception:
                    pass

def main():
    parser = argparse.ArgumentParser(description='Robot Monitor Manager')
    parser.add_argument('--data-dir', help='Data directory path')
    parser.add_argument('--config', action='store_true', help='Show configuration')
    parser.add_argument('--list', action='store_true', help='List sessions')
    parser.add_argument('--date', help='Filter by date (YYYY-MM-DD)')
    parser.add_argument('--detailed', action='store_true', help='Show detailed session info')
    parser.add_argument('--analyze', help='Analyze specific session (path to session directory)')
    parser.add_argument('--view', help='View session data (path to session directory)')
    parser.add_argument('--topic', default='/robot_data', help='Topic to view (default: /robot_data)')
    parser.add_argument('--limit', type=int, default=10, help='Number of messages to show (0 for all)')
    parser.add_argument('--full', action='store_true', help='Show full message content without truncation')
    parser.add_argument('--raw', action='store_true', help='Show raw binary data format (hex dump)')
    parser.add_argument('--db-info', help='Show database structure and statistics (path to session directory)')
    parser.add_argument('--cleanup', type=int, help='Clean data older than N days (dry run)')
    parser.add_argument('--cleanup-confirm', type=int, help='Actually clean data older than N days')
    parser.add_argument('--test', action='store_true', help='Test UDP connection')
    
    args = parser.parse_args()
    
    manager = RobotMonitorManager(args.data_dir)
    
    if args.config:
        manager.show_config()
    elif args.list or args.date:
        manager.list_sessions(args.date, args.detailed)
    elif args.analyze:
        manager.analyze_session(args.analyze)
    elif args.view:
        manager.view_data(args.view, args.topic, args.limit, args.full, args.raw)
    elif args.db_info:
        manager.show_db_info(args.db_info)
    elif args.cleanup:
        manager.cleanup(args.cleanup, dry_run=True)
    elif args.cleanup_confirm:
        manager.cleanup(args.cleanup_confirm, dry_run=False)
    elif args.test:
        manager.test_connection()
    else:
        # Default: show config and recent sessions
        manager.show_config()
        print()
        manager.list_sessions()


if __name__ == '__main__':
    main()
