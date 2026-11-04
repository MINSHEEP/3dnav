"""
SQLite数据库管理器，用于航点存储
"""

import sqlite3
import os
import threading
from typing import List, Dict, Optional

class WaypointDatabase:
    """航点数据库管理器"""
    
    def __init__(self, db_path: str = None):
        """
        初始化数据库管理器
        
        Args:
            db_path: 数据库文件路径，默认为 ~/.ros/waypoints.db
        """
        if db_path is None:
            self.db_path = "/home/a/3dnav/src/ros2_humble_main/src/nav_service/lzp_service/db/waypoints.db"
        else:
            self.db_path = db_path
            
        self.lock = threading.Lock()
        self.init_database()
    
    def init_database(self) -> None:
        """初始化数据库表"""
        # 确保目录存在
        os.makedirs(os.path.dirname(self.db_path), exist_ok=True)
        
        with self.lock:
            conn = sqlite3.connect(self.db_path)
            cursor = conn.cursor()
            
            # 创建航点表
            cursor.execute('''
                CREATE TABLE IF NOT EXISTS waypoints (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    name TEXT NOT NULL,
                    x REAL NOT NULL,
                    y REAL NOT NULL,
                    theta REAL NOT NULL,
                    map_name TEXT DEFAULT 'default_map',       
                    created TIMESTAMP DEFAULT CURRENT_TIMESTAMP
                )
            ''')
            
            # 创建索引提高查询性能
            cursor.execute('''
                CREATE INDEX IF NOT EXISTS idx_waypoint_name ON waypoints(name)
            ''')
            
            conn.commit()
            conn.close()
    
    def create_waypoint(self, name: str, x: float, y: float, theta: float, map_name: str) -> int:
        """
        创建新航点
        
        Args:
            name: 航点名称
            x: X坐标
            y: Y坐标
            theta: 角度
            map_name: 地图名称
            
        Returns:
            新航点的ID
        """
        with self.lock:
            conn = sqlite3.connect(self.db_path)
            cursor = conn.cursor()
            
            cursor.execute('''
                INSERT INTO waypoints (name, x, y, theta,map_name)
                VALUES (?, ?, ?, ?, ?)
            ''', (name, x, y, theta, map_name))
            
            waypoint_id = cursor.lastrowid
            conn.commit()
            conn.close()
            
            return waypoint_id
    
    def get_waypoint(self, waypoint_id: int) -> Optional[Dict]:
        """
        根据ID获取航点
        
        Args:
            waypoint_id: 航点ID
            
        Returns:
            航点字典或None（如果不存在）
        """
        with self.lock:
            conn = sqlite3.connect(self.db_path)
            cursor = conn.cursor()
            
            cursor.execute('''
                SELECT id, name, x, y, theta, map_name  FROM waypoints WHERE id = ?
            ''', (waypoint_id,))
            
            row = cursor.fetchone()
            conn.close()
            
            if row:
                return {
                    'id': row[0],
                    'name': row[1],
                    'x': row[2],
                    'y': row[3],
                    'theta': row[4],
                    'map_name': row[5]
                }
            return None
    
    def list_waypoints(self, map_name: str = None) -> List[Dict]:
        """
        列出所有航点或指定地图的航点
        
        Args:
            map_name: 地图名称，如果为None则列出所有航点
                
        Returns:
            航点列表
        """
        with self.lock:
            conn = sqlite3.connect(self.db_path)
            cursor = conn.cursor()
            
            if map_name is None:
                cursor.execute('''
                    SELECT id, name, x, y, theta, map_name FROM waypoints ORDER BY id 
                ''')
            else:
                cursor.execute('''
                    SELECT id, name, x, y, theta, map_name FROM waypoints WHERE map_name = ? ORDER BY id
                ''', (map_name,))
            
            rows = cursor.fetchall()
            conn.close()
            
            return [
                {
                    'id': row[0],
                    'name': row[1],
                    'x': row[2],
                    'y': row[3],
                    'theta': row[4],
                    'map_name': row[5]
                }
                for row in rows
            ]
    
    def update_waypoint(self, waypoint_id: int, name: str = None, 
                       x: float = None, y: float = None, theta: float = None) -> bool:
        """
        更新航点信息
        
        Args:
            waypoint_id: 航点ID
            name: 新名称（可选）
            x: 新X坐标（可选）
            y: 新Y坐标（可选）
            theta: 新角度（可选）
            
        Returns:
            更新是否成功
        """
        # 首先检查航点是否存在
        waypoint = self.get_waypoint(waypoint_id)
        if not waypoint:
            return False
        
        # 构建更新语句
        updates = []
        params = []
        
        if name is not None:
            updates.append("name = ?")
            params.append(name)
        if x is not None:
            updates.append("x = ?")
            params.append(x)
        if y is not None:
            updates.append("y = ?")
            params.append(y)
        if theta is not None:
            updates.append("theta = ?")
            params.append(theta)
        
        if not updates:
            return True  # 没有需要更新的字段
        
        params.append(waypoint_id)
        
        with self.lock:
            conn = sqlite3.connect(self.db_path)
            cursor = conn.cursor()
            
            query = f"UPDATE waypoints SET {', '.join(updates)} WHERE id = ?"
            cursor.execute(query, params)
            
            affected_rows = cursor.rowcount
            conn.commit()
            conn.close()
            
            return affected_rows > 0
    
    def delete_waypoint(self, waypoint_id: int) -> bool:
        """
        删除航点
        
        Args:
            waypoint_id: 航点ID
            
        Returns:
            删除是否成功
        """
        with self.lock:
            conn = sqlite3.connect(self.db_path)
            cursor = conn.cursor()
            
            cursor.execute("DELETE FROM waypoints WHERE id = ?", (waypoint_id,))
            
            affected_rows = cursor.rowcount
            conn.commit()
            conn.close()
            
            return affected_rows > 0
    
    def get_waypoint_by_name(self, name: str) -> Optional[Dict]:
        """
        根据名称获取航点
        
        Args:
            name: 航点名称
            
        Returns:
            航点字典或None（如果不存在）
        """
        with self.lock:
            conn = sqlite3.connect(self.db_path)
            cursor = conn.cursor()
            
            cursor.execute('''
                SELECT id, name, x, y, theta FROM waypoints WHERE name = ?
            ''', (name,))
            
            row = cursor.fetchone()
            conn.close()
            
            if row:
                return {
                    'id': row[0],
                    'name': row[1],
                    'x': row[2],
                    'y': row[3],
                    'theta': row[4]
                }
            return None
