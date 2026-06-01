#!/usr/bin/env python3

import sqlite3
import sys
import os

def update_task_points_db(db_path, new_map_name):
    """
    更新任务点数据库中的map_name字段
    
    Args:
        db_path (str): 数据库文件路径
        new_map_name (str): 新的地图名称
    """
    try:
        # 检查数据库文件是否存在
        if not os.path.exists(db_path):
            print(f"❌ 数据库文件不存在: {db_path}")
            return False
        
        # 连接数据库
        conn = sqlite3.connect(db_path)
        cursor = conn.cursor()
        
        # 检查表是否存在
        cursor.execute("SELECT name FROM sqlite_master WHERE type='table' AND name='task_points'")
        if not cursor.fetchone():
            print(f"❌ 数据库中没有找到 task_points 表: {db_path}")
            conn.close()
            return False
        
        # 更新map_name字段
        cursor.execute("UPDATE task_points SET map_name = ? WHERE map_name = 'temp'", (new_map_name,))
        
        # 获取更新的行数
        updated_rows = cursor.rowcount
        
        # 提交更改
        conn.commit()
        conn.close()
        
        if updated_rows > 0:
            print(f"✅ 已更新 {updated_rows} 条记录的map_name为: {new_map_name}")
        else:
            print(f"ℹ️ 没有找到需要更新的记录 (map_name = 'temp')")
        
        return True
        
    except Exception as e:
        print(f"❌ 更新数据库失败: {str(e)}")
        return False

def main():
    """主函数"""
    if len(sys.argv) != 3:
        print("用法: python3 update_task_points_db.py <数据库路径> <新地图名称>")
        print("示例: python3 update_task_points_db.py ~/maps/map_2024-01-15_14-30-25_task_points.db map_2024-01-15_14-30-25")
        sys.exit(1)
    
    db_path = sys.argv[1]
    new_map_name = sys.argv[2]
    
    success = update_task_points_db(db_path, new_map_name)
    sys.exit(0 if success else 1)

if __name__ == "__main__":
    main() 