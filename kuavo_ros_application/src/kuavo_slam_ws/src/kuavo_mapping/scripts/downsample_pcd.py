#!/usr/bin/env python3
import argparse
import open3d as o3d
import numpy as np

parser = argparse.ArgumentParser()
parser.add_argument("--pcd", type=str, default="", help="PCD file path")
parser.add_argument("--voxel_size", type=float, default=0.2, help="Voxel size")
parser.add_argument("--output_file", type=str, default="", help="Output file")
parser.add_argument("--z_min", type=float, default=-0.5, help="Minimum height")
parser.add_argument("--z_max", type=float, default=0.5, help="Maximum height")
args = parser.parse_args()

def modify_pcd(pcd, voxel_size, output_file, z_min, z_max):
  pcd = o3d.io.read_point_cloud(args.pcd)
  # # ✅ 高度裁剪
  # points = np.asarray(pcd.points)
  # keep = (points[:, 2] >= z_min) & (points[:, 2] <= z_max)
  # pcd.points = o3d.utility.Vector3dVector(points[keep])

  # ✅ 下采样
  pcd_down = pcd.voxel_down_sample(voxel_size=args.voxel_size)  # 根据地图密度调整 voxel_size
  o3d.io.write_point_cloud(output_file, pcd_down)

if __name__ == "__main__":
  modify_pcd(args.pcd, args.voxel_size, args.output_file, args.z_min, args.z_max)