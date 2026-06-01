#!/bin/bash

pcd_filepath=$(rospack find fast_lio)/PCD/scans.pcd

rm $pcd_filepath || true

echo "✅ PCD 文件已删除"

exit 0