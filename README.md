# navmesh

导航网格寻路内核：凸多边形区域、由公共边组成的连通图、漏斗算法平滑与路点去重，纯标准库 Python 3。

## 内容

- navmesh/core.py：NavMesh（区域、公共边、区域定位）、find_region_path、find_path、funnel、simplify、path_length
- tests/test_core.py：unittest 用例

区域按逆时针方向给出顶点；两个区域共享一整条边时互为邻居。区域里的点用坐标给出。

## 跑测试

在项目根目录执行：

    python3 -m unittest discover -s tests -v
