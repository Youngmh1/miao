# 使用限制与已知问题

[返回 README](../README.md) · [API 与扩展指南](api.md) · [示例指南](examples.md)

本页基于当前副本的源码说明实现限制。文档中的修正版代码仅作为接入示例，下表列出的库与示例源码问题仍需单独修复。

<a id="known-issues"></a>

## 已知问题

| 位置 | 问题 | 接入建议 |
| --- | --- | --- |
| [graph.cc](../core/graph/graph.cc) 的 `RemoveEdge(Edge*)` | `edges_.erase(it)` 后仍解引用 `it`，存在无效迭代器访问 | 修复并验证后再使用删除操作；增量容量替换也依赖此路径 |
| [graph.cc](../core/graph/graph.cc) 的 `RemoveVertex` / `DetachVertex` | 遍历顶点的关联边容器时又修改该容器，存在迭代失效风险 | 核验移除与分离路径的迭代安全；局部小图可每轮重建 |
| [vertex_se2.h](../core/types/vertex_se2.h) | 顶点局部更新维度为 3，但输入被映射为 `Vector6` | 按三维增量映射；[自定义顶点示例](api.md#custom-vertices)给出了对应写法 |
| [curve_fit.cc](../examples/data_fitting/curve_fit.cc) | 对 `lambda` 的导数应为 `-a*x*exp(-lambda*x)` | 使用 [README 曲线拟合示例](../README.md#quick-start)中的解析雅可比 |
| [pose_graph.cc](../examples/pose_graph/pose_graph.cc) | 旋转信息块直接乘 `0.25`，与常见旋转向量到四元数虚部的一阶变换不符，且未处理完整交叉块 | 按实际残差坐标转换信息矩阵，见[位姿图示例](examples.md#pose-graph) |
| [config.h](../core/common/config.h)、[algo_select.h](../core/opti_algo/algo_select.h) | 默认 `OptimizerConfig{}` 为 `SPARSE_EIGEN + is_dense_=true`，不能通过工厂校验 | 显式指定算法、求解器和 `is_dense_`，使用[有效组合](api.md#solvers) |

<a id="lifetime"></a>

## 标识与对象生命周期

顶点 ID 必须在图内唯一。`Graph` 通过 `shared_ptr` 持有顶点和边，但边对关联顶点使用裸指针；调用方应保证这些顶点属于当前图，且关联关系在使用期间有效。

重建图时，保留估计值和测量数据，重新创建顶点、边及关联关系。顶点对象包含关联边与求解内存信息，不宜直接跨图复用。删除或清空图后，也不应复用仍携带旧关联关系的对象。

增量容量替换会改写新顶点的 ID。业务帧号、时间戳和图内 ID 应分别管理，并按 `GetId()` 更新映射。

<a id="active-graph"></a>

## 活动图与求解配置

添加或删除约束、修改顶点固定标记后，应重新初始化活动图和求解结构。全量模式下，负 `level` 选择全部层级；增量模式只接收匹配层级的新增边，详见 [Optimizer 接口](api.md#optimizer)。

稠密模式仅搭配 `LINEAR_SOLVER_DENSE`，且不允许 marginalized 顶点；非稠密模式选择 `LINEAR_SOLVER_SPARSE_EIGEN` 或 `LINEAR_SOLVER_PCG`。模板块维度需要与对应变量的局部更新维度匹配。

增量模式不支持固定顶点或 marginalized 顶点，也不自动维护历史先验。`max_vertex_size_` 只限制顶点容量，不等价于固定滞后平滑器，详见[增量模式](api.md#incremental)。

<a id="validation"></a>

## 结果检查

`InitializeOptimization()` 返回 `true` 不保证图非空、连通或满秩；`Optimize()` 的迭代计数与代价下降也不足以证明模型正确。接入时应检查：

- **输入与图结构**：测量和信息矩阵有效，顶点端点存在，图的连通性及规范自由度得到处理。
- **数值结果**：状态、残差与代价有限，代价变化合理，停止条件符合业务精度要求。
- **模型与性能**：白化残差符合噪声假设；求解耗时和传感器失效时的行为满足要求。

`ActiveChi2()` / `ActiveRobustChi2()` 汇总已计算的误差。需要读取当前状态的代价时，先调用 `ComputeActiveErrors()`。

<a id="scope"></a>

## 示例与内置功能的边界

[API 指南](api.md#relative-pose-factor)中的 `EdgeRelativePoseLog`，以及[局部融合指南](local-fusion.md)中的 `EdgeForwardSpeed`、`SolveLocalWindow` 均为文档示例，未加入库源码。传感器时间同步、预积分、历史先验和窗口管理需要应用自行实现。

`SetMarginalized(true)` 表示当前线性系统中的 Schur 消元分组，不负责滑窗删除或历史信息保留；边缘协方差接口也不等价于历史边缘化先验。
