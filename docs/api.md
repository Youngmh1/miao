# API 与扩展指南

[返回 README](../README.md) · [示例](examples.md) · [使用限制](limitations.md)

本页说明当前 `core/` 中的接口、配置和扩展方式。代码片段以 C++17 为基础；命名空间别名统一为 `namespace miao = lightning::miao;`。除标为完整程序的示例外，片段需要放入调用方已有的工程上下文。

- [核心概念](#concepts)
- [API 参考](#api-reference)
- [内置类型](#builtin-types)
- [算法与求解器](#solvers)
- [鲁棒核](#robust-kernels)
- [自定义顶点](#custom-vertices)
- [自定义边](#custom-edges)
- [增量模式](#incremental)

<a id="concepts"></a>

## 核心概念

图优化把最小二乘问题表述为因子图：

- **Vertex（顶点）**：一个待优化的变量（如某帧位姿、路标点、曲线参数）。顶点在 Hessian 矩阵中占据一个矩阵块，具有 `id` 与 Hessian 列号；可选 `Fixed`（固定不优化）与 `Marginalized`（参与 Schur 消元）标记。
- **Edge（边）**：一条约束，连接若干顶点，携带测量值 `measurement` 与信息矩阵 `information`。优化中反复执行：`ComputeError()` 计算误差 → `LinearizeOplus()` 计算雅可比 → 累积出 H 与 b。
- **鲁棒核（RobustKernel）**：对误差的 χ² 做重加权，抑制外点。
- **Optimizer**：图的容器 + 优化驱动器，负责 Hessian 索引分配、BlockSolver 组装、迭代收敛判断。

继承体系（均在 `core/graph/`）：

| 顶点侧 | 边侧 |
| --- | --- |
| `Vertex`（接口基类） | `Edge`（接口基类） |
| `BaseVertex<D, T>`（编译期维度 + 估计值类型） | `BaseEdge` → `BaseFixedSizedEdge` |
| `BaseVecVertex<D>`（向量参数便捷版） | `BaseUnaryEdge<D, M, V>`（一元边） |
| `VertexSE2 / VertexSE3 / VertexPointXYZ`（内置） | `BaseBinaryEdge<D, E, Vi, Vj>`（二元边） |
| | `EdgeSE2 / EdgeSE3 / 各类 Prior`（内置） |

`BaseVertex<D, T>` 的 `D` 是顶点的局部更新维度（如 SE3 为 6，SE2 为 3），不是估计值存储维度；边模板中的 `D` 则是残差维度，两者不必相等。

所有权约定：`Graph` 用 `shared_ptr` 持有顶点与边；边内部对顶点用裸指针（避免 `weak_ptr` 开销），调用方仍须保证边关联的顶点属于当前图，避免在删除或清空图后复用带有旧关联关系的对象。

<a id="api-reference"></a>

## API 参考

<a id="config"></a>

### OptimizerConfig

[源码：config.h](../core/common/config.h)

| 字段 | 默认值 | 说明 |
| --- | --- | --- |
| `algo_type_` | `LEVENBERG_MARQUARDT` | 优化算法：`GAUSS_NEWTON` / `LEVENBERG_MARQUARDT` / `DOGLEG` |
| `linear_solver_type_` | `LINEAR_SOLVER_SPARSE_EIGEN` | 线性求解器：`LINEAR_SOLVER_DENSE` / `LINEAR_SOLVER_SPARSE_EIGEN` / `LINEAR_SOLVER_PCG` |
| `is_dense_` | `true` | 稠密问题置 `true`；仅可配 `DENSE`，且不允许 marginalized 顶点 |
| `incremental_mode_` | `false` | 增量结构复用，见[增量模式](#incremental) |
| `max_vertex_size_` | `-1` | 增量模式下设为正数时限制顶点数，满容量后替换最早插入的顶点；不生成边缘化先验。`-1` 表示不限，避免使用 0 |
| `parallel_` | `true` | 打开并发，求解器使用多线程 |
| `eps_chi2_` | `1e-4` | 相邻迭代鲁棒代价的绝对变化阈值；受因子数量与权重尺度影响，不是状态精度保证 |

构造：`OptimizerConfig(AlgorithmType, LinearSolverType, bool is_dense)`。注意：`OptimizerConfig{}` 的字段默认组合是 `SPARSE_EIGEN + is_dense_=true`，与工厂校验不兼容；而只传算法的构造函数默认使用 `DENSE + true`。工程代码应显式传入三项，避免混淆。

<a id="setup"></a>

### SetupOptimizer

[源码：algo_select.h](../core/opti_algo/algo_select.h)

```cpp
// 返回创建好的优化器；两个模板参数控制主变量块与 Schur 消元块的矩阵维度
template <int pose_dim = -1, int landmark_dim = -1>
std::shared_ptr<Optimizer> SetupOptimizer(OptimizerConfig options);

// 或对已有 optimizer 对象配置
template <int pose_dim = -1, int landmark_dim = -1>
void SetupOptimizer(Optimizer& optimizer, OptimizerConfig options);
```

它按配置组装 `BlockSolver<BlockSolverTraits<pose_dim, landmark_dim>>`（正数块尺寸在编译期确定）并挂接所选算法。典型取值：纯 SE3 位姿图 `SetupOptimizer<6, -1>`；SE3 位姿 + XYZ 路标 BA 用 `SetupOptimizer<6, 3>`；三维参数拟合用 `SetupOptimizer<3, -1>`。`-1` 是 Eigen 动态维度，并非“禁止路标”的开关；是否启用 Schur 消元由顶点标记决定。混合不同维度的主变量可用 `SetupOptimizer<-1, -1>`，并核验所选求解路径。

<a id="optimizer"></a>

### Optimizer

[源码：optimizer.h](../core/graph/optimizer.h)

| 方法 | 说明 |
| --- | --- |
| `bool InitializeOptimization(int level = 0)` | 建立活动图与 Hessian 索引，求解器矩阵结构在求解首轮构建；全量模式中负 level 选择所有层级，否则选择对应层级；增量模式只接收相同 level 的新增边。图结构或 fixed 标记改变后应重新初始化 |
| `int Optimize(int iterations)` | 返回迭代计数；无活动变量或初始化失败可返回 -1，求解失败可返回 0。需同时检查代价、结果有限性和求解状态 |
| `bool AddVertex(std::shared_ptr<Vertex>)` / `AddEdge(...)` | 添加顶点 / 边；调用方应检查返回值 |
| `bool RemoveEdge(...)` / `RemoveVertex(v, detach)` | 移除边 / 顶点；当前删除路径存在[已知问题](limitations.md#known-issues) |
| `std::shared_ptr<Vertex> GetVertex(int id)` | 按 id 取顶点（不存在返回 nullptr） |
| `GetVertices()` / `GetEdges()` | 全部顶点（id→ptr 映射）/ 边集合 |
| `void SetVerbose(bool)` | 打印迭代编号、鲁棒 χ² 与活动边数；精确耗时需另外统计 |
| `void SetForceStopFlag(bool* flag)` | 外部强制停止标志，每轮迭代检查 |
| `Push()` / `Pop()` / `DiscardTop()` | 顶点估计值备份栈（可回滚试探性优化） |
| `double ActiveChi2()` / `ActiveRobustChi2()` | 当前激活图的 χ²（后者含鲁棒权重） |
| `NewVertices()` / `NewEdges()` / `ClearNewElements()` | 增量模式下获取/清空新增元素（见[增量模式](#incremental)） |
| `void Clear()` | 清空整图 |

<a id="vertex"></a>

### Vertex

[源码：vertex.h](../core/graph/vertex.h)、[base_vertex.h](../core/graph/base_vertex.h)

| 接口 | 说明 |
| --- | --- |
| `SetId(int)` / `GetId()` | 顶点 id（用户自定义，需在图内唯一） |
| `SetEstimate(T)` / `Estimate()` | 设置 / 读取估计值，类型为模板参数 `T` |
| `SetFixed(bool)` | 固定顶点（如第一帧位姿），不参与优化 |
| `SetMarginalized(bool)` | 标记为本轮线性求解的 Schur 消元块（通常用于相互不直接连接的路标点）；不表示滑窗删除或自动生成历史先验 |
| `OplusImpl(const double*)` | 增量更新接口，**自定义顶点必须实现**（见[自定义顶点](#custom-vertices)） |
| `Vertex::Dimension()` | 通过基类接口读取局部更新维度；派生模板同时定义静态常量 `Dimension`，会隐藏同名函数 |

<a id="edge"></a>

### Edge

[源码：edge.h](../core/graph/edge.h)、[base_edge.h](../core/graph/base_edge.h)

| 接口 | 说明 |
| --- | --- |
| `SetVertex(size_t i, std::shared_ptr<Vertex>)` | 关联第 i 个顶点（一元边 0，二元边 0/1） |
| `GetVertex(size_t i)` | 取顶点裸指针 |
| `SetMeasurement(const M&)` / `GetMeasurement()` | 测量值 |
| `SetInformation(const InfoMat&)` / `Information()` | 信息矩阵（协方差之逆），维度 D×D |
| `SetRobustKernel(std::shared_ptr<RobustKernel>)` | 设置鲁棒核（见[鲁棒核](#robust-kernels)） |
| `SetLevel(int)` / `Level()` | 边层级，默认 0；选择规则及增量模式差异见[Optimizer 接口](#optimizer) |
| `ComputeError()` / `LinearizeOplus()` | 必须实现误差；`BaseFixedSizedEdge` 已提供默认数值雅可比，也可覆盖为解析雅可比（见[自定义边](#custom-edges)） |

<a id="builtin-types"></a>

## 内置类型

`core/types/` 已包含以下类型；实际使用须注意[使用限制](limitations.md)列出的实现问题：

| 类型 | 基类 | 维度 D | 测量/估计类型 | 用途 |
| --- | --- | --- | --- | --- |
| `VertexSE2` | `BaseVertex<3, SE2>` | 3 | SE2 | 2D 位姿顶点 |
| `VertexSE3` | `BaseVertex<6, SE3>` | 6 | SE3 | 3D 位姿顶点 |
| `VertexPointXYZ` | `BaseVecVertex<3>` | 3 | Vec3d | 3D 路标点顶点（可设 Marginalized） |
| `EdgeSE2` | `BaseBinaryEdge<3, SE2, V2, V2>` | 3 | SE2 | 2D 相对位姿约束 |
| `EdgeSE3` | `BaseBinaryEdge<6, SE3, V3, V3>` | 6 | SE3 | 3D 相对位姿约束（里程计/回环） |
| `EdgeSE2Prior` | `BaseUnaryEdge<3, SE2, V2>` | 3 | SE2 | 2D 完整位姿先验；纯 GNSS 位置应另建位置因子 |
| `EdgeSE3Prior` | `BaseUnaryEdge<6, SE3, V3>` | 6 | SE3 | 3D 完整位姿先验；位置观测不能伪装成完整位姿 |
| `EdgeHeightPrior` | `BaseUnaryEdge<1, double, V3>` | 1 | double | 高度先验约束 |

<a id="solvers"></a>

## 优化算法与线性求解器

**优化算法**（`AlgorithmType`，见 `core/opti_algo/`）：

| 算法 | 特点 | 建议 |
| --- | --- | --- |
| `GAUSS_NEWTON` | 每步开销通常较低，不保证下降 | 初值较好、问题条件良好时 |
| `LEVENBERG_MARQUARDT` | 阻尼自适应，鲁棒 | **默认推荐** |
| `DOGLEG` | 在信赖域内组合 GN 与最速下降步 | 根据实际收敛性与耗时选择 |

**线性求解器**（`LinearSolverType`）：

| 求解器 | 特点 |
| --- | --- |
| `LINEAR_SOLVER_SPARSE_EIGEN` | Eigen 稀疏 Cholesky，通用首选 |
| `LINEAR_SOLVER_PCG` | 预条件共轭梯度；耗时和精度取决于条件数、预条件与停止阈值 |
| `LINEAR_SOLVER_DENSE` | Eigen 稠密，小规模问题（须 `is_dense=true`） |

组合约束（`SetupOptimizer` 内部会校验）：

- `is_dense=true` 时只允许 `LINEAR_SOLVER_DENSE`，否则 `LOG(FATAL)`；
- `is_dense=false` 时仅选择 `SPARSE_EIGEN` 或 `PCG`；与 `DENSE` 配对会留下空求解器。
- 稠密问题不允许 marginalized 顶点；稀疏问题可选 Schur 消元（把路标顶点 `SetMarginalized(true)`）；
- 正数模板维度必须匹配相应变量分组的实际维度；`-1` 使用动态矩阵。固定窗口长度 N 与单个状态块维度是两个独立概念。

<a id="robust-kernels"></a>

## 鲁棒核

`core/robust_kernel/` 提供 9 种核（经 `core/robust_kernel/robust_kernel_all.h` 一并引入）：`Huber`、`Cauchy`、`Tukey`、`DCS`、`Fair`、`Welsch`、`PseudoHuber`、`Saturated`、`McClure`。

为已有边 `e` 设置 Huber 核和阈值 `delta`：

```cpp
#include "core/robust_kernel/huber.h"

auto rk = std::make_shared<miao::RobustKernelHuber>();
rk->SetDelta(3.0);                 // 平方误差超过 delta² 视为外点降权
e->SetRobustKernel(rk);
```

核作用于马氏平方误差 `s = rᵀΩr`。若 Ω 是正确的逆协方差，Huber 的 delta 是无量纲的白化残差范数阈值，不能直接填入“米”或“弧度”的标准差。可按残差维数与期望概率选择卡方门限，再用数据校准；不同核的阈值含义需分别核对。鲁棒核可减轻外点影响，但不能替代时序、外参和相关性建模。

<a id="custom-vertices"></a>

## 自定义顶点

继承 `BaseVertex<D, T>`，`D` 为流形自由度，`T` 为估计值类型，只需实现增量更新 `OplusImpl`（如何把 `double*` 增量施加到估计值上）。下面给出自定义 SE2 顶点示例，使用显式的三维增量映射。内置 `VertexSE2` 的映射问题见[已知问题](limitations.md#known-issues)：

```cpp
#include "core/graph/base_vertex.h"

class ExampleVertexSE2 : public lightning::miao::BaseVertex<3, lightning::SE2> {
 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    void OplusImpl(const double* update) override {
        Eigen::Map<const Eigen::Vector3d> v(update);
        estimate_.translation() += Eigen::Vector2d(v[0], v[1]);
        estimate_.so2() = lightning::SO2(estimate_.so2().log() + v[2]);
    }
};
```

要点：

- 增量是**流形上的更新**：平移量直接相加，旋转量必须经由李代数叠加（如 `so2().log() + δ` 再指数映射回去），不能直接加到四元数 / 矩阵上。
- 若变量是普通向量（如曲线参数、相机内参），直接用现成的 `BaseVecVertex<D>`，无需写 `OplusImpl`：

```cpp
class VertexParams : public lightning::miao::BaseVecVertex<3> {};
```

<a id="custom-edges"></a>

## 自定义边

<a id="unary-edges"></a>

### 一元边

模板形式为 `BaseUnaryEdge<D, Measurement, VertexType>`：`D` 是残差维度，`Measurement` 是测量值类型，`VertexType` 是关联的顶点类型。

派生类必须实现 `ComputeError()`；`BaseFixedSizedEdge` 提供默认中心差分雅可比，也可覆盖 `LinearizeOplus()` 实现解析导数。以曲线 `a * exp(-lambda * x) + b` 为例，定义预测减观测的残差，其雅可比为：

```text
r = a * exp(-lambda * x) + b - y
J = [exp(-lambda * x), 1, -a * x * exp(-lambda * x)]
```

完整的 `VertexParams`、`EdgePointOnCurve` 与建图流程见 [README 快速开始](../README.md#quick-start)。该文档示例修正了原 `curve_fit.cc` 中对 `lambda` 的导数；原文件的限制见[已知问题](limitations.md#known-issues)。

<a id="relative-pose-factor"></a>

### 二元边：SE3 相对位姿因子

模板参数：`BaseBinaryEdge<残差维度D, 测量类型E, VertexXi, VertexXj>`。雅可比分别写入 `jacobian_oplus_xi_` 与 `jacobian_oplus_xj_`。下面是可沿用默认数值雅可比的 SE3 相对位姿因子；这是新增建模示例，不是内置 `EdgeSE3` 的实现：

```cpp
#include "core/graph/base_binary_edge.h"
#include "core/types/vertex_se3.h"

class EdgeRelativePoseLog
    : public lightning::miao::BaseBinaryEdge<6, lightning::SE3,
          lightning::miao::VertexSE3, lightning::miao::VertexSE3> {
 public:
    void ComputeError() override {
        const auto* vi = static_cast<const lightning::miao::VertexSE3*>(GetVertex(0));
        const auto* vj = static_cast<const lightning::miao::VertexSE3*>(GetVertex(1));
        const lightning::SE3 delta = measurement_.inverse()
            * (vi->Estimate().inverse() * vj->Estimate());
        error_.head<3>() = delta.translation();
        error_.tail<3>() = delta.so3().log();
    }
    // 沿用 BaseFixedSizedEdge 的中心差分；可在验证后换成解析雅可比。
};
```

该残差是 `[delta.translation(), Log(delta.R)]`，**不是完整的 `SE3::log()`**；后者的平移部分还包含 SO3 左雅可比逆。信息矩阵必须匹配实际使用的残差坐标。

自定义边须注意：

- 残差维度由观测模型决定，例如 SE3 顶点上的轮速约束可以只有 1 维，不能一律等同于状态自由度。
- `VertexSE3` 使用 `p ← p + δp`、`R ← R Exp(δθ)`，其中 δp 在世界系、δθ 在当前机体系；它不是完整 SE3 的统一左扰动或右扰动。解析雅可比必须对这一更新规则求导。
- 默认数值微分见 `core/graph/base_fixed_sized_edge.hpp`，中心差分步长为 `1e-9`。它要求顶点可按库内方式复制，并要求残差局部光滑；大尺度变量或角度分支附近需检查精度。
- `EdgeSE3` 的实际旋转残差是误差四元数的虚部，`EdgeSE3Prior` 则使用旋转向量，二者不能直接共用未经变换的信息矩阵。

<a id="incremental"></a>

## 增量模式

增量模式用于复用在线位姿图的索引与矩阵块结构。**每轮仍会对活动图中的因子重新线性化并联合优化活动变量**，并非只优化新增部分，也不能据此宣称具备 iSAM2 式增量分解能力。

```cpp
miao::OptimizerConfig config(miao::AlgorithmType::LEVENBERG_MARQUARDT,
                             miao::LinearSolverType::LINEAR_SOLVER_SPARSE_EIGEN, false);
config.incremental_mode_ = true;
config.max_vertex_size_ = 500;    // 顶点容量限制；替换旧顶点并删除关联边，不保留历史先验
auto optimizer = miao::SetupOptimizer<6, -1>(config);
```

要点：

- 每轮加入新顶点/新边后调用 `InitializeOptimization()` + `Optimize(n)`，增量结构会复用上次的 Hessian 索引（`InitFromLast`），实际耗时收益需要针对图结构评估；
- `NewVertices()` / `NewEdges()` 是待接入求解结构的元素；`BuildStructureInc()` 会消费并清空它们。不要在求解器构建结构前手动 `ClearNewElements()`，也不要在未求解或未消费新增列表时反复初始化同一批元素；
- **限制**（源码注释明确说明）：
  - 不适用于带 Marginalized 顶点的问题（Hessian 索引需要重排）；
  - 不支持 `SetFixed` 的顶点（fixed 节点被跳过同样会触发索引重排）；
  - 打开 `max_vertex_size_` 时，optimizer 会**修改顶点 id**（新顶点顶替被淘汰顶点的 id），调用方需以 `v->GetId()` 的返回值为准重新建立自己的映射。

示例源码见 [incremental.cc](../examples/incremental/incremental.cc)。当前容量替换路径没有 Schur 历史先验；仅有相对因子时还需处理规范自由度。固定规模局部融合优先按[局部融合指南](local-fusion.md)在应用层管理窗口。
