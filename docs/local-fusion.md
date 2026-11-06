# 局部固定规模因子图与运动约束

[返回 README](../README.md) · [API 与扩展指南](api.md) · [使用限制](limitations.md)

本页给出基于 miao 构建局部融合后端的建模方案，涵盖 LIO 相对位姿、轮速和历史先验。`EdgeRelativePoseLog`、`EdgeForwardSpeed` 和 `SolveLocalWindow` 均为文档示例，需要调用方自行集成。

- [适用范围](#scope)
- [坐标与时间约定](#conventions)
- [联合目标与噪声](#objective)
- [LIO 相对位姿](#lio)
- [轮速因子](#wheels)
- [其他运动约束](#motion)
- [窗口管理](#window)
- [边缘化先验](#marginalization)
- [接入验证](#validation)
- [参考资料](#references)

<a id="scope"></a>

## 适用范围与能力边界

**可以将 miao 用作局部固定规模融合后端的非线性最小二乘求解层。** 最简状态为最近 N 个关键帧位姿：

$$
\mathcal X_k=\{T_{WB_i}\}_{i=k-N+1}^{k},\qquad T_{WB_i}\in SE(3).
$$

启动时允许少于 N 帧。“固定规模”通常指窗口状态数有上限；还应限制每帧因子数、路标数、标定变量和先验维度，才能约束内存与计算量。固定帧数窗口不等于固定时间窗口：关键帧频率变化时，时间跨度也会变化。即使规模有界，迭代耗时仍须实测。

| 方案 | 窗口外的信息 | 当前接入方式 |
| --- | --- | --- |
| 截断窗口、固定边界位姿 | 不保留被删因子的完整历史信息；边界估计作为确定值 | 非增量模式，每轮重建小图，适合作为基础实现 |
| 固定滞后平滑、保留边缘化先验 | 保留历史目标在边缘化线性化点处的高斯近似 | 需自行实现边缘化管理、先验因子与线性化点维护 |
| `incremental_mode_ + max_vertex_size_` | 删除旧顶点和关联边，不自动生成历史先验 | 结构复用机制，受[增量模式](api.md#incremental)与[使用限制](limitations.md)限制 |

固定滞后平滑通常通过边缘化旧变量保留历史约束，区别于单纯删除旧状态；参见 [GTSAM 固定滞后平滑文档](https://borglab.github.io/gtsam/fixedlagsmoother/)。这里的工程方案是根据当前 miao 接口作出的设计建议，不是该库已经实现的功能声明。

<a id="conventions"></a>

## 坐标系、时间与状态约定

全文对局部融合采用以下约定：

- `T_AB` 将 B 系坐标映射到 A 系：`p_A = R_AB p_B + t_AB`；`T_WB_i` 表示时刻 `t_i` 的机体在世界系中的位姿。
- B 系采用 x 向前、y 向左、z 向上的右手系；L 为 LIO 输出所对应的传感器系，C 为轮式底盘参考系，O 为连续的 LIO 里程计参考系。
- 固定外参 `T_BL` 将 L 系映射到 B 系，`T_BC` 将 C 系映射到 B 系。`T_WC_i = T_WB_i T_BC`。若 LIO 本身输出 B 系位姿，不应重复应用外参。
- 长度用米、时间用秒、角度用弧度。所有因子对应同一组关键帧时刻；高频轮速应积分或聚合到 `[t_i,t_j]`，瞬时采样不能直接充当长时间区间的平均速度。
- 时间偏差、外参误差、传感器原点的杆臂效应必须在预处理或模型中处理。尤其转弯时，不同安装位置的线速度不能直接互换。

纯位姿状态适合把 LIO 输出作为外部相对位姿观测的**松耦合**融合。若需要原始 IMU 预积分、偏置估计及其与位姿的联合更新，通常需扩展为 `x_i = (R_i,p_i,v_i,b_gi,b_ai)`，局部更新维度为 15，并实现对应顶点和因子；仅有 `VertexSE3` 和相对位姿边不能构成完整的紧耦合 LIO。IMU 预积分与偏置处理参见 [Forster 等人的原始论文](https://arxiv.org/abs/1512.02363)。

<a id="objective"></a>

## 联合目标、规范自由度与噪声

在各测量误差条件独立、噪声模型有效的近似下，可构造：

$$
\min_{\mathcal X_k}\;\frac12\|r_p\|^2
+\frac12\sum_{(i,j)\in\mathcal E_L}\rho_L\!\left(\|r^L_{ij}\|_{\Omega^L_{ij}}^2\right)
+\frac12\sum_{f\in\mathcal F_w}\rho_w\!\left(\|r^w_f\|_{\Omega^w_f}^2\right)
+\frac12\sum_{f\in\mathcal F_m}\rho_m\!\left(\|r^m_f\|_{\Omega^m_f}^2\right)
+\frac12\sum_{f\in\mathcal F_a}\rho_a\!\left(\|r^a_f\|_{\Omega^a_f}^2\right),
\qquad \|r\|_\Omega^2=r^\top\Omega r.
$$

其中 `r_p` 为已白化的历史先验（如启用），L、w、m、a 分别代表 LIO、轮式观测、运动模型及绝对观测；没有某类信息时删除相应项。内置 χ² 统计通常不含前面的 `1/2`，不影响最优点。

仅使用相对位姿和轮速时，完整 SE3 相对图对整体刚体变换不敏感，存在六维规范自由度。可在非增量模式下固定窗口首帧，或提供足以约束相应自由度的绝对先验；局部绝对观测如单一高度或单点 GNSS 位置未必消除全部自由度。先验已确定参考系时，不应再随意硬固定另一个位姿。图中断开的分量须分别处理。

固定首帧是条件化近似，不等同于把该帧及历史信息正确边缘化。首帧不确定性不会自动传播；若没有历史先验或绝对观测，局部优化仍会累计漂移。LM 的阻尼能改善数值求解，但不能代替物理可观测性。

信息矩阵须对应**实际残差的坐标、排列和单位**。正定协方差可用 LLT/LDLT 分解求解得到 Ω；不要把协方差直接传给 `SetInformation()`。对不可观测方向使用低维因子或显式可观测子空间，不能用零方差假装高精度；任意填极小正数也不等于正确建模。若更换残差坐标 `r_new ≈ J r_old`，需同步变换 `Σ_new = J Σ_old Jᵀ`，包括交叉项。

<a id="lio"></a>

## LIO 相对位姿因子

若 LIO 在连续里程计系 O 中提供 `T_OL_i`，先转到机体系，再形成相对测量：

$$
T^{LIO}_{OB_i}=T^{LIO}_{OL_i}T_{BL}^{-1},\qquad
Z^L_{ij}=\left(T^{LIO}_{OB_i}\right)^{-1}T^{LIO}_{OB_j}
=T_{BL}\left(T^{LIO}_{OL_i}\right)^{-1}T^{LIO}_{OL_j}T_{BL}^{-1}.
$$

测量方向为 `Z_ij = T_BiBj`，对应 `vertex[0]=i`、`vertex[1]=j`，预测为 `T_WB_i⁻¹ T_WB_j`。选择[SE3 相对位姿因子](api.md#relative-pose-factor)的 `EdgeRelativePoseLog` 时，定义：

$$
\Delta^L_{ij}=(Z^L_{ij})^{-1}T_{WB_i}^{-1}T_{WB_j},\qquad
r^L_{ij}=\begin{bmatrix}t(\Delta^L_{ij})\\\operatorname{Log}(R(\Delta^L_{ij}))\end{bmatrix}\in\mathbb R^6.
$$

也可复用内置 `EdgeSE3`，但它把后三维换成四元数虚部，必须使用与之匹配的协方差；与旋转向量残差的近似转换见[位姿图示例](examples.md#pose-graph)。

不能把两帧 LIO 的绝对位姿协方差简单相加当作相对协方差。设两帧原始位姿误差的协方差分别为 `Σ_ii`、`Σ_jj`，跨帧协方差为 `C_ij`，测量构造及残差映射的联合雅可比为 `J=[J_i J_j]`，则一阶传播为：

$$
\Sigma^L_{ij}\approx J_i\Sigma_{ii}J_i^\top+J_j\Sigma_{jj}J_j^\top
+J_i C_{ij}J_j^\top+J_j C_{ij}^\top J_i^\top.
$$

若前端无法提供跨帧相关性，应说明近似假设，并通过相对误差统计标定噪声、设置合理噪声下限；不能宣称这种近似严格一致。

接入时还应满足：

- 使用同一连续里程计参考系生成相对测量；重定位、回环校正或前端重置造成的位姿跳变不能直接成为运动观测。
- 将前端协方差从其左/右扰动定义、传感器坐标与变量顺序转换到本因子的残差空间。退化方向应降低信息或仅保留可观测分量。
- 同一前端的绝对输出、相对输出，以及相邻区间的输出可能相关。若 LIO 已融合轮速或同一批 IMU 数据，后端再次把这些数据作为独立因子会重复计数。严格处理需联合相关噪声模型或从原始独立观测构图；协方差膨胀仅是需验证的工程近似。

<a id="wheels"></a>

## 轮速计与轮式里程计因子

轮式因子应匹配实际硬件和运动假设。轮式里程计的低维预积分、约束维数及其嵌入 SE3 状态的方法可参照[轮式机器人 LiDAR–IMU–里程计融合论文](https://www.mdpi.com/1424-8220/22/13/4778)；以下给出适用于本文档约定的建模选择。

**情况 A：差速底盘的平面相对里程计。** 在无侧滑、轮径与轮距已标定的假设下，左右轮角速度产生

$$
v=\tfrac12(r_R\dot\phi_R+r_L\dot\phi_L),\qquad
\omega_z=\frac{r_R\dot\phi_R-r_L\dot\phi_L}{b},
$$

其中轮子转动正方向须统一为车辆前进方向，`b` 为轮距。对轮转角增量使用同样关系得到位移与转角增量，按底盘运动学在 SE2 上积分到关键帧区间，并传播协方差。不能直接以 `v Δt` 代替任意转弯轨迹的二维位移。

预测底盘相对位姿为

$$
\widehat T_{C_iC_j}=(T_{WB_i}T_{BC})^{-1}(T_{WB_j}T_{BC}).
$$

仅在底盘近似平面运动且局部竖直轴约定有效时，定义 `π(T)=(t_x,t_y,yaw(R)) ∈ SE2`。对积分得到的 `Z^w_ij ∈ SE2`，可取

$$
E^w_{ij}=(Z^w_{ij})^{-1}\pi(\widehat T_{C_iC_j}),\qquad
r^w_{ij}=[t_x(E^w_{ij}),t_y(E^w_{ij}),\operatorname{wrap}(yaw(E^w_{ij}))]^\top.
$$

该因子残差为三维，连接两个 `VertexSE3`，需要自定义 `BaseBinaryEdge<3, SE2, VertexSE3, VertexSE3>`。若整个状态本身是 SE2，可复用 `EdgeSE2`。禁止将平面观测补齐为 SE3 后，给 z、roll、pitch 设置很大的信息；这些分量并未被轮编码器直接观测。坡道、大俯仰或非平面运动应改用符合底盘几何的模型。

**情况 B：只有前向轮速。** 对较短区间，在底盘系 C 的起点姿态近似代表区间姿态时，可使用一维平均速度残差：

$$
u^C_{ij}=R_{WC_i}^\top\frac{p_{WC_j}-p_{WC_i}}{\Delta t_{ij}},\qquad
r^v_{ij}=e_x^\top u^C_{ij}-\bar v^w_{ij},\quad \Delta t_{ij}>0.
$$

这里公式中的 `u^C_ij` 表示位移差推算的区间速度，不是严格的瞬时速度；转弯较大时起点投影的模型误差不可忽略。应缩短区间、积分完整运动学，或引入速度状态。若使用位移残差 `e_xᵀ R_CiW Δp - Δs`，应配位移方差；在相同近似下 `σ²_Δs = Δt² σ²_v`，不能直接沿用速度方差。

下面是一维因子实现，与[SE3 相对位姿因子](api.md#relative-pose-factor)的 LIO 因子可共同使用。`T_BC`、区间长度与测量方差由调用方提供，代码不做轮速积分和时间同步：

```cpp
#include <cmath>
#include <stdexcept>
#include "core/graph/base_binary_edge.h"
#include "core/types/vertex_se3.h"

class EdgeForwardSpeed
    : public lightning::miao::BaseBinaryEdge<1, double,
          lightning::miao::VertexSE3, lightning::miao::VertexSE3> {
 public:
    EdgeForwardSpeed(double dt, const lightning::SE3& T_BC)
        : dt_(dt), T_BC_(T_BC) {
        if (!(dt_ > 0.0) || !std::isfinite(dt_)) {
            throw std::invalid_argument("dt must be finite and positive");
        }
    }

    void ComputeError() override {
        const auto* vi = static_cast<const lightning::miao::VertexSE3*>(GetVertex(0));
        const auto* vj = static_cast<const lightning::miao::VertexSE3*>(GetVertex(1));
        const lightning::SE3 T_WCi = vi->Estimate() * T_BC_;
        const lightning::SE3 T_WCj = vj->Estimate() * T_BC_;
        const Eigen::Vector3d u_C = T_WCi.so3().inverse()
            * ((T_WCj.translation() - T_WCi.translation()) / dt_);
        error_[0] = u_C.x() - measurement_;
    }

 private:
    double dt_;
    lightning::SE3 T_BC_;
};
```

仅有单路前向轮速时不能凭空生成偏航角速度。差速、Ackermann、滑移转向等底盘需各自的运动学；轮胎打滑、腾空、急转与编码器异常时，应依据质量判据降权或停用因子，并保留噪声下限。鲁棒核不能修正持续的轮径或轮距标定偏差。

<a id="motion"></a>

## 其他运动约束

| 约束 | 一种残差形式 | 使用条件与边界 |
| --- | --- | --- |
| 非完整约束 | `[e_yᵀv_C, e_zᵀv_C]ᵀ` | 作用于符合无侧滑假设的底盘参考点；仅侧向约束或同时垂向约束取决于实际地形与模型 |
| 匀速模型 | `v_j-v_i` | 需要速度状态；按过程噪声与 Δt 设置权重，避免压制真实加减速 |
| 静止约束 | `v_i=0`，或短区间相对运动为零 | 必须先可靠检测静止；轮速读数为零不足以排除滑动或外力拖动 |
| 平面/高度先验 | `nᵀp_i-d` | 法向与高度属于相同参考系；只适用于真实平面条件，可复用 z 高度特例 `EdgeHeightPrior` |
| GNSS 位置 | `p_WB_i+R_WB_i t_BG-z_G` | 建三维位置因子，处理天线杆臂；位置观测不包含完整姿态 |
| 原始 IMU 预积分 | 相对旋转、速度、位置及偏置模型 | 扩展状态与专用预积分实现；不能用简单位姿边替代 |

非完整约束是运动学假设，不是独立传感器测量。若用位姿差近似 `v_C`，须说明其短区间限制；若平面轮式因子已通过“无侧滑”积分生成横向位移，再叠加高度相关的零侧向速度因子会重复强化同一假设。不能对同一轮速区间同时使用完整轮式里程计和独立轮速因子而忽略相关性。

<a id="window"></a>

## 固定规模窗口的基础接入流程

建议先采用**非增量、小图重建、沿用上轮估计值**的方案，明确验证模型后再优化结构复用。此方案会截断窗口外的因子，并对首帧条件化；需要历史信息时再实现[边缘化先验](#marginalization)。

1. 在业务层保存最近 N 个关键帧、连续时间戳及观测记录；状态初值继承上轮结果，新帧可由连续 LIO 增量预测。
2. 创建新的 `Optimizer`、顶点与边。不要把旧图的顶点对象直接放入新图，顶点内部还保存关联边和求解内存信息。
3. 仅加入端点都在当前窗口内的相对因子；根据输入质量选择 LIO、轮速、非完整约束和绝对观测。锚定策略必须明确。
4. 调用 `InitializeOptimization()`、`Optimize()`，检查有限性、代价变化与图结构，再读取结果。初始化返回 true 并不保证图非空或满秩。
5. 按业务帧号保存估计值，下一轮用这些数值初始化新顶点；观测与状态容量由业务层限制。

下面是一个建图函数示例，依赖[SE3 相对位姿因子](api.md#relative-pose-factor)和[轮速因子](#wheels)定义的两个自定义因子和当前工程的编译配置。输入是至少两个位姿初值，以及每个相邻区间的一条 LIO 测量和一条有效平均轮速；传入前应完成时间同步、质量筛选、独立性检查和协方差标定。为明确基础方案，代码硬固定首帧，不携带历史边缘化先验。

```cpp
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include "core/graph/optimizer.h"
#include "core/opti_algo/algo_select.h"
#include "core/robust_kernel/huber.h"

struct LocalMotionInterval {
    lightning::SE3 Z_lio;       // Bi <- Bj
    Eigen::Matrix<double, 6, 6> info_lio; // 对应 [t_delta, Log(R_delta)]
    double dt;                 // 秒
    double mean_forward_speed; // C 系平均前向速度，米/秒
    double speed_variance;     // 包含所用短区间近似的误差，(米/秒)^2
};

std::vector<lightning::SE3> SolveLocalWindow(
    const std::vector<lightning::SE3>& initial,
    const std::vector<LocalMotionInterval>& intervals,
    const lightning::SE3& T_BC) {
    namespace opt = lightning::miao;
    if (initial.size() < 2 || intervals.size() != initial.size() - 1
        || initial.size() > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
        throw std::invalid_argument("invalid local window sizes");
    }
    opt::OptimizerConfig cfg(opt::AlgorithmType::LEVENBERG_MARQUARDT,
        opt::LinearSolverType::LINEAR_SOLVER_SPARSE_EIGEN, false);
    cfg.incremental_mode_ = false;
    auto optimizer = opt::SetupOptimizer<6, -1>(cfg);
    std::vector<std::shared_ptr<opt::VertexSE3>> vertices;

    for (std::size_t i = 0; i < initial.size(); ++i) {
        auto v = std::make_shared<opt::VertexSE3>();
        v->SetId(static_cast<int>(i));
        v->SetEstimate(initial[i]);
        v->SetFixed(i == 0);
        if (!optimizer->AddVertex(v)) throw std::runtime_error("AddVertex failed");
        vertices.push_back(v);
    }
    for (std::size_t i = 0; i < intervals.size(); ++i) {
        const auto& m = intervals[i];
        if (!(m.speed_variance > 0.0) || !std::isfinite(m.speed_variance)
            || !std::isfinite(m.mean_forward_speed)) {
            throw std::invalid_argument("invalid wheel speed measurement");
        }
        auto lio = std::make_shared<EdgeRelativePoseLog>();
        lio->SetVertex(0, vertices[i]);
        lio->SetVertex(1, vertices[i + 1]);
        lio->SetMeasurement(m.Z_lio);
        lio->SetInformation(m.info_lio);
        if (!optimizer->AddEdge(lio)) throw std::runtime_error("AddEdge failed");

        auto wheel = std::make_shared<EdgeForwardSpeed>(m.dt, T_BC);
        wheel->SetVertex(0, vertices[i]);
        wheel->SetVertex(1, vertices[i + 1]);
        wheel->SetMeasurement(m.mean_forward_speed);
        Eigen::Matrix<double, 1, 1> info;
        info(0, 0) = 1.0 / m.speed_variance;
        wheel->SetInformation(info);
        auto huber = std::make_shared<opt::RobustKernelHuber>();
        huber->SetDelta(3.0); // 1 维白化残差范数阈值；示例值，需用数据校准
        wheel->SetRobustKernel(huber);
        if (!optimizer->AddEdge(wheel)) throw std::runtime_error("AddEdge failed");
    }
    if (!optimizer->InitializeOptimization()) {
        throw std::runtime_error("InitializeOptimization failed");
    }
    optimizer->ComputeActiveErrors();
    const double cost_before = optimizer->ActiveRobustChi2();
    const int iterations = optimizer->Optimize(10);
    optimizer->ComputeActiveErrors();
    const double cost_after = optimizer->ActiveRobustChi2();
    // 0 可能是已在最优点终止，也可能是失败；此处保守交由上层处理。
    if (iterations <= 0 || !std::isfinite(cost_before) || !std::isfinite(cost_after)
        || cost_after > cost_before + 1e-9 * (1.0 + std::abs(cost_before))) {
        throw std::runtime_error("local optimization requires inspection");
    }
    std::vector<lightning::SE3> result;
    for (const auto& v : vertices) {
        if (!v->Estimate().matrix().allFinite()) {
            throw std::runtime_error("non-finite pose estimate");
        }
        result.push_back(v->Estimate());
    }
    return result;
}
```

该函数求解一次给定窗口；**N 帧容量控制在调用方，不由 `max_vertex_size_` 实现**。示例默认每个区间都有有效且提供六自由度信息的 LIO 因子；若某段失效或降为部分观测，应重新检查连通性与秩。`info_lio` 的有限性、对称正定性以及输入位姿有效性须在调用前检查。小图可另测 `DENSE + true`，不能只改求解器枚举而遗漏匹配的配置。

<a id="marginalization"></a>

## 保留历史信息：边缘化先验

需要固定滞后平滑时，应在移出旧状态之前构建历史先验。在线性化点 `x̄` 附近，统一采用

$$
\frac12\|r+J\delta x\|^2
=\frac12\delta x^\top H\delta x-b^\top\delta x+c,
\qquad H=J^\top J,\quad b=-J^\top r,\quad H\delta x=b.
$$

这里 `r,J` 已包含白化以及本次冻结的鲁棒权重。将待移除变量记为 m、保留的边界变量记为 r（下标 r 不表示残差），对需要吸收的因子组装

$$
\begin{bmatrix}H_{mm}&H_{mr}\\H_{rm}&H_{rr}\end{bmatrix}
\begin{bmatrix}\delta x_m\\\delta x_r\end{bmatrix}
=\begin{bmatrix}b_m\\b_r\end{bmatrix}.
$$

在消元块可解时，Schur 补为

$$
H_p=H_{rr}-H_{rm}H_{mm}^{-1}H_{mr},\qquad
b_p=b_r-H_{rm}H_{mm}^{-1}b_m.
$$

实现时用分解求解，不显式求逆；秩亏时要处理规范自由度或使用秩揭示的 QR/SVD。不要将 LM 的临时阻尼作为真实历史信息永久吸收到先验。

可构造平方根先验

$$
r_p(x_r)=J_p\,\delta x_r(x_r;\bar x_r)+r_{p0},\qquad
J_p^\top J_p=H_p,\qquad J_p^\top r_{p0}=-b_p.
$$

对 `VertexSE3`，与其更新规则一致的一种局部坐标是

$$
\delta x_i(x_i;\bar x_i)=\begin{bmatrix}p_i-\bar p_i\\\operatorname{Log}(\bar R_i^\top R_i)\end{bmatrix}.
$$

非零旋转偏移时，局部坐标对当前旋转增量的导数一般不再是单位矩阵；自定义先验的解析雅可比须包含该链式变换，或用已验证的数值雅可比。边缘化后必须保留先验使用的参考状态和变量排列。原始被移除变量已丢失，不能把旧先验当作原始非线性测量任意重新线性化；改变参考点时应同步变换先验，并评估大状态修正带来的近似误差。

吸收规则必须避免重复计数：用所有涉及待移除变量的因子，以及需要传递的旧先验构建新先验；成功生成后，从下一窗口删除已经吸收的原因子。仅涉及保留变量且未被吸收的因子继续保留。不能对整个旧窗口生成先验后，又把其中未删除的测量原样加入一次。

**`SetMarginalized(true)` 不实现上述生命周期。** 当前 Schur 求解路径针对可独立消元的路标块，并假设 H_ll 各列只有对角块；连续旧位姿之间通常直接耦合，不能直接套用该标记实现通用滑窗边缘化。`core/math/marginal_covariance_cholesky.*` 用于求边缘协方差，也不是维护历史先验的接口。实现通用多帧先验还需考虑运行期可变连接数；现有 `BaseFixedSizedEdge` 的顶点数量在编译期确定，不能声称已有通用动态先验因子。

<a id="validation"></a>

## 模型与接入验收

在声明“融合有效”前，至少应完成以下验证；这些是后续实现的验收要求，不是对当前代码的测试结果。

- **约束方向与外参**：用已知刚体运动合成测量，正确状态应产生零残差；调换 i/j、取反外参或改变轴向后应出现可解释的非零误差。
- **雅可比与尺度**：解析雅可比对照与顶点更新规则一致的中心差分；测试不同尺度、非零姿态、杆臂及角度分支附近的行为。
- **轮式模型适用性**：覆盖直行、倒车、转弯、不同 Δt、静止、坡道、打滑与丢包；区分真实观测不足、时间误差和运动近似误差。
- **窗口与先验**：持续滑动多轮，状态数与因子数保持有界，无悬空指针；如实现边缘化，先在线性高斯问题中对照整图解，再检查非线性情况下的近似与重复计数。
- **融合收益与一致性**：对比 LIO 单独使用和加入轮速等约束的结果，统计轨迹误差、按维数归一化的白化残差及耗时分位数；专门检查前端已融合相同传感器时的过度自信。

<a id="references"></a>

## 参考资料

- 当前实现依据：`core/common/config.h`、`core/opti_algo/algo_select.h`、`core/graph/optimizer.cc`、`core/graph/graph.cc`、`core/graph/base_fixed_sized_edge.hpp`、`core/solver/block_solver.hpp` 和 `core/types/`。库能力与示例限制应优先按这些文件核对。
- 固定窗口和边缘化的概念：[GTSAM FixedLagSmoother](https://borglab.github.io/gtsam/fixedlagsmoother/)。该资料解释算法概念，不代表 miao 提供相同 API。
- IMU 预积分、流形误差与偏置处理：[Forster et al., On-Manifold Preintegration for Real-Time Visual-Inertial Odometry](https://arxiv.org/abs/1512.02363)。
- 轮式观测、约束维数与地面运动假设：[A Novel LiDAR–IMU–Odometer Coupling Framework for Two-Wheeled Inverted Pendulum (TWIP) Robot Localization and Mapping with Nonholonomic Constraint Factors](https://www.mdpi.com/1424-8220/22/13/4778)。该论文针对特定底盘，不能直接把所有运动假设推广到任意车辆。
