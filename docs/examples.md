# 示例指南

[返回 README](../README.md) · [API 与扩展指南](api.md) · [使用限制](limitations.md)

本目录包含 7 个示例子目录、8 个可执行目标。默认构建方式见 [README](../README.md#build)；以下命令均从本目录根路径执行。

<a id="targets"></a>

## 示例索引

| 可执行目标 | 源码 | 演示内容 |
| --- | --- | --- |
| `circle_fit` | [circle_fit.cc](../examples/data_fitting/circle_fit.cc) | 圆拟合与多元边 |
| `curve_fit` | [curve_fit.cc](../examples/data_fitting/curve_fit.cc) | 指数曲线拟合；解析导数存在[已知问题](limitations.md#known-issues) |
| `pose_graph` | [pose_graph.cc](../examples/pose_graph/pose_graph.cc) | 读取 g2o 位姿图并优化 |
| `sba_demo` | [sba_demo.cc](../examples/sba/sba_demo.cc) | 位姿、路标联合优化与 Schur 消元 |
| `bal_example` | [bal_example.cc](../examples/bal/bal_example.cc) | BAL 数据集解析与 BA |
| `gicp_demo` | [gicp_demo.cc](../examples/icp/gicp_demo.cc) | GICP 点云配准 |
| `sphere` | [sphere.cc](../examples/sphere/sphere.cc) | 球面位姿图 |
| `incremental` | [incremental.cc](../examples/incremental/incremental.cc) | 增量结构复用与顶点容量限制 |

`SetMarginalized(true)` 在 BA 中表示线性求解时的 Schur 消元；历史滑窗先验需要另外实现。增量示例受[增量模式限制](api.md#incremental)及[删除路径问题](limitations.md#known-issues)影响，使用前应先修复并验证。

<a id="running"></a>

## 运行方式

圆拟合使用程序生成的数据：

```bash
./bin/circle_fit --num_points=100 --max_iterations=10
```

g2o 位姿图和 BAL 示例需要准备外部数据文件；本副本不包含数据集。将下述路径替换为实际文件位置：

```bash
./bin/pose_graph --g2o_file=/path/to/graph.g2o --is_3d_pose=true
./bin/bal_example --graph_input=/path/to/problem.txt
```

可用 `--help` 查看示例参数；具体参数和默认路径以各示例的 `DEFINE_*` 声明为准。曲线拟合的完整修正版程序见 [README 快速开始](../README.md#quick-start)。

<a id="pose-graph"></a>

## 位姿图优化

程序读取 g2o 格式的位姿图并优化（`--is_3d_pose` 切换 2D/3D）。接入流程为：按行解析顶点和边，检查端点存在及信息矩阵有效性，处理参考系约束，再初始化求解。下面仅展示**已有非空且连通的 SE3 图**的求解与结果提取；解析代码见源文件，噪声转换须按下文核验。

```cpp
// optimizer 已配置为非增量模式，并已装载 SE3 顶点和约束。
// 只有相对约束时选取一个参考顶点；已有绝对先验时须调整此锚定策略。
optimizer->GetVertices().begin()->second->SetFixed(true);
optimizer->InitializeOptimization();
optimizer->SetVerbose(true);
optimizer->Optimize(10);

// GetVertices() 返回基类指针，须转换到实际类型后读取 Estimate()。
for (const auto& item : optimizer->GetVertices()) {
    const auto pose = std::dynamic_pointer_cast<lightning::miao::VertexSE3>(item.second);
    if (pose) {
        const lightning::SE3 T = pose->Estimate();
        // 按 item.first 保存 T；id 的遍历顺序不保证按时间排列。
    }
}
```

g2o 数据格式要点：

- 顶点行：`VERTEX_SE3:QUAT <id> <t(3)> <q(xyzw)>`；边行：`EDGE_SE3:QUAT <id1> <id2> <t(3)> <q(xyzw)> <信息矩阵上三角 21 项>`。
- g2o 的四元数存储顺序是 `x y z w`，而 Sophus 构造是 `Quatd(w, x, y, z)`，注意重排。

**残差坐标转换**：当前 `EdgeSE3` 使用 `[t_delta, vec(q_delta)]`，小角度下 `vec(q_delta) ≈ δθ/2`。若输入协方差 Σ 对应 `[t_delta, δθ]`，则一阶变换为

$$
A=\operatorname{diag}(I_3,\tfrac12 I_3),\qquad
\Sigma_q=A\Sigma_\theta A^\top,\qquad
\Omega_q=A^{-\top}\Omega_\theta A^{-1}.
$$

在这组特定坐标约定下，旋转**协方差**块乘 `0.25`，旋转**信息**块乘 `4`，平移与旋转的信息交叉块乘 `2`。当前 `pose_graph.cc` 却直接将旋转信息块乘 `0.25`；这段历史实现不能作为通用转换规则，也未处理完整交叉块。读取 g2o 时必须核对数据生产方的残差定义、四元数分支和信息矩阵坐标，不能只凭文件格式套倍率。

<a id="compatibility"></a>

## 独立构建兼容层

本副本通过以下文件兼容源工程的依赖位置与原示例写法：

| 位置 | 用途 |
| --- | --- |
| `deps/Sophus/` | 随副本提供的 Sophus 头文件；源工程中位于 `thirdparty/` |
| `deps/common/eigen_types.h` | 来自 lightning_lm 的 `src/common/`；提供 Eigen/Sophus 类型别名，并兼容全局与 `lightning::miao` 下的旧写法 |
| `deps/common/std_types.h` | 来自源工程的通用类型定义 |
| `deps/utils/timer.h`、`timer.cc` | 来自 lightning_lm 的 `src/utils/`；兼容 `lightning::miao::Timer` |
| `deps/core/common/eigen_types.h` | 兼容个别示例的旧包含路径 |
| `deps/compat.h` | 建立 `namespace miao = lightning::miao;`，由 CMake 的 `-include` 注入编译单元 |

在 lightning_lm 工程内使用库本体时，依赖由源工程组织，链接目标为 `miao.core`。本副本不包含 `src/core/localization/pose_graph/pgo_impl.cc`，不能据此验证源工程后端的参数和运行效果。局部融合的独立建模说明见[局部融合指南](local-fusion.md)。
