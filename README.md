# miao

miao 是一个 C++17 图优化库，采用“顶点表示优化变量、边表示观测约束”的建模方式，求解非线性最小二乘问题，可用于位姿图优化、束调整（Bundle Adjustment，BA）、点云配准和参数拟合。库的命名空间为 `lightning::miao`。

本目录是 **lightning_lm 中 miao 库的独立可编译副本**：库源码取自 `src/core/miao/`，配套头文件位于 `deps/`，可脱离 lightning_lm 构建。

[依赖与构建](#build) · [CMake 接入](#cmake) · [快速开始](#quick-start) · [示例](docs/examples.md) · [API 参考](docs/api.md) · [使用限制](docs/limitations.md)

<a id="features"></a>

## 功能

| 类别 | 支持内容 |
| --- | --- |
| 优化算法 | Gauss–Newton、Levenberg–Marquardt（LM）、Dogleg |
| 线性求解器 | Eigen 稠密求解器、Eigen 稀疏 Cholesky、预条件共轭梯度（PCG） |
| 建模接口 | 自定义顶点、一元/二元/多元边、解析或数值雅可比 |
| 内置类型 | SE2/SE3 位姿、三维点、相对位姿约束和部分先验 |
| 鲁棒核 | Huber、Cauchy、Tukey 等 9 种核函数 |
| 结构复用 | 增量模式及顶点容量限制；适用条件见[增量模式](docs/api.md#incremental) |

局部融合后端可在此求解层上扩展。LIO/轮速因子、传感器预积分和历史边缘化先验的管理需要调用方实现，详见[局部融合指南](docs/local-fusion.md)。

当前副本的删除路径、SE2 增量映射及部分示例存在[已知问题](docs/limitations.md#known-issues)，接入相关功能前应先阅读对应说明。

<a id="build"></a>

## 依赖与构建

### 环境要求

| 依赖 | 要求与用途 |
| --- | --- |
| C++ 编译器 | 支持 C++17；下述构建方式面向 Linux/GCC |
| CMake | 工程最低要求 3.10；本文的 `-S`/`-B` 命令要求 3.13 或更高版本 |
| Eigen3 | 矩阵运算，需自行安装 |
| Sophus | 位姿与旋转类型，头文件已随 `deps/Sophus/` 提供 |
| glog、TBB | 日志与并发支持，需自行安装 |
| gflags | 示例程序的命令行参数解析，需自行安装 |

当前 CMake 使用 GNU 风格的 `-include`、Linux 依赖路径和库名；Windows/MSVC 需要适配。项目未约束第三方依赖的具体版本，实际兼容性需在目标环境验证。

### 构建库与示例

在本目录执行，构建前需确保第三方依赖的头文件与库可被编译器找到：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
```

默认构建库和全部 8 个示例；当前未提供关闭示例的 CMake 选项。仅编译库目标时可执行：

```bash
cmake --build build --target miao.core --parallel
cmake --build build --target miao.utils --parallel
```

产物输出到**源码目录**下：

| 位置 | 内容 |
| --- | --- |
| `lib/libmiao.core.so` | 图优化核心库 |
| `lib/libmiao.utils.so` | 示例使用的随机采样、计时工具 |
| `bin/` | 示例可执行文件 |

运行无需外部数据的圆拟合示例：

```bash
./bin/circle_fit
```

其他示例的目标名称、运行参数及数据要求见[示例指南](docs/examples.md)。

<a id="cmake"></a>

## CMake 接入

在本目录下保存应用源码（例如下面的 `quick_start.cc`），并在顶层 [CMakeLists.txt](CMakeLists.txt) 的现有 `add_subdirectory(...)` 之后添加目标：

```cmake
add_executable(miao_quick_start quick_start.cc)
target_compile_features(miao_quick_start PRIVATE cxx_std_17)
target_link_libraries(miao_quick_start PRIVATE miao.core)
```

随后构建并运行：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target miao_quick_start --parallel
./bin/miao_quick_start
```

核心求解代码链接 `miao.core`；使用本目录的采样或计时工具时，再链接 `miao.utils`。头文件采用 `core/...` 路径，正式命名空间为 `lightning::miao`，应用可自行定义别名。

当前构建脚本依赖 `CMAKE_SOURCE_DIR` 和目录级编译选项，作为其他工程的子目录引入时需要调整路径与依赖传播。现有安装规则仅安装库目标，未提供完整的头文件安装、导出目标或 `miaoConfig.cmake`，因此不能直接使用 `find_package(miao CONFIG REQUIRED)`。

<a id="quick-start"></a>

## 快速开始

下面是完整的曲线拟合程序，拟合模型为 `y = a * exp(-lambda * x) + b`。程序生成无噪声数据，真实参数为 `[a, b, lambda] = [2.0, 0.4, 0.2]`。将代码保存为 `quick_start.cc`，按上节接入即可运行。

该程序使用正确的解析雅可比；原 [curve_fit.cc](examples/data_fitting/curve_fit.cc) 中的导数问题见[已知问题](docs/limitations.md#known-issues)。

```cpp
#include <Eigen/Core>
#include <glog/logging.h>

#include <cmath>
#include <iostream>
#include <memory>

#include "core/graph/base_unary_edge.h"
#include "core/graph/base_vec_vertex.h"
#include "core/graph/optimizer.h"
#include "core/opti_algo/algo_select.h"

namespace opt = lightning::miao;

class VertexParams : public opt::BaseVecVertex<3> {};

class EdgePointOnCurve
    : public opt::BaseUnaryEdge<1, Eigen::Vector2d, VertexParams> {
 public:
    void ComputeError() override {
        const auto* vertex = static_cast<const VertexParams*>(GetVertex(0));
        const auto& p = vertex->Estimate();
        error_[0] = p[0] * std::exp(-p[2] * measurement_.x())
                    + p[1] - measurement_.y();
    }

    void LinearizeOplus() override {
        const auto* vertex = static_cast<const VertexParams*>(GetVertex(0));
        const auto& p = vertex->Estimate();
        const double x = measurement_.x();
        const double exponential = std::exp(-p[2] * x);
        jacobian_oplus_xi_ << exponential, 1.0, -p[0] * x * exponential;
    }
};

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);

    // 1. 明确指定算法、线性求解器和稠密模式。
    opt::Optimizer optimizer;
    opt::SetupOptimizer<3, -1>(optimizer, opt::OptimizerConfig(
        opt::AlgorithmType::LEVENBERG_MARQUARDT,
        opt::LinearSolverType::LINEAR_SOLVER_DENSE, true));

    // 2. 添加一个三维参数顶点。
    auto params = std::make_shared<VertexParams>();
    params->SetId(0);
    params->SetEstimate(Eigen::Vector3d(1.0, 1.0, 1.0));
    if (!optimizer.AddVertex(params)) {
        return 1;
    }

    // 3. 每个观测点对应一条一维残差边。
    for (int i = 0; i <= 100; ++i) {
        const double x = 0.1 * i;
        const double y = 2.0 * std::exp(-0.2 * x) + 0.4;
        auto edge = std::make_shared<EdgePointOnCurve>();
        edge->SetVertex(0, params);
        edge->SetMeasurement(Eigen::Vector2d(x, y));
        edge->SetInformation(Eigen::Matrix<double, 1, 1>::Identity());
        if (!optimizer.AddEdge(edge)) {
            return 1;
        }
    }

    // 4. 初始化活动图，然后迭代求解。
    if (!optimizer.InitializeOptimization()) {
        return 1;
    }
    optimizer.ComputeActiveErrors();
    const double cost_before = optimizer.ActiveRobustChi2();
    const int iterations = optimizer.Optimize(20);
    optimizer.ComputeActiveErrors();
    const double cost_after = optimizer.ActiveRobustChi2();

    // 5. 检查基本数值条件，再读取估计值。
    if (iterations < 0 || !params->Estimate().allFinite()
        || !std::isfinite(cost_before) || !std::isfinite(cost_after)
        || cost_after > cost_before) {
        return 1;
    }
    std::cout << "a b lambda: " << params->Estimate().transpose() << '\n';
    std::cout << "cost: " << cost_before << " -> " << cost_after << '\n';
    return 0;
}
```

求解结果应接近真实参数；精度受初值、迭代次数和停止阈值影响。示例仅检查基本数值条件；应用中还应检查模型、图的可观测性和残差统计，不能只以迭代计数判断成功。

`SetupOptimizer<3, -1>` 的首个模板参数表示主变量块的局部更新维度。纯 SE3 位姿图通常使用 `<6, -1>`，位姿与三维路标联合优化通常使用 `<6, 3>`；细节见 [SetupOptimizer](docs/api.md#setup)。

<a id="layout"></a>

## 目录结构

```text
miao/
├── CMakeLists.txt       # 独立构建入口
├── core/                # 图结构、优化算法、求解器、鲁棒核与内置类型
├── utils/               # 随机采样等工具
├── examples/            # 7 个子目录、8 个示例程序
├── deps/                # 从源工程复制的依赖头文件及独立构建兼容层
├── docs/                # API、示例、使用限制和局部融合指南
└── README.md
```

`deps/compat.h` 通过编译选项注入 `miao = lightning::miao` 别名；`deps/common/`、`deps/core/common/` 和 `deps/utils/` 兼容原示例的类型别名与包含路径。它们用于本副本的独立构建，更多说明见[构建兼容层](docs/examples.md#compatibility)。

<a id="documentation"></a>

## 详细文档

| 文档 | 内容 |
| --- | --- |
| [API 与扩展指南](docs/api.md) | 核心概念、配置、常用接口、自定义顶点/边、求解器和增量模式 |
| [示例指南](docs/examples.md) | 可执行目标、运行方法、数据要求与位姿图信息矩阵转换 |
| [使用限制与已知问题](docs/limitations.md) | 当前实现问题、对象生命周期与结果检查 |
| [局部融合指南](docs/local-fusion.md) | LIO/轮速建模、窗口管理、历史边缘化先验与参考资料 |

<a id="license"></a>

## 许可

本目录未包含独立的 `LICENSE` 文件，许可信息应以 lightning_lm 源项目及各依赖的许可声明为准。
